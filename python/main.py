# UNO-Twin intelligence layer (MPU / Debian Linux), run by Arduino App Lab.

import json
import math
import os
import threading
import time
from collections import deque

from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI

WS_PORT = 7788
BASELINE_WINDOW = 600      # samples, ~30 s at 20 Hz
WARMUP_SAMPLES = 40
ANOMALY_Z = 7.0            # sigma over baseline before a sample counts as hot
BASELINE_GATE = 2.5        # sigma above which a sample is not learned as normal
BROADCAST_HZ = 20.0
SENSOR_TIMEOUT = 1.5       # seconds without a sample before declaring absent
CAL_SPAN = 60.0            # uT each axis must sweep before the fit is trusted
CAL_POINTS = 900           # samples retained for the sphere fit (~45 s)
ROTATION_GATE = 30.0       # deg/s above which the ghost channel is not trusted
FREEZE_MAX = 120.0         # seconds an alert may hold the baseline frozen
ALERT_HOLD = 5             # consecutive samples over threshold before alerting
EMI_FLOOR = 3.0            # uT of field departure before a contact is plotted
EMI_FULL = 30.0            # uT of departure plotted at the centre of the scope
HEADING_SX = -1.0
HEADING_SY = -1.0
YAW_SIGN = -1.0
CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hard_iron.json")

logger = Logger("uno-twin")

_lock = threading.Lock()
_latest = None
_count = 0
_flag = False              # last on_status value from the MCU
_last_sample = 0.0
_prev_yaw = None
_prev_t = 0.0
_alert_since = 0.0
_hot_since = 0.0
_over = 0
_raw_B = 0.0

_hist_B = deque(maxlen=BASELINE_WINDOW)
_hist_raw = deque(maxlen=BASELINE_WINDOW)
_hist_ghost = deque(maxlen=BASELINE_WINDOW)
_hist_wx = deque(maxlen=BASELINE_WINDOW)
_hist_wy = deque(maxlen=BASELINE_WINDOW)

_cal_lo = [1e9] * 3
_cal_hi = [-1e9] * 3
_cal_off = [0.0] * 3
_cal_done = False
_cal_pts = deque(maxlen=CAL_POINTS)


def _wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _circ_mean(hist):
    # Arithmetic means are wrong for angles near the +/-180 wrap.
    if not hist:
        return 0.0
    s = sum(math.sin(math.radians(a)) for a in hist)
    c = sum(math.cos(math.radians(a)) for a in hist)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(s, c))


def _zscore(value, hist):
    if len(hist) < WARMUP_SAMPLES:
        return 0.0
    n = len(hist)
    mean = sum(hist) / n
    std = math.sqrt(sum((x - mean) ** 2 for x in hist) / n)
    return 0.0 if std < 1e-6 else (value - mean) / std


def _level_xy(roll_deg, pitch_deg, mx, my, mz):
    """Tilt-compensated horizontal field components in the level body frame."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    xh = mx * math.cos(p) + my * math.sin(r) * math.sin(p) + mz * math.cos(r) * math.sin(p)
    yh = my * math.cos(r) - mz * math.sin(r)
    return xh, yh


def _sphere_fit(pts):
    A = [[0.0] * 4 for _ in range(4)]
    b = [0.0] * 4
    for x, y, z in pts:
        row = (2.0 * x, 2.0 * y, 2.0 * z, 1.0)
        rhs = x * x + y * y + z * z
        for i in range(4):
            for j in range(4):
                A[i][j] += row[i] * row[j]
            b[i] += row[i] * rhs

    for c in range(4):
        p = max(range(c, 4), key=lambda r: abs(A[r][c]))
        if abs(A[p][c]) < 1e-9:
            return None
        A[c], A[p] = A[p], A[c]
        b[c], b[p] = b[p], b[c]
        for r in range(4):
            if r == c:
                continue
            f = A[r][c] / A[c][c]
            for k in range(c, 4):
                A[r][k] -= f * A[c][k]
            b[r] -= f * b[c]
    return [b[i] / A[i][i] for i in range(3)]


def _hard_iron(m):
    global _cal_done
    if _cal_done:
        return
    _cal_pts.append(m)
    for i in range(3):
        _cal_lo[i] = min(_cal_lo[i], m[i])
        _cal_hi[i] = max(_cal_hi[i], m[i])

    if len(_cal_pts) < CAL_POINTS // 3:
        return
    if not all(_cal_hi[i] - _cal_lo[i] >= CAL_SPAN for i in range(3)):
        return

    centre = _sphere_fit(_cal_pts)
    if centre is None:
        return
    _cal_off[:] = centre
    _cal_done = True
    _save_cal()
    # |B| steps by tens of uT here, so the old baseline is invalid.
    _clear_hist()
    logger.info("hard-iron calibrated: %+.1f %+.1f %+.1f uT", *_cal_off)


def _clear_hist():
    global _hot_since
    _hot_since = 0.0
    for h in (_hist_B, _hist_raw, _hist_ghost, _hist_wx, _hist_wy):
        h.clear()


def _save_cal():
    try:
        with open(CAL_FILE, "w") as f:
            json.dump({"hard_iron": _cal_off}, f)
    except OSError as e:
        logger.warning("could not save calibration: %s", e)


def _load_cal():
    global _cal_done
    try:
        with open(CAL_FILE) as f:
            off = json.load(f)["hard_iron"]
    except (OSError, ValueError, KeyError):
        return
    if isinstance(off, list) and len(off) == 3:
        _cal_off[:] = [float(v) for v in off]
        _cal_done = True
        logger.info("hard-iron restored: %+.1f %+.1f %+.1f uT", *_cal_off)


def _recalibrate():
    global _cal_done
    _cal_done = False
    _cal_pts.clear()
    for i in range(3):
        _cal_lo[i], _cal_hi[i], _cal_off[i] = 1e9, -1e9, 0.0
    _clear_hist()
    try:
        os.remove(CAL_FILE)
    except OSError:
        pass
    logger.info("calibration reset")
    return {"calibrated": False}


def _sensor_live():
    return _flag and (time.time() - _last_sample) < SENSOR_TIMEOUT


def on_status(present):
    global _flag
    new = bool(int(present))
    if new != _flag:
        logger.info("sensor %s", "detected" if new else "lost")
        if not new:
            _reset_baseline()
    _flag = new


def _reset_baseline():
    global _latest, _count, _prev_yaw
    with _lock:
        _clear_hist()
        _latest = None
        _count = 0
        _prev_yaw = None


def on_sample(roll, pitch, yaw, mx, my, mz):
    global _latest, _count, _last_sample, _prev_yaw, _prev_t, _alert_since, _raw_B, _over, _hot_since

    now = time.time()
    yaw = (YAW_SIGN * yaw) % 360.0
    _hard_iron((mx, my, mz))
    _raw_B = math.sqrt(mx * mx + my * my + mz * mz)
    if _cal_done:
        cx, cy, cz = mx - _cal_off[0], my - _cal_off[1], mz - _cal_off[2]
    else:
        cx, cy, cz = mx, my, mz

    B = math.sqrt(cx * cx + cy * cy + cz * cz)
    xh, yh = _level_xy(roll, pitch, cx, cy, cz)
    mhead = math.degrees(math.atan2(HEADING_SY * yh, HEADING_SX * xh))
    raw_delta = _wrap180(yaw - mhead)

    ry = -math.radians(yaw)
    wx = xh * math.cos(ry) - yh * math.sin(ry)
    wy = xh * math.sin(ry) + yh * math.cos(ry)

    rate = 0.0
    if _prev_yaw is not None and now > _prev_t:
        rate = abs(_wrap180(yaw - _prev_yaw)) / (now - _prev_t)
    _prev_yaw, _prev_t = yaw, now

    baseline = _circ_mean(_hist_raw) if _hist_raw else raw_delta
    ghost = _wrap180(raw_delta - baseline)

    z_B = _zscore(B, _hist_B)
    z_d = _zscore(ghost, _hist_ghost) if rate < ROTATION_GATE else 0.0
    score = max(max(z_B, 0.0), abs(z_d))

    warm = len(_hist_B) >= WARMUP_SAMPLES
    _over = _over + 1 if score > ANOMALY_Z else 0
    alert = bool(warm and _cal_done and _over >= ALERT_HOLD)

    if alert:
        if _alert_since == 0.0:
            _alert_since = now
        frozen = (now - _alert_since) < FREEZE_MAX
    else:
        _alert_since = 0.0
        frozen = False

    if score > BASELINE_GATE:
        if _hot_since == 0.0:
            _hot_since = now
        gated = (now - _hot_since) < FREEZE_MAX
    else:
        _hot_since = 0.0
        gated = False

    if not (frozen or gated):
        _hist_B.append(B)
        _hist_raw.append(raw_delta)
        _hist_ghost.append(ghost)
        _hist_wx.append(wx)
        _hist_wy.append(wy)

    # Contact: departure of the world-frame field vector from its baseline.
    contact = None
    if warm and _cal_done and _hist_wx:
        dx = wx - sum(_hist_wx) / len(_hist_wx)
        dy = wy - sum(_hist_wy) / len(_hist_wy)
        strength = math.hypot(dx, dy)
        if strength >= EMI_FLOOR:
            brg = math.degrees(math.atan2(HEADING_SY * dy, HEADING_SX * dx))
            contact = {
                "bearing": round(brg % 360.0, 1),
                "relative": round(_wrap180(brg - yaw) % 360.0, 1),
                "strength": round(strength, 2),
                "proximity": round(min(1.0, (strength - EMI_FLOOR) / (EMI_FULL - EMI_FLOOR)), 3),
            }

    sample = {
        "t": now,
        "sensor": True,
        "warm": warm,
        "calibrated": _cal_done,
        "cal_progress": round(min(1.0, min(_cal_hi[i] - _cal_lo[i] for i in range(3)) / CAL_SPAN), 2)
                        if _cal_pts and not _cal_done else (1.0 if _cal_done else 0.0),
        "yaw_rate": round(rate, 1),
        "orientation": {"roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 2)},
        "mag": {"x": round(cx, 2), "y": round(cy, 2), "z": round(cz, 2), "B": round(B, 2)},
        "mag_heading": round(mhead % 360.0, 2),
        "ghost_delta": round(ghost, 2),
        "contact": contact,
        "anomaly": {
            "z_B": round(z_B, 2),
            "z_delta": round(z_d, 2),
            "score": round(score, 2),
            "alert": alert,
        },
    }

    with _lock:
        _latest = sample
        _count += 1
        _last_sample = now


def _frame():
    with _lock:
        if _sensor_live() and _latest:
            return _latest
        return {"t": time.time(), "sensor": False}


ui = WebUI()
ui.expose_api("GET", "/latest", _frame)
ui.expose_api("GET", "/recalibrate", _recalibrate)
ui.expose_api("GET", "/health", lambda: {
    "sensor": _sensor_live(),
    "calibrated": _cal_done,
    "hard_iron": [round(v, 2) for v in _cal_off],
    "raw_B": round(_raw_B, 2),
    "cal_span": [round(_cal_hi[i] - _cal_lo[i], 1) if _cal_pts else 0.0 for i in range(3)],
    "cal_target": CAL_SPAN,
    "samples": _count,
    "baseline_fill": len(_hist_B),
    "baseline_window": BASELINE_WINDOW,
})
ui.on_connect(lambda sid: ui.send_message("telemetry", _frame()))


# Raw WebSocket mirror for external clients and capture tooling.
def _start_raw_ws():
    try:
        import asyncio
        import websockets
    except Exception as exc:
        logger.warning("raw WebSocket disabled (%s)", exc)
        return

    clients = set()

    async def handler(ws):
        clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)

    async def pump():
        period = 1.0 / BROADCAST_HZ
        last = None
        while True:
            await asyncio.sleep(period)
            if not clients:
                continue
            f = _frame()
            key = (f["sensor"], f["t"])
            if key == last:
                continue
            last = key
            msg = json.dumps(f)
            await asyncio.gather(*(c.send(msg) for c in list(clients)),
                                 return_exceptions=True)

    async def main():
        try:
            async with websockets.serve(handler, "0.0.0.0", WS_PORT):
                logger.info("raw telemetry WebSocket on :%d", WS_PORT)
                await pump()
        except Exception as exc:
            logger.warning("raw WebSocket failed: %s", exc)

    asyncio.run(main())


threading.Thread(target=_start_raw_ws, daemon=True).start()

Bridge.provide("on_sample", on_sample)
Bridge.provide("on_status", on_status)

_last_key = None


def loop():
    global _last_key
    time.sleep(1.0 / BROADCAST_HZ)
    f = _frame()
    key = (f["sensor"], f["t"])
    if key == _last_key:
        return
    _last_key = key
    try:
        ui.send_message("telemetry", f)
    except Exception:
        pass


logger.info("UNO-Twin intelligence layer starting")
_load_cal()
App.run(user_loop=loop)

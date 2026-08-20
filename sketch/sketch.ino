// UNO-Twin sensor node (MCU / STM32U585).
// Reads a Bosch BNO055 over the UNO Q Qwiic connector and pushes fused
// orientation plus raw magnetic flux to the Linux side over the Bridge.
//
// Bridge contract:
//   on_sample(roll, pitch, yaw, mx, my, mz)   deg, deg, deg, uT, uT, uT
//   on_status(present)                        1 = sensor live, 0 = absent

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MadgwickAHRS.h>

const float FUSION_HZ    = 100.0f;
const float TELEMETRY_HZ = 20.0f;

const unsigned long FUSION_PERIOD_US    = (unsigned long)(1000000.0f / FUSION_HZ);
const unsigned long TELEMETRY_PERIOD_US = (unsigned long)(1000000.0f / TELEMETRY_HZ);
const unsigned long STATUS_PERIOD_US    = 2000000UL;
const unsigned long RESCAN_PERIOD_US    = 3000000UL;

Madgwick filter;
Adafruit_BNO055 *bno = nullptr;
bool sensorOk = false;

unsigned long lastFusionUs = 0, lastTelemetryUs = 0, lastStatusUs = 0, lastScanUs = 0;

static bool probeAddr(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return bus.endTransmission() == 0;
}

// Qwiic is routed to Wire1 on the UNO Q; Wire is probed as a fallback.
static bool findSensor(TwoWire *&busOut, uint8_t &addrOut) {
  TwoWire *buses[2] = { &Wire1, &Wire };
  const uint8_t addrs[2] = { 0x28, 0x29 };

  for (uint8_t b = 0; b < 2; b++) {
    buses[b]->begin();
    buses[b]->setClock(400000);
    for (uint8_t a = 0; a < 2; a++) {
      if (probeAddr(*buses[b], addrs[a])) {
        busOut = buses[b];
        addrOut = addrs[a];
        return true;
      }
    }
  }
  return false;
}

// AMG mode keeps the magnetometer powered; on-chip fusion modes shut it down.
static bool attachSensor() {
  TwoWire *bus = nullptr;
  uint8_t addr = 0x28;
  if (!findSensor(bus, addr)) return false;

  delete bno;
  bno = new Adafruit_BNO055(55, addr, bus);
  if (!bno->begin(OPERATION_MODE_AMG)) {
    Monitor.println("BNO055 present on I2C but begin() failed");
    delete bno;
    bno = nullptr;
    return false;
  }
  bno->setExtCrystalUse(true);
  Monitor.print("BNO055 up in AMG mode at 0x");
  Monitor.println(addr, HEX);
  return true;
}

void setup() {
  Monitor.begin();
  Bridge.begin();
  filter.begin(FUSION_HZ);
  sensorOk = attachSensor();
  if (!sensorOk) Monitor.println("No BNO055 detected on Qwiic");
}

void loop() {
  const unsigned long nowUs = micros();

  if (!sensorOk) {
    if (nowUs - lastScanUs >= RESCAN_PERIOD_US) {
      lastScanUs = nowUs;
      sensorOk = attachSensor();
    }
    if (nowUs - lastStatusUs >= STATUS_PERIOD_US) {
      lastStatusUs = nowUs;
      Bridge.notify("on_status", 0);
    }
    delay(1);
    return;
  }

  if (nowUs - lastFusionUs >= FUSION_PERIOD_US) {
    lastFusionUs = nowUs;
    imu::Vector<3> acc  = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // getVector already scales the gyro to deg/s, which is what Madgwick expects.
    filter.updateIMU(gyro.x(), gyro.y(), gyro.z(), acc.x(), acc.y(), acc.z());
  }

  if (nowUs - lastTelemetryUs >= TELEMETRY_PERIOD_US) {
    lastTelemetryUs = nowUs;
    imu::Vector<3> mag = bno->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    Bridge.notify("on_sample", filter.getRoll(), filter.getPitch(), filter.getYaw(),
                  (float)mag.x(), (float)mag.y(), (float)mag.z());
  }

  if (nowUs - lastStatusUs >= STATUS_PERIOD_US) {
    lastStatusUs = nowUs;
    Bridge.notify("on_status", 1);
  }
}

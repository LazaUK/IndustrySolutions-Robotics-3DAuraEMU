# UNO-Twin: 3D Sensing Aura for EMI-Sensitive Robotics

This repo contains the source code for **UNO-Twin** app, a real-time digital twin that detects electromagnetic interference (EMI) and visualise it as a 3D "sensing aura".

Invisible electromagnetic hazards may cause robot sensors to drift and their navigation to fail. Using an *Arduino UNO Q* and a *Bosch BNO055* 9-axis sensor, this solution mirrors the robot's live orientation and flags interference risks in realtime.

> [!TIP]
> This project was built for the [Arduino UNO Q Hackathon](https://www.hackster.io/contests/arduino-uno-q). Qualcomm Arduino Uno Q board (with 4G RAM) was kindly provided by Hackster.io team, thank you!

## 📑 Table of Contents

- [The Invisible Hazard Problem](#the-invisible-hazard-problem)
- [Hardware Setup](#hardware-setup)
- [Dual-Brain Architecture](#dual-brain-architecture)
- [Detection Layer: Hard-Iron Calibration](#detection-layer-hard-iron-calibration)
- [Detection Layer: Anomaly Scoring](#detection-layer-anomaly-scoring)
- [Visualisation Layer: The 3D Twin](#visualisation-layer-the-3d-twin)
- [Deploying Without Installing Anything](#deploying-without-installing-anything)
- [Repository Layout](#repository-layout)
- [Demos & Results](#demos--results)

# Industrial Robotics: Digital Twin with 3D EMI Aura

This repo demonstrates how to use Arduino Uno Q to "sense" electromagnetic inference (EMI) hazards. Uno Q board is a dual-brain solution, that has both MPU (from Qualcomm) and MCU (from ST Microelectronic) processing capabilities. MCU can be used to process sensor data real-time, while MPU can host a user-facing app to visualise interactive content.

> [!NOTE]
> 

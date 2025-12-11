# Beach Robot – ESP32 ↔ ROS 2 (Jetson) Communication Protocol

This document describes the communication protocol between:

- **Jetson (ROS 2)**: running the `esp32_bridge` node  
- **ESP32**: running the custom firmware for wheel control, IMU and buzzer

The goal is to make it easy to understand, debug, or re-implement either side.

---

## Jetson → ESP32
```json
{"wheel_cmd":[v_fl, v_fr, v_rl, v_rr]}
```
- 4 floats, order: `[front_left, front_right, rear_left, rear_right]` in m/s

```json
{"buzzer_duration": duration}
```
- `duration` (float) in seconds

```json
{"vibration_enable": bool}
```
- `true` → turn ON vibration motor
- `false` → turn OFF vibration motor

---

## ESP32 → Jetson
```json
{
  "enc_vel":[v_fl, v_fr, v_rl, v_rr],
  "imu_quat":[x,y,z,w],
  "imu_gyro":[gx,gy,gz],
  "imu_lin_acc":[ax,ay,az]
}
```
- `enc_vel`: wheel velocities in m/s
- `imu_quat`: orientation as quaternion
- `imu_gyro`: angular velocity
- `imu_lin_acc`: linear acceleration

```json
{"info":"ESP32 comm started"}
```
- purely for logging/debugging in the bridge node
# Wheel Response Test

Use this test with the robot lifted safely. The runner publishes `/cmd_vel` test
steps and records high-level ROS commands plus low-level ESP32 debug feedback to
one CSV file.

## Build

```bash
colcon build --packages-select beach_robot_esp32_bridge beach_robot_bringup --symlink-install
source install/setup.bash
```

## Launch

Start the normal hardware bridge and wheel mixer. Teleop is not required for this
script because it publishes `/cmd_vel` directly.

```bash
ros2 launch beach_robot_esp32_bridge esp32_bridge.launch.py
ros2 launch beach_wheel_mixer wheel_mps_mixer.launch.py
```

## Run

```bash
ros2 run beach_robot_bringup wheel_response_test --label lifted_after_fix
```

The default sequence is:

- forward
- backward
- spin_left
- spin_right
- curve_left
- curve_right

CSV and metadata files are written to:

```text
~/beach_robot_logs/wheel_response/
```

## Useful Options

```bash
ros2 run beach_robot_bringup wheel_response_test \
  --label lifted_slow \
  --forward-speed 0.20 \
  --spin-rate 0.30 \
  --curve-speed 0.18 \
  --curve-rate 0.22 \
  --step-sec 5.0
```

Run only spin tests:

```bash
ros2 run beach_robot_bringup wheel_response_test --tests spin_left,spin_right
```

Analyze a CSV summary:

```bash
ros2 run beach_robot_bringup wheel_response_analyze \
  ~/beach_robot_logs/wheel_response/wheel_response_YYYYMMDD_HHMMSS_lifted_after_fix.csv
```

## Logged Signals

- commanded `/cmd_vel` from the runner
- high-level `/wheel_cmd` from `wheel_mps_mixer`
- ROS `/enc_vel` from `esp32_bridge`
- low-level `/esp32/debug` fields from `esp32_main`
- low-level firmware `wheel_cmd`, `enc_vel_raw`, `enc_vel_corr`, `u_ff`,
  `u_pid`, `motor_u`, PID gains, VMAX, and timing ages

If `/esp32/debug` columns are empty, confirm the updated `esp32_bridge` is
running and that the ESP32 firmware supports `{"dbg_enable": true}`.

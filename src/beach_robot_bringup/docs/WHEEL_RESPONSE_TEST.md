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

The default profile is intentionally gentle for lifted-wheel testing:

- forward/backward linear speed: `0.17 m/s`
- spin-in-place angular speed: `0.20 rad/s`
- curve linear/angular speed: `0.12 m/s`, `0.08 rad/s`

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

## Joy Floor Test

For floor testing with the joystick, run the bridge, mixer, teleop, and recorder
as separate terminals. The recorder only listens in `--manual-record` mode; it
does not publish `/cmd_vel`, so the joystick remains the only motion source.

```bash
ros2 launch beach_robot_esp32_bridge esp32_bridge.launch.py
ros2 launch beach_wheel_mixer wheel_mps_mixer.launch.py
ros2 launch beach_robot_teleop teleop.launch.py
```

Record a short manual drive:

```bash
ros2 run beach_robot_bringup wheel_response_test \
  --manual-record \
  --label joy_floor_001 \
  --record-sec 90 \
  --log-rate-hz 5 \
  --no-esp32-debug
```

Suggested joystick sequence:

- forward at low stick for 3-5 seconds
- backward at low stick for 3-5 seconds
- curve left and curve right while moving forward
- spin left and spin right briefly
- stop between motions so the CSV has clean sections

Analyze with ROS encoder data when debug is disabled:

```bash
ros2 run beach_robot_bringup wheel_response_analyze \
  ~/beach_robot_logs/wheel_response/wheel_response_YYYYMMDD_HHMMSS_joy_floor_001.csv \
  --enc-source ros \
  --max-enc-mps 2.0
```

If we need firmware internals for a specific issue, re-run with debug enabled at
a slow rate such as `--debug-rate-ms 1000` instead of `--no-esp32-debug`.

## Useful Options

```bash
ros2 run beach_robot_bringup wheel_response_test \
  --label lifted_slow \
  --forward-speed 0.17 \
  --spin-rate 0.20 \
  --curve-speed 0.12 \
  --curve-rate 0.08 \
  --debug-rate-ms 250 \
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

If `cmdbad%` is high, the ESP32 did not receive/hold the same wheel command
that ROS published. Re-run with a slower debug stream such as
`--debug-rate-ms 500` before tuning motor constants from that log.

## Logged Signals

- commanded `/cmd_vel` from the runner
- high-level `/wheel_cmd` from `wheel_mps_mixer`
- ROS `/enc_vel` from `esp32_bridge`
- low-level `/esp32/debug` fields from `esp32_main`
- low-level firmware `wheel_cmd`, `enc_vel_raw`, `enc_vel_corr`, `u_ff`,
  `u_pid`, `motor_u`, PID gains, VMAX, and timing ages

If `/esp32/debug` columns are empty, confirm the updated `esp32_bridge` is
running and that the ESP32 firmware supports `{"dbg_enable": true}`.

# Odometry Localization Tests

This package has a side-by-side odometry test launch for comparing:

- Raw wheel encoder odometry: `/wheel/odom`
- Wheel-only EKF: `/odometry/wheel_only`
- IMU-only EKF: `/odometry/imu_only`
- Wheel + IMU EKF: `/odometry/fusion`

The test EKFs publish no TF, so all modes can run together without fighting over
`odom -> base_link`. Use the normal `localization.launch.py` for the production
TF used by Nav2.

## Build

```bash
cd ~/beach_robot_ws
colcon build --symlink-install --packages-select beach_robot_localization
source install/setup.bash
```

## Hardware inputs

Start the ESP32 bridge in one terminal:

```bash
ros2 launch beach_robot_esp32_bridge esp32_bridge.launch.py port:=/dev/esp32_beach
```

Check that the two required sensor topics are alive:

```bash
ros2 topic hz /enc_vel
ros2 topic hz /imu/data
```

## Run all odometry comparisons

```bash
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=all
```

Run one mode at a time if you want cleaner logs:

```bash
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=wheel
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=imu
ros2 launch beach_robot_localization localization_odom_test.launch.py mode:=fusion
```

## Record a bag

```bash
ros2 bag record \
  /enc_vel /imu/data /wheel/odom \
  /odometry/wheel_only /odometry/imu_only /odometry/fusion \
  /tf_static
```

## Suggested checks

1. Static test, 30 seconds:
   `/wheel/odom`, `/odometry/wheel_only`, and `/odometry/fusion` should stay
   close to zero. `/odometry/imu_only` may drift.

2. Straight test, measured distance:
   Drive forward 1-2 m and compare x distance. Wheel-only and fusion should
   roughly match tape distance.

3. In-place yaw test:
   Rotate 90 or 360 degrees. Confirm wheel yaw sign matches IMU yaw sign.

4. Square path:
   Fusion should usually close better than wheel-only if the IMU yaw is stable.

IMU-only x/y is diagnostic only. A low-cost IMU will drift quickly because
position comes from double-integrating acceleration.


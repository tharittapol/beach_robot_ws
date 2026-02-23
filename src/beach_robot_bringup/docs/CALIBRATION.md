# Calibration

## 1) Motor direction
- Lift wheels
- Publish /cmd_vel forward
- All wheels must spin forward
- Fix using wheel_mixer params: dir_fl/dir_fr/dir_rl/dir_rr

## 2) Wheel radius
Drive 10m straight (slow).
Compare odom distance vs tape measure.
Adjust wheel_radius until error < 3%.

## 3) Track width
Spin in place 360 degrees.
Compare odom yaw vs real rotation.
Adjust track_width until yaw error < 5 degrees per 360.

## 4) IMU orientation
Ensure IMU axes match ROS convention.
If yaw drifts badly at standstill, check IMU mounting and biases.

## 5) GNSS usage
Use GNSS outdoors only.
Check /gps/fix covariance and update rate.
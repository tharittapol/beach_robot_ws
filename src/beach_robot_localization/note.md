ros2 launch beach_robot_localization localization_full_test.launch.py \
  use_zed:=true \
  use_gnss:=true \
  esp32_port:=/dev/ttyESP32 \
  wheel_cmd_send_rate_hz:=20.0 \
  publish_raw_json:=false \
  linear_scale:=1.53 \
  angular_scale:=1.0 \
  mixer_params_file:=$HOME/beach_robot_ws/src/beach_wheel_mixer/config/mixer.yaml

mkdir -p ~/beach_robot_logs/report_pose
STAMP=$(date +%Y%m%d_%H%M%S)

ros2 bag record -o ~/beach_robot_logs/report_pose/straight_10m_${STAMP} \
  /cmd_vel /wheel_cmd \
  /wheel/odom \
  /odometry/bno_imu_only /odometry/zed_imu_only \
  /odometry/fusion_bno /odometry/fusion_zed \
  /gps/fix /gps/fix_quality /odometry/gps \
  /imu/data /zed/zed_node/imu/data /tf_static

BAG=~/beach_robot_logs/report_pose/straight_10m_YYYYMMDD_HHMMSS

ros2 run beach_robot_bringup localization_pose_report "$BAG" \
  --trial-id straight_10m_001 \
  --straight-distance-m 10.0 \
  --meter-step-m 1.0 \
  --out ~/beach_robot_logs/report_pose/straight_10m_summary.csv \
  --meters-out ~/beach_robot_logs/report_pose/straight_10m_every_1m.csv \
  --plot-out ~/beach_robot_logs/report_pose/straight_10m_pose.svg

ros2 launch beach_robot_localization localization_full_test.launch.py \
  use_teleop:=false \
  use_zed:=true \
  use_gnss:=true \
  esp32_port:=/dev/ttyESP32 \
  wheel_cmd_send_rate_hz:=20.0 \
  publish_raw_json:=false \
  linear_scale:=1.53 \
  angular_scale:=1.0 \
  mixer_params_file:=$HOME/beach_robot_ws/src/beach_wheel_mixer/config/mixer.yaml

ros2 topic pub -r 20 --times 1334 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15}, angular: {z: 0.0}}"

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

ros2 topic pub -r 20 --times 1177 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.17}, angular: {z: 0.0}}"

# beach_robot_ws

## build
```bash
cd ~/beach_robot_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## run bringup
```bash
ros2 launch beach_robot_bringup hardware_bringup.launch.py
```

## PASS/FAIL
- ros2 topic list | grep -E "enc_vel|imu/data|cmd_vel|wheel_cmd"
- ros2 run tf2_ros tf2_echo map base_link
- เปิด RViz ด้วย config
```bash
rviz2 -d $(ros2 pkg prefix beach_robot_bringup)/share/beach_robot_bringup/rviz/bringup.rviz
```
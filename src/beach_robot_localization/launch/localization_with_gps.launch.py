from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('beach_robot_localization')

    ekf_local_cfg = os.path.join(pkg, 'config', 'ekf_local.yaml')
    ekf_global_cfg = os.path.join(pkg, 'config', 'ekf_global.yaml')
    navsat_cfg = os.path.join(pkg, 'config', 'navsat.yaml')

    return LaunchDescription([
        # wheel odometry (enc_vel -> wheel/odom)
        Node(
            package='beach_robot_localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{'track_width': 0.8}],
        ),

        # EKF local (wheel + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local_cfg],
            remappings=[
                # assume ekf_local publish as /odometry/filtered
                ('/odometry/filtered', '/odometry/local'),
            ],
        ),

        # navsat_transform: /fix + /imu/data + /odometry/local -> /odometry/gps
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_cfg],
            remappings=[
                ('imu', 'imu/data'),
                ('gps/fix', 'fix'),
                ('odometry/filtered', 'odometry/local'),
                ('odometry/gps', 'odometry/gps'),
            ],
        ),

        # EKF global (GPS + IMU) -> /odometry/global + TF map->odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global_cfg],
            remappings=[
                ('/odometry/filtered', '/odometry/global'),
            ],
        ),
    ])

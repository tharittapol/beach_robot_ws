from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('beach_robot_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')

    return LaunchDescription([
        # Wheel odometry from enc_vel
        Node(
            package='beach_robot_localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{'track_width': 0.8}],  # <-- tune this value as needed
        ),

        # EKF fusion (wheel + IMU)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_config],
        ),
    ])

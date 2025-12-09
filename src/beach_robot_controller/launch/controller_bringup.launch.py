from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beach_robot_controller',
            executable='buzzer_action_server',
            name='buzzer_action_server',
            output='screen',
        ),
    ])

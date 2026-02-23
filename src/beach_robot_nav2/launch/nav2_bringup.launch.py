from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                FindPackageShare('beach_robot_nav2').find('beach_robot_nav2'),
                'config', 'nav2_params.yaml'
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])
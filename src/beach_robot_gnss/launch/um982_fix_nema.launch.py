from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gga_send_period = LaunchConfiguration('gga_send_period')
    force_gpgga = LaunchConfiguration('force_gpgga')

    return LaunchDescription([
        DeclareLaunchArgument('gga_send_period', default_value='0.5'),
        DeclareLaunchArgument('force_gpgga', default_value='true'),

        Node(
            package='beach_robot_gnss',
            executable='um982_bridge_node',
            name='um982_ntrip_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyGNSS',
                'baud': 115200,
                'frame_id': 'gps_link',

                'ntrip_host': 'rtk2go.com',
                'ntrip_port': 2101,
                'mountpoint': 'SRPDI1968',

                # rtk2go: email conversion + password none
                'username': 'wachiramate-at-gmail-d-com',
                'password': 'none',
                'user_agent': 'NTRIP UM982/1.0',

                'gga_send_period': gga_send_period,
                'force_gpgga': force_gpgga,
            }],
        ),
    ])

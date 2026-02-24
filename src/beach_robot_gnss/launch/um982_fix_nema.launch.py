from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
                'mountpoint': 'Pump236',

                # rtk2go: email conversion + password none
                'username': 'wachiramate-at-gmail-d-com',
                'password': 'none',

                'gga_send_period': 0.5
            }],
        ),
    ])

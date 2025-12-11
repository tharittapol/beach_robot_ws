from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_nmea_driver',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',      # adjust as needed
                'baud': 115200,
                'frame_id': 'gps_link',      # frame of GNSS on robot
                'time_ref_source': 'gps',
                'use_RMC': False,            # Primarily uses GGA, adjust as needed
            }]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # UM982 NMEA output port
    port = '/dev/ttyACM2'
    baud = 115200
    frame_id = 'gps_link'   # must exist in TF

    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='um982_nmea_fix',
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
                'frame_id': frame_id,
                'useRMC': False,      # keep False unless you know you want RMC for time/vel
                'time_ref_source': 'gps'
            }],
            remappings=[
                # default output is /fix
                ('fix', 'gps/fix'),
            ],
        ),
    ])

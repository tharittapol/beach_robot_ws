import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


class Teleop4WDSkid(Node):
    def __init__(self):
        super().__init__('teleop_4wd')

        # Parameters
        self.declare_parameter('max_linear', 0.5)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('track_width', 0.5)  # m, Left-right wheel distance
        self.declare_parameter('axis_linear', 1)    # joystick axis index
        self.declare_parameter('axis_angular', 0)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)
        # Per-wheel scale
        self.declare_parameter('front_left_scale', 1.0)
        self.declare_parameter('rear_left_scale', 1.0)
        self.declare_parameter('front_right_scale', 1.0)
        self.declare_parameter('rear_right_scale', 1.0)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.axis_linear = int(self.get_parameter('axis_linear').value)
        self.axis_angular = int(self.get_parameter('axis_angular').value)
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)
        self.front_left_scale = float(self.get_parameter('front_left_scale').value)
        self.rear_left_scale = float(self.get_parameter('rear_left_scale').value)
        self.front_right_scale = float(self.get_parameter('front_right_scale').value)
        self.rear_right_scale = float(self.get_parameter('rear_right_scale').value)

        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.pub_wheel_cmd = self.create_publisher(
            Float32MultiArray,
            'wheel_cmd',
            10
        )

        self.get_logger().info('Teleop4WDSkid node started (front = tracks, rear = wheels)')

    def joy_callback(self, msg: Joy):
        # read joystick axes values
        try:
            raw_lin = msg.axes[self.axis_linear]
            raw_ang = msg.axes[self.axis_angular]
        except IndexError:
            self.get_logger().warn('Joy axes index out of range')
            return

        # scaling
        v = raw_lin * self.scale_linear * self.max_linear
        w = raw_ang * self.scale_angular * self.max_angular

        # calculate left/right speed for skid-steer
        v_left = v - (w * self.track_width / 2.0)
        v_right = v + (w * self.track_width / 2.0)

        # apply per-wheel scale
        v_fl = v_left * self.front_left_scale
        v_rl = v_left * self.rear_left_scale
        v_fr = v_right * self.front_right_scale
        v_rr = v_right * self.rear_right_scale


        wheel_cmd = Float32MultiArray()
        # [front_left, front_right, rear_left, rear_right]
        wheel_cmd.data = [v_fl, v_fr, v_rl, v_rr]

        self.pub_wheel_cmd.publish(wheel_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Teleop4WDSkid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

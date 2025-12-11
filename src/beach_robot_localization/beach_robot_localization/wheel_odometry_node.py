import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class WheelOdometry(Node):
    """
    Integrate wheel velocities from 'enc_vel' to produce 2D odometry.

    Input:
      enc_vel (Float32MultiArray) = [v_fl, v_fr, v_rl, v_rr] (m/s)

    Output:
      wheel/odom (nav_msgs/Odometry)
      TF: odom -> base_link
    """

    def __init__(self):
        super().__init__('wheel_odometry')

        # Track width (distance between left and right wheel groups) in meters
        self.declare_parameter('track_width', 0.8)
        self.track_width = float(self.get_parameter('track_width').value)

        # 2D pose state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = None

        self.odom_pub = self.create_publisher(
            Odometry,
            'wheel/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub_enc = self.create_subscription(
            Float32MultiArray,
            'enc_vel',
            self.enc_callback,
            10
        )

        self.get_logger().info(
            f'WheelOdometry node started (track_width = {self.track_width:.3f} m)'
        )

    def enc_callback(self, msg: Float32MultiArray):
        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            return

        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        data = list(msg.data)
        if len(data) < 4:
            self.get_logger().warn('enc_vel must have 4 elements [v_fl,v_fr,v_rl,v_rr]')
            return

        v_fl, v_fr, v_rl, v_rr = data[0], data[1], data[2], data[3]

        # Left/right average (skid-steer)
        v_left = 0.5 * (v_fl + v_rl)
        v_right = 0.5 * (v_fr + v_rr)

        # Robot linear and angular velocity
        v = 0.5 * (v_left + v_right)              # m/s
        w = (v_right - v_left) / self.track_width # rad/s

        # Integrate pose in odom frame
        self.yaw += w * dt
        # normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        # simple covariances (tune later)
        # x, y, yaw
        odom.pose.covariance = [
            0.05, 0,    0, 0, 0,    0,
            0,    0.05, 0, 0, 0,    0,
            0,    0,    9999, 0, 0, 0,
            0,    0,    0, 9999, 0, 0,
            0,    0,    0, 0, 9999, 0,
            0,    0,    0, 0, 0,    0.2,
        ]
        odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                 0, 0.1, 0, 0, 0, 0,
                                 0, 0, 9999, 0, 0, 0,
                                 0, 0, 0, 9999, 0, 0,
                                 0, 0, 0, 0, 9999, 0,
                                 0, 0, 0, 0, 0, 0.3]

        self.odom_pub.publish(odom)

        # Also broadcast TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

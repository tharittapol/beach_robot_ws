import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


class WheelOdometryNode(Node):
    """
    Input:
      /enc_vel  std_msgs/Float32MultiArray [FL, FR, RL, RR]  (m/s)

    Output:
      /wheel/odom nav_msgs/Odometry (odom -> base_link pose + twist)
    """

    def __init__(self):
        super().__init__('wheel_odometry')

        # ---- Parameters ----
        self.declare_parameter('enc_vel_topic', 'enc_vel')
        self.declare_parameter('odom_topic', 'wheel/odom')

        self.declare_parameter('track_width', 0.60)  # meters (left-right distance)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_rate', 50.0)  # Hz publish odom even if slow enc_vel
        self.declare_parameter('timeout_sec', 0.5)     # if no enc_vel, use v=0

        self.enc_vel_topic = self.get_parameter('enc_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.track_width = float(self.get_parameter('track_width').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_enc_time = None
        self.last_update_time = self.get_clock().now()

        # latest wheel velocities
        self.v_fl = 0.0
        self.v_fr = 0.0
        self.v_rl = 0.0
        self.v_rr = 0.0

        # ---- ROS I/O ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            self.enc_vel_topic,
            self.enc_cb,
            20
        )

        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            f'Wheel odometry: sub={self.enc_vel_topic} pub={self.odom_topic} track_width={self.track_width}'
        )

    def enc_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('enc_vel must contain 4 elements [FL, FR, RL, RR]')
            return

        self.v_fl = float(msg.data[0])
        self.v_fr = float(msg.data[1])
        self.v_rl = float(msg.data[2])
        self.v_rr = float(msg.data[3])

        self.last_enc_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        # If enc_vel is stale, stop
        if self.last_enc_time is None:
            v_left = 0.0
            v_right = 0.0
        else:
            age = (now - self.last_enc_time).nanoseconds / 1e9
            if age > self.timeout_sec:
                v_left = 0.0
                v_right = 0.0
            else:
                # skid left/right speed = average of front & rear on each side
                v_left = 0.5 * (self.v_fl + self.v_rl)
                v_right = 0.5 * (self.v_fr + self.v_rr)

        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / max(self.track_width, 1e-6)

        # integrate pose in odom
        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))  # wrap

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        # (optional) put reasonable covariances
        # smaller = more trusted; adjust later
        odom.pose.covariance[0] = 0.05 * 0.05   # x
        odom.pose.covariance[7] = 0.05 * 0.05   # y
        odom.pose.covariance[35] = 0.10 * 0.10  # yaw

        odom.twist.covariance[0] = 0.10 * 0.10  # vx
        odom.twist.covariance[35] = 0.20 * 0.20 # wz

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

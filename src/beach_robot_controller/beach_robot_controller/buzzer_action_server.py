import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Float32
from beach_robot_interfaces.action import Buzzer


class BuzzerActionServer(Node):
    def __init__(self):
        super().__init__('buzzer_action_server')

        # Publish duration to ESP32 bridge
        self.pub_buzzer = self.create_publisher(
            Float32,
            'buzzer_duration',
            10
        )

        self._action_server = ActionServer(
            self,
            Buzzer,
            'buzzer_beep',
            self.execute_callback
        )

        self.get_logger().info('BuzzerActionServer started (action: /buzzer_beep)')

    async def execute_callback(self, goal_handle):
        duration = max(0.0, float(goal_handle.request.duration))
        self.get_logger().info(f'Received buzzer goal: {duration:.2f} sec')

        # Turn buzzer on
        msg = Float32()
        msg.data = duration
        self.pub_buzzer.publish(msg)

        feedback_msg = Buzzer.Feedback()
        start_time = self.get_clock().now()

        update_period = 0.1  # sec

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Buzzer goal canceled')
                # Turn buzzer off
                off = Float32()
                off.data = 0.0
                self.pub_buzzer.publish(off)

                goal_handle.canceled()
                result = Buzzer.Result()
                result.success = False
                result.time_remaining = 0.0
                return result

            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds / 1e9
            remaining = max(0.0, duration - elapsed)

            feedback_msg.time_remaining = float(remaining)
            goal_handle.publish_feedback(feedback_msg)

            if elapsed >= duration:
                break

            await asyncio.sleep(update_period)

        # Time up → turn buzzer off
        off = Float32()
        off.data = 0.0
        self.pub_buzzer.publish(off)

        goal_handle.succeed()
        result = Buzzer.Result()
        result.success = True
        result.time_remaining = 0.0
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

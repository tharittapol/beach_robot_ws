import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class Teleop4WDSkid(Node):
    def __init__(self):
        super().__init__('teleop_4wd')

        # ---------------- Parameters ----------------
        # Movement limits
        self.declare_parameter('max_linear', 0.5)   # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s

        # Joystick axes
        self.declare_parameter('axis_linear', 1)    # e.g. 1 = left stick Y
        self.declare_parameter('axis_angular', 0)   # e.g. 0 = left stick X
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        # Deadman / enable button (LB on Logitech F710 is usually index 4)
        self.declare_parameter('enable_button', 4)

        # Button mapping (Logitech F710 in XInput mode ~ Xbox layout)
        # A=0, B=1, X=2, Y=3 by default
        self.declare_parameter('button_a', 0)   # hold 3s → toggle auto/manual
        self.declare_parameter('button_b', 1)   # press in auto mode → start cleaning
        self.declare_parameter('button_x', 2)   # toggle vibration
        self.declare_parameter('button_y', 3)   # stop / unlock stop

        # Hold durations (seconds)
        self.declare_parameter('mode_hold_sec', 3.0)    # A button hold
        self.declare_parameter('estop_hold_sec', 3.0)   # Y button hold for unlock

        # Service name for starting auto cleaning
        self.declare_parameter('auto_start_service', 'auto_clean/start')

        # Output cmd_vel topic
        self.declare_parameter('cmd_vel_out', 'cmd_vel')

        # ----- read parameters -----
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.axis_linear = int(self.get_parameter('axis_linear').value)
        self.axis_angular = int(self.get_parameter('axis_angular').value)
        self.scale_linear = float(self.get_parameter('scale_linear').value)
        self.scale_angular = float(self.get_parameter('scale_angular').value)

        self.enable_button = int(self.get_parameter('enable_button').value)

        self.button_a = int(self.get_parameter('button_a').value)
        self.button_x = int(self.get_parameter('button_x').value)
        self.button_y = int(self.get_parameter('button_y').value)
        self.mode_hold_sec = float(self.get_parameter('mode_hold_sec').value)
        self.estop_hold_sec = float(self.get_parameter('estop_hold_sec').value)

        self.auto_start_service_name = (
            self.get_parameter('auto_start_service')
            .get_parameter_value().string_value
        )

        self.cmd_vel_out_topic = (
            self.get_parameter('cmd_vel_out')
            .get_parameter_value().string_value
        )

        # ---------------- State ----------------
        # Control mode: manual / auto
        self.manual_mode = True  # start in manual
        # Vibration state (only used in manual)
        self.vibration_on = False
        # Emergency stop: if True, base must not move
        self.estop_active = False

        # Button state tracking (for edge + hold detection)
        self.prev_a = False
        self.prev_x = False
        self.prev_y = False

        self.a_press_start = None
        self.a_hold_toggled = False
        self.y_press_start = None

        # ---------------- Subscribers / Publishers ----------------
        self.sub_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        self.pub_vel_cmd = self.create_publisher(
            Twist,
            self.cmd_vel_out_topic,
            10
        )

        # Send vibration enable/disable to ESP32 (through esp32_bridge)
        self.pub_vibration = self.create_publisher(
            Bool,
            'vibration_enable',
            10
        )

        # Publish current mode: "manual" or "auto" (for other nodes)
        self.pub_mode = self.create_publisher(
            String,
            'control_mode',
            10
        )

        # Publish emergency stop state
        self.pub_estop = self.create_publisher(
            Bool,
            'e_stop',
            10
        )

        # ----- Service client for auto cleaning -----
        self.auto_start_client = self.create_client(
            Trigger,
            self.auto_start_service_name
        )

        self.get_logger().info(
            'Teleop4WDSkid started (LB=deadman, Left-stick=base, X=vibration, Y=stop, A=mode, B=start-auto-clean)'
        )

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def _now(self) -> float:
        """Current time in seconds (float)."""
        return self.get_clock().now().nanoseconds / 1e9
    
    # Service call helper
    def start_auto_clean(self):
        """Call auto cleaning start service when in auto mode."""
        if self.manual_mode:
            self.get_logger().warn(
                'B pressed but robot is in MANUAL mode; auto cleaning not started.'
            )
            return

        if self.estop_active:
            self.get_logger().warn(
                'Cannot start auto cleaning: E-STOP is active.'
            )
            return

        if not self.auto_start_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(
                f'Auto start service "{self.auto_start_service_name}" not available.'
            )
            return

        req = Trigger.Request()
        future = self.auto_start_client.call_async(req)
        future.add_done_callback(self._auto_start_response_cb)
        self.get_logger().info('Auto cleaning start request sent.')

    def _auto_start_response_cb(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'Auto cleaning service call failed: {e}')
            return

        if resp.success:
            self.get_logger().info(f'Auto cleaning started: {resp.message}')
        else:
            self.get_logger().warn(f'Auto cleaning rejected: {resp.message}')

    # ------------------------------------------------------------------
    # Joystick callback
    # ------------------------------------------------------------------
    def joy_callback(self, msg: Joy):
        now = self._now()
        axes = msg.axes
        buttons = msg.buttons

        def get_axis(idx: int, default: float = 0.0) -> float:
            if 0 <= idx < len(axes):
                return axes[idx]
            return default

        def get_button(idx: int) -> bool:
            if 0 <= idx < len(buttons):
                return bool(buttons[idx])
            return False

        # -------- Read axes (for base movement) --------
        raw_lin = get_axis(self.axis_linear)
        raw_ang = get_axis(self.axis_angular)

        # -------- Read buttons --------
        enable_pressed = get_button(self.enable_button)
        a_pressed = get_button(self.button_a)
        b_pressed = get_button(self.button_b)
        x_pressed = get_button(self.button_x)
        y_pressed = get_button(self.button_y)

        # ==============================================================
        # Handle A button: hold 3s → toggle manual/auto mode
        #    (independent of deadman; works anytime)
        # ==============================================================
        if a_pressed and not self.prev_a:
            # just pressed
            self.a_press_start = now
            self.a_hold_toggled = False
        elif a_pressed and self.prev_a:
            # still holding
            if (self.a_press_start is not None
                    and not self.a_hold_toggled
                    and (now - self.a_press_start) >= self.mode_hold_sec):
                # toggle mode
                self.manual_mode = not self.manual_mode
                self.a_hold_toggled = True

                mode_str = 'manual' if self.manual_mode else 'auto'
                self.get_logger().info(f'Control mode switched to: {mode_str}')

                mode_msg = String()
                mode_msg.data = mode_str
                self.pub_mode.publish(mode_msg)
        else:
            # released
            self.a_press_start = None
            self.a_hold_toggled = False

        # ==============================================================
        # Handle Y button: emergency stop
        #    - short press → activate stop immediately
        #    - hold 3s (while stopped) → release
        #    (works in both manual & auto, independent of deadman)
        # ==============================================================
        if y_pressed and not self.prev_y:
            # rising edge
            if not self.estop_active:
                self.estop_active = True
                self.get_logger().warn('E-STOP activated by Y button')
            # start potential unlock timer
            self.y_press_start = now

        elif y_pressed and self.prev_y:
            # still holding
            if (self.estop_active
                    and self.y_press_start is not None
                    and (now - self.y_press_start) >= self.estop_hold_sec):
                self.estop_active = False
                self.get_logger().info('E-STOP released after holding Y')
                self.y_press_start = None
        else:
            self.y_press_start = None

        # ==============================================================
        # Handle X button: toggle vibration motor (manual mode only)
        #    (independent of deadman)
        # ==============================================================
        if self.manual_mode:
            if x_pressed and not self.prev_x:
                self.vibration_on = not self.vibration_on
                vmsg = Bool()
                vmsg.data = self.vibration_on
                self.pub_vibration.publish(vmsg)
                self.get_logger().info(
                    f'Vibration motor {"ENABLED" if self.vibration_on else "DISABLED"}'
                )

        # ==============================================================
        # Handle B button: start auto cleaning (AUTO mode only)
        #     - only reacts on rising edge
        # ==============================================================
        if b_pressed and not self.prev_b:
            # only attempt when in auto mode; function itself checks estop + service
            self.start_auto_clean()
        
        # ==============================================================
        # Compute wheel_cmd (deadman only affects movement)
        #    - Need: manual_mode == True
        #    - AND   estop_active == False
        #    - AND   deadman (LB) pressed
        # ==============================================================
        if self.manual_mode and not self.estop_active and enable_pressed:
            # scaling
            v = raw_lin * self.scale_linear * self.max_linear # m/s
            w = raw_ang * self.scale_angular * self.max_angular # rad/s

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = float(w)

            self.pub_vel_cmd.publish(cmd)

        elif self.estop_active:
            # e-stop active → publish zero speeds
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_vel_cmd.publish(cmd)

        # ==============================================================
        # Publish mode & e-stop for other nodes
        # ==============================================================
        mode_msg = String()
        mode_msg.data = 'manual' if self.manual_mode else 'auto'
        self.pub_mode.publish(mode_msg)

        estop_msg = Bool()
        estop_msg.data = self.estop_active
        self.pub_estop.publish(estop_msg)

        # update previous button states
        self.prev_a = a_pressed
        self.prev_x = x_pressed
        self.prev_y = y_pressed


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

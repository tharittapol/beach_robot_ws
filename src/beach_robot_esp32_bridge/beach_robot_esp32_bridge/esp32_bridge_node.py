import json
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu

import serial


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('imu_frame_id', 'imu_link')

        # Read parameters and store in self.*
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

        self.ser = None
        self.serial_lock = threading.Lock()

        # Publishers
        self.pub_enc_vel = self.create_publisher(
            Float32MultiArray,
            'enc_vel',
            10
        )
        self.pub_imu = self.create_publisher(
            Imu,
            'imu/data',
            10
        )

        # Subscribers
        self.sub_wheel_cmd = self.create_subscription(
            Float32MultiArray,
            'wheel_cmd',
            self.wheel_cmd_callback,
            10
        )
        self.sub_buzzer = self.create_subscription(
            Float32,
            'buzzer_duration',
            self.buzzer_callback,
            10
        )

        # Try to open serial (with retry)
        self.open_serial_with_retry()

        # Start read thread
        self.read_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.read_thread.start()

        self.get_logger().info('ESP32Bridge node started.')

    # ------------------------------------------------------------------
    # Serial helpers
    # ------------------------------------------------------------------
    def open_serial_with_retry(self):
        """Try to open the serial port, retrying until success or node shutdown."""
        while rclpy.ok():
            try:
                self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout
                )
                self.get_logger().info(
                    f'Opened serial port {self.port} at {self.baudrate} baud'
                )
                return
            except serial.SerialException as e:
                self.get_logger().error(
                    f'Failed to open serial port {self.port}: {e}. Retrying in 2s...'
                )
                time.sleep(2.0)

    def close_serial(self):
        with self.serial_lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    # ------------------------------------------------------------------
    # Callbacks to send commands to ESP32
    # ------------------------------------------------------------------
    def wheel_cmd_callback(self, msg: Float32MultiArray):
        """Send wheel_cmd to ESP32 as JSON: {"wheel_cmd":[v_fl,v_fr,v_rl,v_rr]}"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial not open, cannot send wheel command')
            return

        data = {'wheel_cmd': list(msg.data)}
        line = json.dumps(data) + '\n'
        encoded = line.encode('utf-8')

        with self.serial_lock:
            try:
                self.ser.write(encoded)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (wheel_cmd): {e}')
                self.close_serial()
                self.open_serial_with_retry()

    def buzzer_callback(self, msg: Float32):
        """Send buzzer_duration to ESP32 as JSON: {"buzzer_duration": sec}"""
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial not open, cannot send buzzer command')
            return

        dur = float(msg.data)
        data = {'buzzer_duration': dur}
        line = json.dumps(data) + '\n'
        encoded = line.encode('utf-8')

        with self.serial_lock:
            try:
                self.ser.write(encoded)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error (buzzer): {e}')
                self.close_serial()
                self.open_serial_with_retry()

    # ------------------------------------------------------------------
    # Read loop from ESP32
    # ------------------------------------------------------------------
    def read_loop(self):
        """
        Run in separate thread, read JSON line by line from ESP32.

        Expected JSON from ESP32:
        {
          "enc_vel":[v_fl,v_fr,v_rl,v_rr],
          "imu_quat":[x,y,z,w],
          "imu_gyro":[gx,gy,gz],
          "imu_lin_acc":[ax,ay,az]
        }
        plus sometimes {"info":"..."} lines.
        """
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.open_serial_with_retry()

            try:
                line = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}. Reconnecting...')
                self.close_serial()
                time.sleep(1.0)
                continue

            if not line:
                continue

            try:
                text = line.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue

                # Skip non-JSON garbage (boot logs, etc.)
                if not text.startswith('{'):
                    continue

                data = json.loads(text)
            except Exception as e:
                self.get_logger().warn(f'Invalid JSON from ESP32: {e}')
                continue

            # Info messages
            if 'info' in data:
                self.get_logger().info(f"ESP32 info: {data['info']}")

            # Encoders
            if 'enc_vel' in data:
                self.publish_enc_vel(data['enc_vel'])

            # IMU (only if fields present)
            if any(k in data for k in ('imu_quat', 'imu_gyro', 'imu_lin_acc')):
                self.publish_imu(data)

    def publish_enc_vel(self, enc_list):
        if not isinstance(enc_list, list):
            self.get_logger().warn('enc_vel is not a list from ESP32')
            return

        msg = Float32MultiArray()
        try:
            msg.data = [float(x) for x in enc_list]
        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid enc_vel format from ESP32: {e}')
            return

        self.pub_enc_vel.publish(msg)

    def publish_imu(self, data: dict):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        try:
            # Orientation
            if 'imu_quat' in data:
                q = data['imu_quat']
                if isinstance(q, list) and len(q) == 4:
                    imu_msg.orientation.x = float(q[0])
                    imu_msg.orientation.y = float(q[1])
                    imu_msg.orientation.z = float(q[2])
                    imu_msg.orientation.w = float(q[3])

            # Angular velocity
            if 'imu_gyro' in data:
                g = data['imu_gyro']
                if isinstance(g, list) and len(g) == 3:
                    imu_msg.angular_velocity.x = float(g[0])
                    imu_msg.angular_velocity.y = float(g[1])
                    imu_msg.angular_velocity.z = float(g[2])

            # Linear acceleration
            if 'imu_lin_acc' in data:
                a = data['imu_lin_acc']
                if isinstance(a, list) and len(a) == 3:
                    imu_msg.linear_acceleration.x = float(a[0])
                    imu_msg.linear_acceleration.y = float(a[1])
                    imu_msg.linear_acceleration.z = float(a[2])

        except (TypeError, ValueError) as e:
            self.get_logger().warn(f'Invalid IMU data from ESP32: {e}')
            return

        self.pub_imu.publish(imu_msg)



def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

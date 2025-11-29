import json
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
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

        # Publisher for encoder velocity from ESP32 (m/s)
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

        # Subscriber for wheel commands [FL, FR, RL, RR]
        self.sub_wheel_cmd = self.create_subscription(
            Float32MultiArray,
            'wheel_cmd',
            self.wheel_cmd_callback,
            10
        )

        # Lock for serial write
        self.serial_lock = threading.Lock()

        # Try to open serial (with retry)
        self.open_serial_with_retry()

        # Start read thread
        self.read_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.read_thread.start()

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

    def wheel_cmd_callback(self, msg: Float32MultiArray):
        # No serial right now → drop command
        if self.ser is None or not self.ser.is_open:
            return

        # Expect 4 elements: [FL, FR, RL, RR]
        data = {
            'wheel_cmd': list(msg.data)
        }
        line = json.dumps(data) + '\n'
        encoded = line.encode('utf-8')

        with self.serial_lock:
            try:
                self.ser.write(encoded)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write error: {e}')
                # try to recover
                self.close_serial()
                self.open_serial_with_retry()

    def read_loop(self):
        """Run in separate thread, read JSON line by line."""
        while rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                # Wait until can open the port again
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
                # decode safely
                text = line.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue

                # skip boot logs / garbage that isn't JSON
                if not text.startswith('{'):
                    continue

                data = json.loads(text)
            except Exception as e:
                self.get_logger().warn(f'Invalid JSON from ESP32: {e}')
                continue

            # Debug: log info messages from ESP32
            if 'info' in data:
                self.get_logger().info(f"ESP32 info: {data['info']}")

            # Parse encoder velocity field from ESP32
            if 'enc_vel' in data:
                vel = data['enc_vel']
                if isinstance(vel, list):
                    msg = Float32MultiArray()
                    msg.data = [float(x) for x in vel]
                    self.pub_enc_vel.publish(msg)
            
            # Build and publish IMU message if fields are present
            has_imu = False
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame_id if hasattr(self, 'imu_frame_id') else 'imu_link'

            # Orientation (quaternion)
            if 'imu_quat' in data:
                q = data['imu_quat']
                if isinstance(q, list) and len(q) == 4:
                    imu_msg.orientation.x = float(q[0])
                    imu_msg.orientation.y = float(q[1])
                    imu_msg.orientation.z = float(q[2])
                    imu_msg.orientation.w = float(q[3])
                    has_imu = True

            # Angular velocity (gyro)
            if 'imu_gyro' in data:
                g = data['imu_gyro']
                if isinstance(g, list) and len(g) == 3:
                    imu_msg.angular_velocity.x = float(g[0])
                    imu_msg.angular_velocity.y = float(g[1])
                    imu_msg.angular_velocity.z = float(g[2])
                    has_imu = True

            # Linear acceleration
            if 'imu_lin_acc' in data:
                a = data['imu_lin_acc']
                if isinstance(a, list) and len(a) == 3:
                    imu_msg.linear_acceleration.x = float(a[0])
                    imu_msg.linear_acceleration.y = float(a[1])
                    imu_msg.linear_acceleration.z = float(a[2])
                    has_imu = True

            if has_imu:
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

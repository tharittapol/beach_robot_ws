import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32

import serial


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout
            )
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise e

        self.get_logger().info(f'Opened serial port {port} at {baudrate} baud')

        # Publishers for sensors
        self.pub_encoders = self.create_publisher(
            Int32MultiArray,
            'encoders',
            10
        )
        self.pub_battery = self.create_publisher(
            Float32,
            'battery_voltage',
            10
        )

        # Subscriber for wheel commands
        self.sub_wheel_cmd = self.create_subscription(
            Float32MultiArray,
            'wheel_cmd',
            self.wheel_cmd_callback,
            10
        )

        # Lock for serial write
        self.serial_lock = threading.Lock()

        # Start read thread
        self.read_thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.read_thread.start()

    def wheel_cmd_callback(self, msg: Float32MultiArray):
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

    def read_loop(self):
        # Run in separate thread, read JSON line by line
        while rclpy.ok():
            try:
                line = self.ser.readline()
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break

            if not line:
                continue

            try:
                text = line.decode('utf-8').strip()
                if not text:
                    continue
                data = json.loads(text)
            except Exception as e:
                self.get_logger().warn(f'Invalid JSON from ESP32: {e}')
                continue

            # Parse encoders
            if 'encoders' in data:
                enc = data['encoders']
                if isinstance(enc, list):
                    msg = Int32MultiArray()
                    msg.data = [int(x) for x in enc]
                    self.pub_encoders.publish(msg)

            # Parse battery voltage
            if 'battery' in data:
                try:
                    v = float(data['battery'])
                    m = Float32()
                    m.data = v
                    self.pub_battery.publish(m)
                except (TypeError, ValueError):
                    pass


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

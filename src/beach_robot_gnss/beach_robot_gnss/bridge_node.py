#!/usr/bin/env python3
import base64
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8, String
import serial
import pynmea2


class Um982NtripBridge(Node):
    def __init__(self):
        super().__init__('um982_ntrip_bridge')

        # --- Serial params ---
        self.port = self.declare_parameter('port', '/dev/ttyACM2').value
        self.baud = int(self.declare_parameter('baud', 115200).value)
        self.frame_id = self.declare_parameter('frame_id', 'gps_link').value

        # --- NTRIP params ---
        self.ntrip_host = self.declare_parameter('ntrip_host', 'rtk2go.com').value
        self.ntrip_port = int(self.declare_parameter('ntrip_port', 2101).value)
        self.mountpoint = self.declare_parameter('mountpoint', 'TH-Kukot').value
        self.username = self.declare_parameter('username', 'tharittapol.big-at-gmail-d-com').value
        self.password = self.declare_parameter('password', 'none').value

        # how often to send GGA upstream to caster (seconds)
        self.gga_send_period = float(self.declare_parameter('gga_send_period', 1.0).value)

        # --- ROS pub ---
        self.pub_fix = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.pub_fixq = self.create_publisher(UInt8, 'gps/fix_quality', 10)
        self.pub_rtk = self.create_publisher(String, 'gps/rtk_state', 10)

        # keep latest GST sigmas (meters)
        self._gst_sigma_lat = None
        self._gst_sigma_lon = None
        self._gst_sigma_alt = None
        self._last_gst_time = 0.0

        # shared state
        self._latest_gga_sentence = None
        self._latest_fix_quality = 0
        self._stop = False

        # open serial once
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')

        # start threads
        self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.ntrip_thread = threading.Thread(target=self._ntrip_loop, daemon=True)
        self.serial_thread.start()
        self.ntrip_thread.start()

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def _serial_loop(self):
        """Read NMEA lines from UM982, publish NavSatFix, store latest GGA for NTRIP."""
        buf = b''
        while rclpy.ok() and not self._stop:
            try:
                data = self.ser.read(4096)
                if not data:
                    continue
                buf += data

                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue

                    # NMEA lines start with $
                    if not line.startswith(b'$'):
                        continue

                    try:
                        s = line.decode('ascii', errors='ignore')
                    except Exception:
                        continue

                    # Parse only sentences we care about
                    if 'GGA' in s:
                        self._latest_gga_sentence = s
                        self._handle_gga(s)
                    elif 'GST' in s:
                        self._handle_gst(s)
            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')
                time.sleep(0.5)

    def _handle_gga(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return

        # GGA fix quality: 0 invalid, 1 GNSS, 2 DGPS, 4 RTK FIX, 5 RTK FLOAT
        try:
            fix_q = int(msg.gps_qual)
        except Exception:
            fix_q = 0
        self._latest_fix_quality = fix_q

        # publish fix_quality
        q = UInt8()
        q.data = max(0, min(255, fix_q))
        self.pub_fixq.publish(q)

        # publish rtk_state
        state = String()
        if fix_q == 4:
            state.data = "RTK_FIX"
        elif fix_q == 5:
            state.data = "RTK_FLOAT"
        elif fix_q == 2:
            state.data = "DGPS"
        elif fix_q == 1:
            state.data = "GNSS"
        elif fix_q == 0:
            state.data = "NO_FIX"
        else:
            state.data = f"UNKNOWN_{fix_q}"
        self.pub_rtk.publish(state)

        # optional: log once per second
        # self.get_logger().info(f"GGA fixQ={fix_q} sats={msg.num_sats} hdop={msg.horizontal_dil} state={state.data}")


        # Build NavSatFix
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id

        fix.latitude = float(msg.latitude) if msg.latitude else 0.0
        fix.longitude = float(msg.longitude) if msg.longitude else 0.0
        fix.altitude = float(msg.altitude) if msg.altitude else 0.0

        fix.status.service = NavSatStatus.SERVICE_GPS
        if fix_q == 0:
            fix.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            fix.status.status = NavSatStatus.STATUS_FIX

        # Covariance (dynamic):
        use_gst = (
            self._gst_sigma_lat is not None and
            self._gst_sigma_lon is not None and
            (time.time() - self._last_gst_time) < 2.5
        )

        if use_gst:
            sigma_lat = max(0.01, self._gst_sigma_lat)  # clamp min 1 cm prevent 0
            sigma_lon = max(0.01, self._gst_sigma_lon)
            sigma_alt = max(0.02, self._gst_sigma_alt) if self._gst_sigma_alt is not None else 1.0

            var_x = sigma_lon * sigma_lon
            var_y = sigma_lat * sigma_lat
            var_z = sigma_alt * sigma_alt
        else:
            # fallback mapping by fix type
            if fix_q == 4:
                sigma_xy = 0.02
            elif fix_q == 5:
                sigma_xy = 0.10
            elif fix_q == 2:
                sigma_xy = 0.50
            elif fix_q == 1:
                sigma_xy = 1.50
            else:
                sigma_xy = 10.0
            var_x = var_y = sigma_xy * sigma_xy
            var_z = (sigma_xy * 2.0) ** 2

        fix.position_covariance = [
            var_x, 0.0, 0.0,
            0.0, var_y, 0.0,
            0.0, 0.0, var_z
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_fix.publish(fix)

    def _handle_gst(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return

        # GST fields in pynmea2:
        # msg.std_dev_latitude, msg.std_dev_longitude, msg.std_dev_altitude (meters)
        try:
            self._gst_sigma_lat = float(msg.std_dev_latitude) if msg.std_dev_latitude else None
            self._gst_sigma_lon = float(msg.std_dev_longitude) if msg.std_dev_longitude else None
            self._gst_sigma_alt = float(msg.std_dev_altitude) if msg.std_dev_altitude else None
            self._last_gst_time = time.time()
        except Exception:
            return


    def _ntrip_loop(self):
        """Connect to NTRIP, read RTCM, write to UM982 serial; optionally send GGA upstream."""
        if not self.username:
            self.get_logger().error('NTRIP username is empty. Set username parameter.')
            return

        while rclpy.ok() and not self._stop:
            sock = None
            try:
                sock = socket.create_connection((self.ntrip_host, self.ntrip_port), timeout=10.0)
                auth = base64.b64encode(f'{self.username}:{self.password}'.encode()).decode()

                # NTRIP request
                req = (
                    f'GET /{self.mountpoint} HTTP/1.1\r\n'
                    f'Host: {self.ntrip_host}\r\n'
                    f'User-Agent: NTRIP um982_ntrip_bridge\r\n'
                    f'Authorization: Basic {auth}\r\n'
                    f'Connection: close\r\n\r\n'
                )
                sock.sendall(req.encode('ascii'))

                # Read header
                header = sock.recv(4096)
                if b'200' not in header and b'ICY 200 OK' not in header:
                    self.get_logger().error(f'NTRIP rejected: {header[:200]!r}')
                    time.sleep(2.0)
                    continue

                self.get_logger().info('NTRIP connected, streaming RTCM.')

                sock.settimeout(1.0)

                last_gga_sent = 0.0
                while rclpy.ok() and not self._stop:
                    # Send GGA upstream periodically (some casters need it)
                    now = time.time()
                    if self._latest_gga_sentence and (now - last_gga_sent) >= self.gga_send_period:
                        gga = (self._latest_gga_sentence + '\r\n').encode('ascii', errors='ignore')
                        try:
                            sock.sendall(gga)
                        except Exception:
                            pass
                        last_gga_sent = now

                    # Receive RTCM and write to serial
                    try:
                        rtcm = sock.recv(4096)
                        if not rtcm:
                            raise RuntimeError('NTRIP stream closed')
                        self.ser.write(rtcm)
                    except socket.timeout:
                        continue

            except Exception as e:
                self.get_logger().warn(f'NTRIP error: {e}. Reconnecting...')
                time.sleep(2.0)
            finally:
                if sock:
                    try:
                        sock.close()
                    except Exception:
                        pass


def main():
    rclpy.init()
    node = Um982NtripBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

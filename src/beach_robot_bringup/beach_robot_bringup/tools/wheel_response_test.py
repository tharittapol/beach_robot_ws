import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


WHEEL_NAMES = ("fl", "fr", "rl", "rr")


class WheelResponseTest(Node):
    def __init__(self, args):
        super().__init__("wheel_response_test")
        self.args = args
        self.started_wall = time.time()
        self.started_mono = time.monotonic()

        self.latest_wheel_cmd = None
        self.latest_wheel_cmd_stamp = None
        self.latest_enc_vel = None
        self.latest_enc_vel_stamp = None
        self.latest_debug = {}
        self.latest_debug_stamp = None

        self.pub_cmd = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.pub_json = self.create_publisher(String, args.esp32_json_cmd_topic, 10)
        self.create_subscription(
            Float32MultiArray,
            args.wheel_cmd_topic,
            self._wheel_cmd_cb,
            50,
        )
        self.create_subscription(
            Float32MultiArray,
            args.enc_vel_topic,
            self._enc_vel_cb,
            50,
        )
        self.create_subscription(
            String,
            args.esp32_debug_topic,
            self._debug_cb,
            50,
        )

        self.out_dir = Path(args.out_dir).expanduser()
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in args.label)
        self.csv_path = self.out_dir / f"wheel_response_{stamp}_{safe_label}.csv"
        self.meta_path = self.out_dir / f"wheel_response_{stamp}_{safe_label}.json"

        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self._csv_fields())
        self.writer.writeheader()

    def _wheel_cmd_cb(self, msg):
        self.latest_wheel_cmd = [float(x) for x in msg.data[:4]]
        self.latest_wheel_cmd_stamp = self.get_clock().now()

    def _enc_vel_cb(self, msg):
        self.latest_enc_vel = [float(x) for x in msg.data[:4]]
        self.latest_enc_vel_stamp = self.get_clock().now()

    def _debug_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Invalid JSON on ESP32 debug topic")
            return
        if not isinstance(data, dict):
            return
        self.latest_debug = data
        self.latest_debug_stamp = self.get_clock().now()

    def _csv_fields(self):
        fields = [
            "wall_time",
            "ros_time_sec",
            "elapsed_sec",
            "test",
            "phase",
            "cmd_v",
            "cmd_w",
            "wheel_cmd_age_sec",
            "enc_vel_age_sec",
            "debug_age_sec",
        ]
        for prefix in (
            "ros_wheel_cmd",
            "ros_enc_vel",
            "dbg_wheel_cmd",
            "dbg_enc_vel_raw",
            "dbg_enc_vel_corr",
            "dbg_u_ff",
            "dbg_u_pid",
            "dbg_motor_u",
            "dbg_pid_kp",
            "dbg_pid_ki",
            "dbg_vmax_mps",
            "dbg_use_closed_loop",
        ):
            fields.extend(f"{prefix}_{wheel}" for wheel in WHEEL_NAMES)
        fields.extend(("dbg_cmd_age_ms", "dbg_enc_age_ms"))
        return fields

    def _age_sec(self, stamp, now):
        if stamp is None:
            return ""
        return f"{(now - stamp).nanoseconds / 1e9:.3f}"

    def _array_value(self, arr, idx):
        if not isinstance(arr, list) or idx >= len(arr):
            return ""
        try:
            return f"{float(arr[idx]):.6f}"
        except (TypeError, ValueError):
            return ""

    def _add_array(self, row, prefix, arr):
        for idx, wheel in enumerate(WHEEL_NAMES):
            row[f"{prefix}_{wheel}"] = self._array_value(arr, idx)

    def _sample(self, test_name, phase, cmd_v, cmd_w):
        now = self.get_clock().now()
        debug = self.latest_debug or {}

        row = {
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "ros_time_sec": f"{now.nanoseconds / 1e9:.6f}",
            "elapsed_sec": f"{time.monotonic() - self.started_mono:.3f}",
            "test": test_name,
            "phase": phase,
            "cmd_v": f"{cmd_v:.6f}",
            "cmd_w": f"{cmd_w:.6f}",
            "wheel_cmd_age_sec": self._age_sec(self.latest_wheel_cmd_stamp, now),
            "enc_vel_age_sec": self._age_sec(self.latest_enc_vel_stamp, now),
            "debug_age_sec": self._age_sec(self.latest_debug_stamp, now),
            "dbg_cmd_age_ms": debug.get("cmd_age_ms", ""),
            "dbg_enc_age_ms": debug.get("enc_age_ms", ""),
        }

        self._add_array(row, "ros_wheel_cmd", self.latest_wheel_cmd)
        self._add_array(row, "ros_enc_vel", self.latest_enc_vel)
        self._add_array(row, "dbg_wheel_cmd", debug.get("wheel_cmd"))
        self._add_array(row, "dbg_enc_vel_raw", debug.get("enc_vel_raw"))
        self._add_array(row, "dbg_enc_vel_corr", debug.get("enc_vel_corr"))
        self._add_array(row, "dbg_u_ff", debug.get("u_ff"))
        self._add_array(row, "dbg_u_pid", debug.get("u_pid"))
        self._add_array(row, "dbg_motor_u", debug.get("motor_u"))
        self._add_array(row, "dbg_pid_kp", debug.get("pid_kp"))
        self._add_array(row, "dbg_pid_ki", debug.get("pid_ki"))
        self._add_array(row, "dbg_vmax_mps", debug.get("vmax_mps"))
        self._add_array(row, "dbg_use_closed_loop", debug.get("use_closed_loop"))

        self.writer.writerow(row)
        self.csv_file.flush()

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _send_esp32_json(self, data, repeat=3):
        msg = String()
        msg.data = json.dumps(data, separators=(",", ":"))
        for _ in range(max(1, repeat)):
            self.pub_json.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _run_phase(self, test_name, phase, v, w, duration_sec):
        end_time = time.monotonic() + max(0.0, duration_sec)
        next_pub = 0.0
        next_log = 0.0
        pub_period = 1.0 / max(1.0, self.args.publish_rate_hz)
        log_period = 1.0 / max(1.0, self.args.log_rate_hz)

        while rclpy.ok() and time.monotonic() < end_time:
            now_mono = time.monotonic()
            if now_mono >= next_pub:
                self._publish_cmd(v, w)
                next_pub = now_mono + pub_period
            if now_mono >= next_log:
                self._sample(test_name, phase, v, w)
                next_log = now_mono + log_period
            rclpy.spin_once(self, timeout_sec=0.002)

    def _publish_zero_burst(self, duration_sec=0.5):
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def run(self, tests):
        metadata = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "csv_path": str(self.csv_path),
            "topics": {
                "cmd_vel": self.args.cmd_vel_topic,
                "wheel_cmd": self.args.wheel_cmd_topic,
                "enc_vel": self.args.enc_vel_topic,
                "esp32_debug": self.args.esp32_debug_topic,
                "esp32_json_cmd": self.args.esp32_json_cmd_topic,
            },
            "publish_rate_hz": self.args.publish_rate_hz,
            "log_rate_hz": self.args.log_rate_hz,
            "settle_sec": self.args.settle_sec,
            "step_sec": self.args.step_sec,
            "stop_sec": self.args.stop_sec,
            "tests": tests,
        }
        self.meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

        if not self.args.no_esp32_debug:
            self._send_esp32_json({"dbg_enable": True, "dbg_rate_ms": self.args.debug_rate_ms}, repeat=5)

        try:
            self.get_logger().info(f"Writing wheel response CSV: {self.csv_path}")
            self._run_phase("startup", "zero", 0.0, 0.0, self.args.settle_sec)
            for test_name, v, w in tests:
                self.get_logger().info(f"Running {test_name}: v={v:.3f} m/s w={w:.3f} rad/s")
                self._run_phase(test_name, "settle_zero", 0.0, 0.0, self.args.settle_sec)
                self._run_phase(test_name, "command", v, w, self.args.step_sec)
                self._run_phase(test_name, "stop_zero", 0.0, 0.0, self.args.stop_sec)
        finally:
            self._publish_zero_burst()
            if not self.args.no_esp32_debug:
                self._send_esp32_json({"dbg_enable": False}, repeat=3)
            self.csv_file.close()
            self.get_logger().info(f"Saved CSV: {self.csv_path}")
            self.get_logger().info(f"Saved metadata: {self.meta_path}")


def build_tests(args):
    all_tests = {
        "forward": (args.forward_speed, 0.0),
        "backward": (-args.forward_speed, 0.0),
        "spin_left": (0.0, args.spin_rate),
        "spin_right": (0.0, -args.spin_rate),
        "curve_left": (args.curve_speed, args.curve_rate),
        "curve_right": (args.curve_speed, -args.curve_rate),
    }

    selected = []
    requested = [name.strip() for name in args.tests.split(",") if name.strip()]
    for name in requested:
        if name not in all_tests:
            raise ValueError(f"Unknown test '{name}'. Available: {', '.join(all_tests)}")
        v, w = all_tests[name]
        selected.append((name, v, w))
    return selected


def parse_args():
    parser = argparse.ArgumentParser(description="Run wheel response tests and log ROS/ESP32 signals to CSV.")
    parser.add_argument("--label", default="lifted", help="Label appended to output filenames.")
    parser.add_argument("--out-dir", default="~/beach_robot_logs/wheel_response", help="Directory for CSV logs.")
    parser.add_argument("--tests", default="forward,backward,spin_left,spin_right,curve_left,curve_right")
    parser.add_argument("--forward-speed", type=float, default=0.25, help="m/s for forward/backward tests.")
    parser.add_argument("--spin-rate", type=float, default=0.35, help="rad/s for spin-in-place tests.")
    parser.add_argument("--curve-speed", type=float, default=0.20, help="m/s for curve tests.")
    parser.add_argument("--curve-rate", type=float, default=0.25, help="rad/s for curve tests.")
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--step-sec", type=float, default=4.0)
    parser.add_argument("--stop-sec", type=float, default=1.5)
    parser.add_argument("--publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--log-rate-hz", type=float, default=50.0)
    parser.add_argument("--debug-rate-ms", type=int, default=100)
    parser.add_argument("--no-esp32-debug", action="store_true")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--wheel-cmd-topic", default="/wheel_cmd")
    parser.add_argument("--enc-vel-topic", default="/enc_vel")
    parser.add_argument("--esp32-debug-topic", default="/esp32/debug")
    parser.add_argument("--esp32-json-cmd-topic", default="/esp32/json_cmd")
    return parser.parse_args()


def main():
    args = parse_args()
    tests = build_tests(args)
    rclpy.init()
    node = WheelResponseTest(args)
    try:
        node.run(tests)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# Beach Robot Workspace — CLAUDE.md

## Project Overview

Autonomous beach-cleaning robot running **ROS2** on a **Jetson Nano Super**.
The robot has 4 wheels (front: track/caterpillar tyres, narrow; rear: round tyres, wider), a scoop under the chassis, and a vibration-motor sieve to separate trash from sand.

**Development workflow**: edit code on laptop → `git push` → SSH into Jetson → `git pull` → `colcon build` → test.

---

## Repository Layout

```
beach_robot_ws/
├── src/                        # ROS2 packages (build with colcon)
│   ├── beach_robot_bringup/        # Hardware orchestrator + analysis tools
│   ├── beach_robot_coverage_nav2/  # Coverage planner + Nav2 bringup
│   ├── beach_robot_localization/   # Wheel odom + EKF + GNSS fusion
│   ├── beach_wheel_mixer/          # cmd_vel → wheel_cmd (C++)
│   ├── beach_robot_esp32_bridge/   # Serial bridge to ESP32
│   ├── beach_robot_teleop/         # Joystick teleop
│   ├── beach_robot_gnss/           # UM982 GNSS driver + RTK/NTRIP
│   ├── beach_robot_nav2/           # Standalone Nav2 config package
│   ├── beach_robot_controller/     # Buzzer action server
│   ├── beach_robot_sim/            # Gazebo simulation launcher
│   ├── beach_robot_description/    # URDF / robot description
│   ├── beach_robot_interfaces/     # Custom messages / actions
│   ├── zed_nav2_cloud_filter/      # ZED depth → obstacle layer (C++)
│   └── zed-ros2-wrapper/           # External submodule (Stereolabs)
└── firmware/                   # Version-tracked ESP32 source (reference only)
    └── src/main_esp.cpp            # Master ESP32 firmware (~1160 lines)
```

> `firmware/` is **not built here**. Actual flashing uses PlatformIO in a separate repo.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Compute | Jetson Nano Super |
| MCU | 2× ESP32 (motor/sensor control via serial) |
| GNSS | UM982 (RTK-capable) |
| IMU | BNO055 (via ESP32) |
| Depth cam + IMU | ZED Mini |
| Ultrasonic | 3× HC-SR04 (left/middle/right) |
| Joystick | 2.4 GHz receiver on `/dev/input/js_joy` |
| ESP32 serial | `/dev/ttyESP32` @ 230400 baud |

### Wheel geometry (important — asymmetric robot)
| Wheel | Type | Track width | Encoder scale |
|-------|------|------------|---------------|
| FL / FR (front) | Caterpillar | 0.734 m | 1.0 / 1.0 |
| RL / RR (rear) | Round | 1.179 m | 0.64 / 0.64 |

The rear encoder scale of **0.64** compensates for chain drive reduction.

---

## Tuned Parameters (do not change without re-testing)

| Parameter | Value | Tuned in |
|-----------|-------|---------|
| `linear_scale` | **1.45** | Wheel odom field test (10 m straight) |
| `angular_scale` | 1.0 | Baseline (spin tests pending) |
| ESP32 PID Kp | [0.15, 0.18, 0.04, 0.03] | On-air test 2026-03-21 |
| ESP32 PID Ki | [0.01, 0.01, 0.00, 0.00] | Same session |
| Wheel V_MAX | [1.25, 1.10, 9.70, 8.60] m/s | Hardware characterisation |

---

## Key Topics

| Topic | Type | Producer |
|-------|------|---------|
| `/cmd_vel` | Twist | Nav2 controller / teleop |
| `/wheel_cmd` | Float32MultiArray | beach_wheel_mixer |
| `/enc_vel` | Float32MultiArray | esp32_bridge |
| `/wheel/odom` | Odometry | wheel_odometry_node |
| `/odometry/fusion_bno` | Odometry | EKF (wheel + BNO055) |
| `/odometry/local` | Odometry | EKF output used by Nav2 |
| `/odometry/gps` | Odometry | navsat_transform (only if use_gnss=true) |
| `/imu/data` | Imu | BNO055 via ESP32 |
| `/zed/filtered_cloud` | PointCloud2 | zed_nav2_cloud_filter |
| `/ultrasonic/{left,middle,right}` | Range | ESP32 bridge |
| `/gps/fix` | NavSatFix | UM982 bridge |
| `/coverage/path` | Path | coverage_follow_waypoints (preview) |

---

## Package Details

### `beach_robot_coverage_nav2`
Coverage planner + full Nav2 bringup.

**Main files:**
- `launch/beach_cleaning_bringup.launch.py` — master launch
- `beach_robot_coverage_nav2/coverage_follow_waypoints.py` — generates waypoints, sends to Nav2 `follow_waypoints` action
- `config/nav2_params_keepout.yaml` — Nav2 tuning
- `config/keepout_mask.yaml` + `keepout_mask.pgm` — boundary / keepout map

**Coverage node key parameters:**

| Parameter | Default | Notes |
|-----------|---------|-------|
| `pattern` | `boustrophedon` | also: `spiral` |
| `area.width / height` | 30 / 10 m | long axis parallel to beach |
| `lane_spacing` | 0.60 m | = tool_width when overlap=0 |
| `turn_radius` | 0.30 m | min arc radius; warn if < lane_spacing/2 |
| `auto_widen_lanes_for_turn` | false | auto-expand lane_spacing to 2×turn_radius |
| `boundary_margin` | 0.30 m | shrinks effective area |
| `waypoint_step` | 1.0 m | dense waypoints along each lane |
| `turn_style` | `arc` | also: `corner` |
| `autostart` | true | set false to preview path only |
| `start_delay_sec` | 15.0 s | wait for Nav2 to stabilise |

**Nav2 controller (RegulatedPurePursuit):**

| Parameter | Value |
|-----------|-------|
| `desired_linear_vel` | 0.5 m/s |
| `lookahead_dist` | 1.5 m |
| `rotate_to_heading_angular_vel` | 0.6 rad/s |
| `controller_frequency` | 20 Hz |
| `stop_on_failure` | false (waypoint follower) |

**Nav2 costmap sources:**
- Global: keepout mask (static layer) + inflation 0.6 m
- Local 12×12 m rolling: ZED point cloud (`/zed/filtered_cloud`) + ultrasonics + inflation

### `beach_robot_localization`
- `localization_full_test.launch.py` — full stack (ESP32 + EKF + optional GNSS)
- `localization_imu_compare.launch.py` — EKF with static TFs; used by coverage bringup
- `wheel_odometry_node.py` — integrates per-wheel velocities with geometry; publishes `/wheel/odom`
- EKF config in `config/ekf_*.yaml`; `/odometry/local` is the Nav2-facing topic

### `beach_wheel_mixer` (C++)
Converts `/cmd_vel` Twist → `/wheel_cmd` Float32MultiArray (m/s for 4 wheels).
Config: `config/mixer.yaml`. Skid-steer kinematics, accounting for asymmetric track widths.

### `beach_robot_esp32_bridge`
- Serial JSON protocol to/from ESP32 @ 230400 baud
- Encoder filter: drops sample if any wheel > 3.0 m/s or step > 1.0 m/s
- Wheel command send rate: 30 Hz (configurable via `wheel_cmd_send_rate_hz`)
- Stale cmd timeout: 0.5 s → sends zeros

---

## Coverage Test Procedure

### Prerequisites
1. Robot connected: ESP32 on `/dev/ttyESP32`, ZED USB, joystick USB
2. Workspace built: `colcon build --symlink-install`
3. Environment sourced: `source install/setup.bash`

### Step 1 — Path preview only (no movement)
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=false \
  coverage_pattern:=boustrophedon \
  area_origin_x:=0.0 area_origin_y:=0.0 \
  area_width:=2.0 area_height:=1.8 area_yaw:=0.0 \
  lane_spacing:=1.80 turn_radius:=0.90 \
  boundary_margin:=0.0 angular_scale:=1.0
```
Open RViz2 and visualise `/coverage/path_viz` (type: Path, frame: map, republish 3 sec, QoS: volatile).
Verify the lane pattern covers the expected rectangle.

### Step 2 — Autonomous boustrophedon run
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=true \
  coverage_pattern:=boustrophedon \
  area_origin_x:=0.0 area_origin_y:=0.0 \
  area_width:=2.0 area_height:=1.8 area_yaw:=0.0 \
  lane_spacing:=1.80 turn_radius:=0.90 \
  boundary_margin:=0.0 angular_scale:=1.0
```

### Flags when ZED is not available
```bash
  use_zed:=false
```
This removes ZED from the costmap — ultrasonics only for obstacle detection.

### Coverage bag recording
```bash
STAMP=$(date +%Y%m%d_%H%M%S)
ros2 bag record -o ~/beach_robot_logs/coverage/coverage_boustrophedon_${STAMP} \
  /cmd_vel /wheel_cmd /enc_vel \
  /wheel/odom /odometry/fusion_bno /odometry/local \
  /imu/data \
  /coverage/path \
  /plan /local_plan \
  /follow_waypoints/_action/feedback \
  /navigate_through_poses/_action/feedback \
  /local_costmap/costmap /global_costmap/costmap \
  /tf /tf_static
```

---

## Build & Deploy (on Jetson via SSH)

```bash
cd ~/beach_robot_ws
git pull
colcon build --symlink-install --packages-select <pkg>
source install/setup.bash
```

Full rebuild (slow):
```bash
colcon build --symlink-install
```

---

## Common Failure Modes

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Nav2 not active after launch | TF `odom→base_link` missing | Check ESP32 bridge connected, EKF publishing |
| Coverage node exits immediately | `follow_waypoints` action server not ready | Increase `start_delay_sec` |
| Robot doesn't turn (arc turns fail) | `lane_spacing < 2×turn_radius` | Add `auto_widen_lanes_for_turn:=true` or increase `lane_spacing` |
| ZED costmap errors | ZED not connected | Pass `use_zed:=false` |
| `/odometry/local` missing | Localization not launched | Verify `use_robot_stack:=true` (default) |
| Waypoints skipped silently | `stop_on_failure: false` in nav2_params | Normal; Nav2 continues to next waypoint |
| Encoder spike drops | Wheel velocity > 3 m/s or > 1 m/s step | Expected filter behaviour; increase thresholds if needed |

---

## Analysis Tools (in `beach_robot_bringup`)

| Tool | Purpose |
|------|---------|
| `preflight_check.py` | Checks all topics alive before running |
| `localization_pose_report.py` | Post-process bag → pose accuracy CSV/SVG |
| `drive_straight_odom.py` | Commanded straight-drive + odom validation |
| `analyze_spin_tune.py` | Angular velocity tuning analysis |
| `wheel_response_test.py` | Step-response per wheel |
| `coverage_bag_report.py` | Post-process coverage bag → CSV tables for thesis |

### `coverage_bag_report` — usage

```bash
# After sourcing workspace:
ros2 run beach_robot_bringup coverage_bag_report \
  ~/beach_robot_logs/coverage/coverage_boustrophedon_20260513_120000 \
  [--out ~/thesis_data/run1]

# Or run directly (no colcon build needed):
python3 src/beach_robot_bringup/beach_robot_bringup/tools/coverage_bag_report.py \
  <bag_path> [--out <output_dir>] [--min-lane-sec 2.0]
```

**Output CSV files:**

| File | Contents |
|------|---------|
| `pose_trajectory.csv` | time, x, y, yaw, phase (straight_forward/backward/turn/idle), cmd_vel |
| `cmd_vel.csv` | time series of linear_x and angular_z commands |
| `wheel_speeds.csv` | wheel_cmd and enc_vel per wheel (FL/FR/RL/RR) |
| `lane_tracking.csv` | per lane: planned_y, actual_y start/end/mean, y errors, timing, x range |
| `turn_tracking.csv` | per turn: y before/after, planned vs actual y_end, y error, duration |
| `coverage_summary.csv` | totals: duration, lanes, mean/max/RMS y error, mean vel, area |

**Phase detection**: uses `|angular_z| > 0.15 rad/s` to classify straight vs turn phases. Short straight blips (< 2s) are merged into adjacent turns. Adjust `--min-lane-sec` if lanes are misclassified.

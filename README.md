# beach_robot_ws
แพ็กเกจนี้ทำระบบ coverage ทำความสะอาดชายหาดแบบ Boustrophedon (วิ่งไป-กลับเป็นแถว) โดยใช้:
- Coverage planner สร้าง waypoints ครอบคลุมพื้นที่สี่เหลี่ยม
- Nav2 ทำหน้าที่ “วิ่งตาม waypoint + หลบสิ่งกีดขวาง”
- Keepout/Boundary บังคับให้หุ่นไม่ออกนอกเขต ด้วย mask map

### Flow ล่าสุด
- พื้นที่: 10 x 30 m (ด้านยาว 30 m ขนานชายหาด)
- Start: มุมล่างซ้าย
- หุ่นหันหน้า: ขนานด้านยาว (แนว 30 m)
- Coverage: วิ่งเป็นแถวตามด้านยาว 30 m และ “ขยับแถว” ไปทางด้านกว้าง 10 m
⇒ เลี้ยวเฉพาะหัวแถว/ท้ายแถว (เลี้ยวน้อย)

### โครงสร้าง beach_robot_coverage_nav2
Nodes

1. coverage_follow_waypoints

- สร้าง waypoint แบบ boustrophedon แล้วส่งเข้า Nav2 ด้วย Action FollowWaypoints
- ใช้กรอบสี่เหลี่ยมที่กำหนดด้วยพารามิเตอร์ area.*
- sweep direction = แกน X ของสี่เหลี่ยม
- lane shift = แกน Y ของสี่เหลี่ยม

2. generate_keepout_mask

- สร้าง keepout_mask.pgm + keepout_mask.yaml สำหรับ boundary/keepout

Launch

beach_cleaning_bringup.launch.py

 - keepout mask server (/keepout_mask)
 - costmap filter info server (/costmap_filter_info)
 - nav2 navigation launch
 - coverage_follow_waypoints (autostart)

## build

บน **Jetson Nano** แรมน้อย ถ้า build ทุกแพ็กเกจพร้อมกัน (ค่าเริ่มต้น = parallel) มักจะ
OOM / ค้าง ให้ build แบบ **sequential** (ทีละแพ็กเกจ) เสมอ:

```bash
cd ~/beach_robot_ws
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential
source install/setup.bash
```

- `--symlink-install` — symlink ไฟล์ Python / launch / config เข้า `install/` แทนการคัดลอก
  ⇒ แก้โค้ด Python/yaml แล้วใช้ได้เลยโดยไม่ต้อง build ใหม่ (build ใหม่เฉพาะตอนเพิ่มไฟล์ใหม่
  หรือแก้ C++). หลังแก้ไฟล์ Python ให้ `source install/setup.bash` แล้วรันใหม่ได้เลย
- `--executor sequential` — build ทีละแพ็กเกจ (ค่าเริ่มต้นคือ `parallel`) ลดการใช้แรม
  พร้อมกัน กัน OOM บน Jetson
- `MAKEFLAGS="-j1"` — จำกัดให้ compiler (C++) ทำทีละ job ลดพีคแรมตอน build แพ็กเกจ C++
  เช่น `beach_wheel_mixer`, `zed_nav2_cloud_filter`. ถ้าเครื่องแรมเยอะใช้ `-j2` ได้

Build เฉพาะแพ็กเกจที่แก้ (เร็วกว่ามาก):
```bash
colcon build --symlink-install --executor sequential \
  --packages-select beach_robot_coverage_nav2
source install/setup.bash
```

Clean build (ช้า ใช้เมื่อมีปัญหา):
```bash
rm -rf build install log
MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential
```

## launch

> ก่อนรัน: ต่อ ESP32 (`/dev/ttyESP32`), ZED, joystick แล้ว `source install/setup.bash`

**1) Preview เส้นทางอย่างเดียว (ไม่ขยับหุ่น)** — ดูใน RViz2 ที่ topic `/coverage/path_viz`
(type: Path, frame: map):
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=false use_keepout:=false num_passes:=3 \
  area_width:=4.0 area_height:=4.0 \
  lane_spacing:=1.80 tool_width:=0.60 turn_radius:=0.90 boundary_margin:=0.30
```

**2) วิ่งอัตโนมัติ 3 รอบ (100% coverage)** — `start_coverage:=true`:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  start_coverage:=true use_keepout:=false num_passes:=3 \
  area_width:=4.0 area_height:=4.0 \
  lane_spacing:=1.80 tool_width:=0.60 turn_radius:=0.90 boundary_margin:=0.30
```

อาร์กิวเมนต์ที่ใช้บ่อย:

| arg | default | ความหมาย |
|-----|---------|----------|
| `num_passes` | `3` | จำนวน pass สลับเส้น (3 รอบ = ครอบคลุม 100%); `1` = pass เดียว |
| `lane_spacing` | `1.80` | ระยะเส้นภายใน pass (= `num_passes × tool_width`) |
| `tool_width` | `0.60` | ความกว้างตัวตัก |
| `turn_radius` | `0.90` | รัศมีโค้งหัวแถว (= lane_spacing/2) |
| `use_keepout` | `false` | `false` = ไม่ใช้ keepout (deadhead วนนอกพื้นที่ได้); `true` = ใช้ mask |
| `deadhead_style` | `outside` | การย้ายระหว่าง pass: `outside` วนนอกเขต / `direct` ตัดตรง |
| `start_coverage` | `true` | `false` = preview เฉย ๆ |
| `use_zed` | `true` | `use_zed:=false` ถ้าไม่ได้ต่อ ZED (ใช้ ultrasonic อย่างเดียว) |

> ให้ตั้ง `area_yaw:=0` และ `area_origin_*:=0` (แกน X ของ map ขนานชายหาด) — เรขาคณิต
> ของ lane/deadhead สมมติว่ากรอบพื้นที่ align กับ map frame

รายละเอียดพารามิเตอร์ทั้งหมด ดู `CLAUDE.md` (หัวข้อ Coverage) และคู่มือจูนทราย
`docs/sand_tuning_guide.md`

## run hardware bringup (อย่างเดียว ไม่มี Nav2)
```bash
ros2 launch beach_robot_bringup hardware_bringup.launch.py
```

## PASS/FAIL
- ros2 topic list | grep -E "enc_vel|imu/data|cmd_vel|wheel_cmd"
- ros2 run tf2_ros tf2_echo map base_link
- เปิด RViz ด้วย config
```bash
rviz2 -d $(ros2 pkg prefix beach_robot_bringup)/share/beach_robot_bringup/rviz/bringup.rviz
```

## Concept

### Mapless Nav2 มี boundary + keepout polygon / boustrophedon planner
โครง:
1. Coverage planner node → สร้าง waypoint แบบ boustrophedon ในพื้นที่สี่เหลี่ยม
2. ส่ง waypoint เข้า Nav2 FollowWaypoints
3. Nav2 ใช้ costmap จาก ZED depth + ultrasonic เพื่อหลบ obstacle

คำว่า “mapless” ในที่นี้หมายถึง ไม่ต้องทำ SLAM / ไม่ต้องมีแผนที่สิ่งแวดล้อมจริง
แต่สำหรับ “boundary/keepout” เราจะใช้ “mask map” แบบง่าย ๆ (เป็นแผนที่ดำ/ขาว) เพื่อบังคับเขตได้

### Costmap Filters + Keepout Mask
- ทำไฟล์ mask (.pgm/.png) ที่
    - พื้นที่อนุญาต = free
    - นอกพื้นที่ = lethal
    - โซนห้ามเข้า (keepout) = lethal
    - ให้ Nav2 โหลด mask ผ่าน map_server/costmap_filter_info_server
ผลคือ:
- planner/controller จะไม่วางเส้นออกนอกขอบ
- ไม่ต้องมี LiDAR/SLAM ก็ทำ boundary ได้

### ตั้งค่าพื้นที่สี่เหลี่ยม + ระยะเส้น (lane spacing)
เครื่องตักกว้าง W = ... m
เลือก overlap กันหลุด:
    - spacing = W * (1 - overlap) 
ตัวอย่าง (W = 0.6 m, overlap 15%):
    - spacing = 0.6 * (1-0.15) = 0.51 m

ระยะเผื่อขอบ (boundary margin) -> เพื่อให้ “ตัวตักครอบคลุมถึงขอบแต่ไม่เลยออกนอกเขต”
- ใช้ margin = W/2 m (ครึ่งหนึ่งของความกว้างตัวตัก)

### รูปแบบการเลี้ยว (ให้ครอบคลุมขอบ)
โหมด 1: U-turn แบบโค้ง
- กำหนด turning radius คร่าว ๆ เช่น R = ช่วง 0.8 ถึง 1.2 m
- ตอนท้ายแถว ให้เพิ่ม waypoint เป็น “โค้งครึ่งวงกลม” ไปแถวถัดไป (lane shift = lane spacing [m])

### ระบบพิกัด
GNSS-RTK → แนะนำให้ใช้ map frame เป็น ENU/UTM ที่นิ่ง
ทางปฏิบัติที่ง่าย:
1. ตั้ง datum ของ navsat_transform_node ให้เป็น “มุมหนึ่งของสี่เหลี่ยม” (หรือจุดเริ่มงาน)
2. coverage planner สร้าง waypoint ใน map frame (เมตร) → ส่งให้ Nav2

### Nav2 “mapless + keepout” config (ภาพรวม)
- global_costmap: ใช้ keepout mask เป็นหลัก (static layer จาก mask)
- local_costmap: rolling window + obstacle จาก ZED/ultrasonic

1. `ZED depth` เข้า costmap (Nav2 รับได้ทั้ง PointCloud2 หรือ LaserScan)
2. `Ultrasonic` ใช้ `range sensor layer` เป็น safety ใกล้ตัว (หยุด/ชะลอ) 

`range sensor layer` is costmap plugin that converts 1D distance data from `sensor_msgs/Range` into 2D, cone-shaped, or circular obstacle representations on the costmap. Key parameters include `topics` (list of sensor topics), `phi` (sensor angular width), `inflation_radius`, and `clear_on_max_reading`

### ขั้นตอนทำงานจริง
Step 1: สร้าง boundary polygon
- ถ้าพื้นที่สี่เหลี่ยม: เอา 4 มุมจาก GNSS (วัด/ขับไปจุดมุมแล้วกดบันทึก)
- ได้ (lat,lon) 4 จุด → แปลงเป็น ENU/UTM (หรือใช้ navsat datum)

Step 2: สร้าง keepout zones
- เช่น “แนวลงน้ำ”, “โซนอันตราย”, “กองหิน”
- เพิ่มเป็น polygon ในไฟล์เดียวกัน

Step 3: สร้าง keepout mask map
- สร้างรูปภาพขนาดพอดีกับพื้นที่ (เช่น 120m x 60m, resolution 0.1m/pixel)
- ระบาย “นอกพื้นที่ + keepout” เป็นค่า lethal (254)

Step 4: ตั้ง Nav2 costmap filters
- โหลด mask + filter info server
- ใส่ filter ใน global/local costmap

Step 5: Coverage planner node
- input: rectangle corners + spacing + margin + turn radius
- output: waypoints list
- ส่งให้ Nav2 ผ่าน FollowWaypoints

### Generate boundary mask
```bash
ros2 run beach_robot_coverage_nav2 generate_keepout_mask --ros-args \
  -p output_dir:=$HOME/beach_robot_ws/src/beach_robot_coverage_nav2/config \
  -p mask_basename:=keepout_mask \
  -p area.origin_x:=0.0 -p area.origin_y:=0.0 \
  -p area.width:=20.0 -p area.height:=15.0 -p area.yaw:=0.0 \
  -p resolution:=0.10 -p outer_margin:=5.0
```
### ต้องต่อกับ localization


### รัน Nav2 + Coverage (3-pass boustrophedon)
ดูคำสั่งเต็มที่หัวข้อ [launch](#launch) ด้านบน. สรุปสิ่งที่ launch ทำ:
- เปิด `nav2_bringup/navigation_launch.py` (controller / planner / bt_navigator)
- เปิด node `coverage_follow_waypoints` → สร้าง waypoints แบบ multipass แล้วส่งให้ Nav2
- `use_keepout:=true` (ออปชัน): เพิ่ม publish `/keepout_mask` + `/costmap_filter_info`
  และใช้ `nav2_params_keepout.yaml`. ค่าเริ่มต้น `use_keepout:=false` ไม่ใช้ keepout
  (ใช้ `nav2_params_nokeepout.yaml` เพื่อให้ deadhead วนนอกพื้นที่ได้)
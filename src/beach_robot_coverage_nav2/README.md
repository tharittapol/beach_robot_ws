# beach_robot_coverage_nav2
แพ็กเกจนี้ทำระบบ “ทำความสะอาดแบบครอบคลุมพื้นที่” (coverage) สำหรับหุ่นวิ่งบนชายหาด โดยใช้:
- Boustrophedon coverage (วิ่งไป-กลับเป็นแถว)
- Spiral coverage (ก้นหอยจากวงนอกเข้าวงใน)
- ใช้ Nav2 เป็นตัว “นำทางไปตาม waypoint” และทำ obstacle avoidance
- ใช้ Keepout / Boundary mask บังคับให้หุ่นไม่ออกนอกเขต (mapless ในความหมายว่าไม่ต้อง SLAM map)

## Nodes ที่แพ็กเกจนี้มี
1) coverage_follow_waypoints
    - สร้าง waypoint แบบ boustrophedon หรือ spiral แล้วส่งเข้า Nav2 ผ่าน Action FollowWaypoints
    - publish preview path ที่ `/coverage/path` สำหรับดูใน RViz

2) generate_keepout_mask
    - สร้างไฟล์ keepout_mask.pgm + keepout_mask.yaml สำหรับ boundary/keepout (นอกพื้นที่เป็น “ห้ามเข้า”)

## Launch ที่แพ็กเกจนี้มี
1) beach_cleaning_bringup.launch.py
    - รันครบ: keepout mask server + costmap filter info server + nav2 + coverage planner

## Flow
- Start: มุมล่างซ้าย
- พื้นที่สี่เหลี่ยมผืนผ้าแนวตั้ง
- หุ่นหันหน้า: ขนานด้านยาว
- Coverage: วิ่งเป็นแถวตามด้านยาว และ “ขยับแถว” ไปทางด้านกว้าง ⇒ เลี้ยวเฉพาะหัวแถว/ท้ายแถว (เลี้ยวน้อย)

## กำหนดพิกัดพื้นที่แบบ “มุมล่างซ้ายเป็น origin”
กำหนด origin_x/origin_y ให้เป็นมุมล่างซ้ายใน map
1) รัน localization ให้ได้ map.
2) ขับหุ่นไปที่ “มุมล่างซ้ายจริง”
3) อ่านค่า (x,y) ของหุ่นใน map ด้วย:
    ```bash
    ros2 run tf2_ros tf2_echo map base_link
    ```
    จด Translation: x=..., y=...
4) เอาค่านี้ไปใส่เป็น:
    - area.origin_x = x
    - area.origin_y = y

## ค่า Coverage
เครื่องตักกว้าง 0.60 m

ค่าเริ่มต้นใช้ lane spacing = 0.60 m ตามความกว้างเครื่องตัก:
- spacing = 0.60 m
- lane count ~ 10/0.60 ≈ 17 lanes
- แถวละ 30 m ที่ 0.5 m/s → ~60s/แถว

พารามิเตอร์แนะนำ
- area.width = 30.0 (แนววิ่งขนานชายหาด / ด้านยาว)
- area.height = 10.0 (แนวขยับแถว / ด้านกว้าง)
- tool_width = 0.60
- overlap = 0.0 หรือกำหนด lane_spacing = 0.60 โดยตรง
- pattern = boustrophedon หรือ spiral
- boundary_margin = 0.30 (เท่ากับ tool_width/2)
- waypoint_step = 1.0
- turn_style = arc
- turn_radius = 0.30 ถ้าต้องการ lane spacing 0.60 m
- ถ้าหุ่นเลี้ยวแคบไม่ไหว ให้เพิ่ม lane_spacing เอง หรือใช้ auto_widen_lanes_for_turn=true
- กรณี forward-only U-turn ต้องใช้ lane_spacing >= 2*turn_radius เช่น turn_radius=0.8 → lane_spacing≈1.6 m

เรื่อง “หุ่นหันหน้าขนานด้านยาวตอนเริ่ม”
planner จะสร้างแถวแรกให้ yaw = แนวแกน X ของสี่เหลี่ยม
ดังนั้นถ้า area.yaw ทำให้แกน X ขนานชายหาด หุ่นจะเริ่มหันถูกทิศ

## area.yaw คืออะไร (ทำให้ขนานชายหาด)
- area.yaw คือมุมหมุนของสี่เหลี่ยม (เรเดียน) ใน frame map
- ถ้าแกน X ของ map ขนานชายหาดอยู่แล้ว → area.yaw = 0.0
- ถ้า map เอียงจากแนวชายหาด → ตั้ง area.yaw ให้แกน X ของสี่เหลี่ยม “ขนานชายหาด”

แปลงองศาเป็นเรเดียน:
10° = 0.1745
15° = 0.2618

## Keepout / Boundary Mask
### Example Generate mask ให้เป็น 30x10
```bash
ros2 run beach_robot_coverage_nav2 generate_keepout_mask --ros-args \
  -p output_dir:=$HOME/beach_robot_ws/src/beach_robot_coverage_nav2/config \
  -p mask_basename:=keepout_mask \
  -p area.origin_x:=0.0 -p area.origin_y:=0.0 \
  -p area.width:=30.0 -p area.height:=10.0 -p area.yaw:=0.0 \
  -p resolution:=0.10 -p outer_margin:=5.0
```
หมายเหตุ: mask map ไม่ควรหมุนด้วย yaw ใน YAML origin (ควรเป็น 0.0)
การหมุนพื้นที่ให้ขนานชายหาดให้ใช้ area.yaw ใน planner แทน
### เช็คว่า keepout ทำงานจริง
หลังรัน bringup:
```bash
ros2 topic echo /keepout_mask --once
ros2 topic echo /costmap_filter_info --once
```

## Run step (ระบบเต็ม)
### Terminal 1: localization
```bash
ros2 launch beach_robot_localization localization.launch.py
```
เช็ค TF:
```bash
ros2 run tf2_ros tf2_echo map base_link
```
### Terminal 2: Nav2 + keepout + coverage
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py
```

เลือก pattern ได้:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  coverage_pattern:=boustrophedon \
  lane_spacing:=0.60 \
  turn_radius:=0.30
```

หรือก้นหอยวงนอกเข้าวงใน:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  coverage_pattern:=spiral \
  lane_spacing:=0.60 \
  turn_radius:=0.30
```

ถ้าหุ่นเลี้ยวแคบไม่ไหว:
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py \
  coverage_pattern:=boustrophedon \
  turn_radius:=0.80 \
  auto_widen_lanes_for_turn:=true
```

## ปรับพารามิเตอร์ใน launch
ไปแก้ใน launch/beach_cleaning_bringup.launch.py ที่ node coverage_follow_waypoints ให้มี:
- area.width: 30.0
- area.height: 10.0
- origin_x/origin_y: มุมล่างซ้ายใน map (ให้อ่าน x,y จาก tf2_echo แล้วใส่)
- area.yaw: ทำให้วิ่งขนานชายหาด

## Visualization / Debug
### RViz2
Add displays:
- TF
- Map → /keepout_mask
- Costmap → /global_costmap/costmap, /local_costmap/costmap
- Odometry → /odometry/local และ /odometry/global
- Path (ถ้ามี)
### เช็ค Nav2 สั่งงานจริงไหม
```bash
ros2 topic echo /cmd_vel
```

## ZED obstacle safety stop

เมื่อ point cloud ตรวจพบวัตถุด้านหน้าภายใน 2.0 m ระบบจะ:

- publish `/safety/e_stop=true` ให้ ESP32 bridge บังคับล้อหยุด
- ร้อง buzzer เป็นระยะ
- คง Nav2 goal เดิมไว้
- เมื่อทางโล่งต่อเนื่องเกิน 3 วินาที publish `/safety/e_stop=false` แล้ว Nav2 วิ่งต่อ

Coverage bringup เปิด **กล้อง ZED + สาย E-stop** (bridge OR `/e_stop` จอย กับ `/safety/e_stop`)
ให้พร้อม แต่ **ไม่ได้ spawn ตัว detector เอง** — รัน node ตรวจจับแยก (persistent เหมือน GNSS):

```bash
# เทอร์มินอล 1: bringup (เปิดกล้อง ZED ให้แล้ว)
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py
# เทอร์มินอล 2: detector แยก — กล้องเปิดจาก bringup แล้ว จึง launch_zed:=false
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py launch_zed:=false
```

ทดสอบกล้อง + safety node แบบ standalone (detector launch เปิดกล้องเองในตัว):

```bash
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py
```

สำหรับหยุดล้อจริง ต้องมี `beach_robot_esp32_bridge` รันอยู่ด้วย

ถ้า ZED camera และ `/zed/filtered_cloud` รันอยู่แล้ว:

```bash
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py launch_zed:=false
```

ดูสถานะ:

```bash
ros2 topic echo /safety/obstacle_stop
ros2 topic echo /safety/obstacle_distance
ros2 topic echo /safety/e_stop
```

ขอบเขตตรวจจับเป็นกล่องใน frame `base_link`:

- ลึก/ด้านหน้า X: `min_forward_distance` ถึง `stop_distance`
- กว้าง Y: `-cone_half_width` ถึง `+cone_half_width` (ความกว้างรวม = 2 เท่า)
- สูง Z: `min_z` ถึง `max_z`
- ต้องมีอย่างน้อย `min_points` จุดในกล่องจึงถือว่าเป็น obstacle

ตัวอย่างกล่องตรวจจับลึก 0.30-2.00 m, กว้างรวม 1.50 m, สูง 0.15-1.40 m:

```bash
ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py \
  min_forward_distance:=0.30 stop_distance:=2.0 \
  cone_half_width:=0.75 min_z:=0.15 max_z:=1.40 \
  min_points:=8 clear_time_sec:=3.0
```

`filter_min_range`, `filter_max_range`, `filter_min_z`, `filter_max_z` ใช้ปรับ
ขอบเขต point cloud ชั้นแรก ค่าชั้น filter ต้องครอบคลุมกล่อง detector เสมอ เช่น
ถ้าต้องตรวจถึงความสูง 1.80 m ให้เพิ่มทั้ง `max_z:=1.80 filter_max_z:=1.80`
ค่า `filter_*` จาก launch นี้มีผลเมื่อ `launch_zed:=true` เท่านั้น

เช็กค่าที่ node ใช้งานหลัง launch:

```bash
ros2 param dump /zed_obstacle_stop
ros2 param dump /zed_cloud_filter_node
```

### ดู graph
```bash
rqt_graph
```
### Debug “ทิศผิด / วิ่งสลับแกน”
อาการ: หุ่นวิ่งเป็นแถวสั้น (10 m) แทน 30 m
แก้: สลับ area.width/area.height หรือ area.yaw ผิด

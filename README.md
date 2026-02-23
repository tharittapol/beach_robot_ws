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
```bash
cd ~/beach_robot_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## run bringup
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


### รัน Nav2 + Keepout + Coverage (boustrophedon)
```bash
ros2 launch beach_robot_coverage_nav2 beach_cleaning_bringup.launch.py
```
สิ่งที่ launch นี้ทำ:
- publish /keepout_mask (OccupancyGrid)
- publish /costmap_filter_info (type=0 สำหรับ keepout)
- เปิด nav2_bringup/navigation_launch.py
- เปิด node coverage_follow_waypoints แล้วส่ง waypoints ให้ Nav2
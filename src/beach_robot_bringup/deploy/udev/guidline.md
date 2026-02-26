## ต้องรู้ VID/PID ของอุปกรณ์
```bash
udevadm monitor --udev --property
```

## ติดตั้ง rule:
```bash
sudo cp ~/beach_robot_ws/src/beach_robot_bringup/deploy/udev/99-beach-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## เช็ค:
```bash
ls -l /dev/ttyESP32 /dev/input/js_joy /dev/ttyGNSS /dev/cam
```
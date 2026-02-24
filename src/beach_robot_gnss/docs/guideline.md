# Confirm the caster login works (from Jetson)
```bash
curl --http0.9 -v --user "tharittapol.big-at-gmail-d-com:none" \
  "http://rtk2go.com:2101/TH-Kukot" --max-time 5 --output /dev/null
```
`Expected`: see ICY 200 OK in the headers and then it will stream binary until timeout.

# Confirm NTRIP is truly streaming RTCM (strong proof)
```bash
printf "GET /TH-Kukot HTTP/1.0\r\nUser-Agent: NTRIP\r\nAuthorization: Basic $(printf 'tharittapol.big-at-gmail-d-com:none' | base64)\r\n\r\n" \
| timeout 5 nc rtk2go.com 2101 | wc -c
```
`Expected`: If it returns a large number (typically thousands to hundreds of thousands of bytes in 5s), RTCM is streaming.

# Confirm RTK status by raw GGA
```bash
sudo stty -F /dev/ttyGNSS 115200 raw -echo
sudo timeout 2 cat /dev/ttyGNSS | head
sudo timeout 10 cat /dev/ttyGNSS | tr -cd '\11\12\15\40-\176' | grep "^\$..GGA" | head -n 10
```

# Run your node and watch state switch to RTK
```bash
ros2 launch beach_robot_gnss um982_fix_nema.launch.py
```
then
```bash
ros2 topic echo /gps/fix_quality
ros2 topic echo /gps/rtk_state
```
`Expected`:
After 10–120 seconds (depends on sky view), you should see:
- RTK_FLOAT (fix_quality 5)
- then sometimes RTK_FIX (fix_quality 4)
If it stays at 1/2 only, corrections are not being applied yet (or not valid for your location).
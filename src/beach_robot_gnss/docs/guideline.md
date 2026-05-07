# Confirm the caster login works (from Jetson)

Do not use normal `curl` as the main NTRIP check. Some SNIP/RTK2GO casters
reject plain HTTP/1.1 clients with:

```text
You have just sent nonsense to the NTRIP Caster.
```

Use a minimal NTRIP client request instead.

```bash
AUTH=$(printf 'wachiramate-at-gmail-d-com:none' | base64)
printf "GET /SRPDI1968 HTTP/1.0\r\nUser-Agent: NTRIP UM982/1.0\r\nAuthorization: Basic ${AUTH}\r\n\r\n" \
| timeout 5 nc rtk2go.com 2101 | tee /tmp/ntrip_check.bin | head -c 120
```
`Expected`: see `ICY 200 OK` or `HTTP/... 200` and then binary RTCM bytes.

# Confirm NTRIP is truly streaming RTCM (strong proof)
```bash
AUTH=$(printf 'wachiramate-at-gmail-d-com:none' | base64)
printf "GET /SRPDI1968 HTTP/1.0\r\nUser-Agent: NTRIP UM982/1.0\r\nAuthorization: Basic ${AUTH}\r\n\r\n" \
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

# ESP32 Beach Controller – Arduino Board Config

Tested with:

- **Arduino IDE**: 1.8.19
- **ESP32 core**: `esp32 by Espressif Systems` (version 3.3.3)

## Board Settings (Tools → ...)

- **Board**: ESP32S3 Dev Module
- **USB CDC On Boot**: Enabled
- **Flash Size**: 16MB (128Mb)
- **PSRAM**: OPI PSRAM
- **Flash Mode**: QIO 80MHz
- **Partition Scheme**: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
- **CPU Frequency**: 240MHz (WiFi)
- **Upload Speed**: 921600
- **Upload Port**: `/dev/ttyACM0` (on Jetson / Linux)
- **USB Mode**: Hardware CDC and JTAG
- **Monitor Baud Rate**: 115200

## Serial Mapping

- On ESP32 firmware use:

  ```cpp
  #define BRIDGE_SERIAL Serial0

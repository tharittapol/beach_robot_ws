// Minimal send/receive JSON for ROS2 bridge
// - Receive: {"wheel_cmd":[v_fl,v_fr,v_rl,v_rr]}
// - Send:    {"enc_vel":[v_fl,v_fr,v_rl,v_rr]}
// Encoder data is mocked (no real hardware).

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// Use UART0 as main serial port (matches /dev/ttyACM0)
#define BRIDGE_SERIAL Serial0

// wheel command from ROS2 [FL, FR, RL, RR] in m/s
static float wheel_cmd[4] = {0, 0, 0, 0};

// --- Command timeout ---
// If no wheel_cmd received from ROS2 for this many ms, set wheel_cmd to 0
const unsigned long CMD_TIMEOUT_MS = 500;   //  tune!
static unsigned long last_cmd_ms = 0;

// timing
static unsigned long last_telemetry_ms = 0;
const unsigned long TELEMETRY_PERIOD_MS = 100;   // send encoders every 100ms

// buffer for incoming line from serial
static String rx_line;

// === Encoder → velocity config ===
// Change these to match encoder/motor/wheel
const float ENCODER_PPR      = 360.0f;   // pulses per motor rev (A or B channel)
const float QUAD_FACTOR      = 1.0f;     // 1 if using only RISING on A, 2 or 4 if x2/x4 decoding
const float GEAR_RATIO       = 30.0f;    // motor rev / wheel rev (adjust!)
const float WHEEL_RADIUS_M   = 0.10f;    // wheel radius in meters (adjust!)

// mock encoder counts [FL, FR, RL, RR]
static long encoders[4] = {0, 0, 0, 0};

// previous counts and time for velocity computation
static long prev_encoders[4] = {0, 0, 0, 0};
static unsigned long prev_vel_ms = 0;

// computed velocities (m/s)
static float enc_vel_mps[4] = {0, 0, 0, 0};

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // 0x28 = default address, change if needed
bool imu_ok = false;


// -----------------------------------------------------------
// Parse one JSON line from ROS2, extract wheel_cmd array
// Expected format: {"wheel_cmd":[v_fl,v_fr,v_rl,v_rr]}
// -----------------------------------------------------------
void processLine(const String &line)
{
  // basic check
  if (!line.startsWith("{")) {
    return;
  }

  // find start and end of array [...]
  int idx = line.indexOf('[');
  int end = line.indexOf(']', idx);
  if (idx < 0 || end < 0) {
    return;
  }

  String arr = line.substring(idx + 1, end);
  // arr example: "0.1,0.2,0.1,0.2"

  float vals[4] = {0, 0, 0, 0};
  int valIndex = 0;

  int start = 0;
  while (start < arr.length() && valIndex < 4) {
    int comma = arr.indexOf(',', start);
    String token;
    if (comma == -1) {
      token = arr.substring(start);
      start = arr.length();
    } else {
      token = arr.substring(start, comma);
      start = comma + 1;
    }
    token.trim();
    if (token.length() > 0) {
      vals[valIndex] = token.toFloat();
      valIndex++;
    }
  }

  if (valIndex == 4) {
    // update global command (no interrupts needed here, single-thread)
    for (int i = 0; i < 4; ++i) {
      wheel_cmd[i] = vals[i];
    }
    // record time of last valid command
    last_cmd_ms = millis();
  }
}

// -----------------------------------------------------------
// Read bytes from Serial until '\n', then process the line
// -----------------------------------------------------------
void readSerialLines()
{
  while (BRIDGE_SERIAL.available() > 0) {
    char c = BRIDGE_SERIAL.read();
    if (c == '\n') {
      String line = rx_line;
      rx_line = "";
      line.trim();
      if (line.length() > 0) {
        processLine(line);
      }
    } else if (c != '\r') {
      rx_line += c;
      // prevent overly long line
      if (rx_line.length() > 200) {
        rx_line = "";
      }
    }
  }
}

// Set wheel_cmd to 0 if no command has been received recently
void applyWheelCmdTimeout()
{
  unsigned long now = millis();

  // If we never got a command yet, do nothing
  if (last_cmd_ms == 0) {
    return;
  }

  // If last command is too old, zero all wheel commands
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) {
    for (int i = 0; i < 4; ++i) {
      wheel_cmd[i] = 0.0f;
    }
  }
}

// Read IMU and format data into JSON pieces
// embed them into the same JSON line as enc_vel
void appendImuJson()
{
  // if (!imu_ok) {
  //   // If IMU not available, still send empty arrays (optional)
  //   BRIDGE_SERIAL.print(",\"imu_quat\":[0,0,0,1]");
  //   BRIDGE_SERIAL.print(",\"imu_gyro\":[0,0,0]");
  //   BRIDGE_SERIAL.print(",\"imu_lin_acc\":[0,0,0]");
  //   return;
  // }

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Quaternion: [x, y, z, w]
  BRIDGE_SERIAL.print(",\"imu_quat\":[");
  BRIDGE_SERIAL.print(quat.x(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(quat.y(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(quat.z(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(quat.w(), 6);
  BRIDGE_SERIAL.print("]");

  // Gyro (rad/s or deg/s depending on library config, check docs)
  BRIDGE_SERIAL.print(",\"imu_gyro\":[");
  BRIDGE_SERIAL.print(gyro.x(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(gyro.y(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(gyro.z(), 6);
  BRIDGE_SERIAL.print("]");

  // Linear acceleration (m/s^2)
  BRIDGE_SERIAL.print(",\"imu_lin_acc\":[");
  BRIDGE_SERIAL.print(linacc.x(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(linacc.y(), 6);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(linacc.z(), 6);
  BRIDGE_SERIAL.print("]");
}

// Update mock encoder counts and compute velocity in m/s,
// then send JSON: {"enc_vel":[v_fl,v_fr,v_rl,v_rr]}
void updateAndSendEncoderVel()
{
  // Apply timeout logic before using wheel_cmd[]
  applyWheelCmdTimeout();

  unsigned long now = millis();

  if (prev_vel_ms == 0) {
    prev_vel_ms = now;
    // skip if first prev
    return;
  }
  float dt = (now - prev_vel_ms) / 1000.0f; // seconds

  if (dt <= 0.0f) {
    // first call or timer glitch; just send zeros
    dt = 1e-3f;
  }

  // simulate encoder counts based on current wheel_cmd (mock)
  for (int i = 0; i < 4; ++i) {
    float mag = fabs(wheel_cmd[i]);
    long delta = (long)(mag * 10.0f);
    if (wheel_cmd[i] >= 0.0f) {
      encoders[i] += delta;
    } else {
      encoders[i] -= delta;
    }
  }

  // compute delta_count and velocity (m/s)
  for (int i = 0; i < 4; ++i) {
    long dcount = encoders[i] - prev_encoders[i];
    prev_encoders[i] = encoders[i];

    // motor revolutions during dt
    float motor_rev = dcount / (ENCODER_PPR * QUAD_FACTOR);

    // wheel revolutions
    float wheel_rev = motor_rev / GEAR_RATIO;

    // distance in meters
    float distance = wheel_rev * 2.0f * (float)M_PI * WHEEL_RADIUS_M;

    // velocity m/s
    enc_vel_mps[i] = distance / dt;
  }

  prev_vel_ms = now;

  // JSON start: {"enc_vel":[...]
  BRIDGE_SERIAL.print("{\"enc_vel\":[");
  BRIDGE_SERIAL.print(enc_vel_mps[0], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[1], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[2], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[3], 4);
  BRIDGE_SERIAL.print("]");

  // Append IMU fields: "imu_quat", "imu_gyro", "imu_lin_acc"
  appendImuJson();

  // Close JSON object
  BRIDGE_SERIAL.println("}");
  
  // So each telemetry line is now
  // {
  // "enc_vel":[...],
  // "imu_quat":[x,y,z,w],
  // "imu_gyro":[gx,gy,gz],
  // "imu_lin_acc":[ax,ay,az]
  // }
}


void setup()
{
  BRIDGE_SERIAL.begin(115200);
  delay(1000);
  BRIDGE_SERIAL.println("{\"info\":\"ESP32 comm started\"}");

  // --- IMU init ---
  Wire.begin();  // use default I2C pins for ESP32-S3
  if (!bno.begin()) {
    BRIDGE_SERIAL.println("{\"info\":\"BNO055 not detected\"}");
    imu_ok = false;
  } else {
    imu_ok = true;
    delay(1000);                 // let it settle
    bno.setExtCrystalUse(true);  // use external crystal if wired
    BRIDGE_SERIAL.println("{\"info\":\"BNO055 initialized\"}");
  }
  
  last_telemetry_ms = millis();
}

void loop()
{
  // receive commands from ROS2 bridge
  readSerialLines();

  // periodically send mock encoder data
  unsigned long now = millis();
  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    updateAndSendEncoderVel();
  }
}

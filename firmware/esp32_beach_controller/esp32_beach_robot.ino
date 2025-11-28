// Minimal send/receive JSON for ROS2 bridge
// - Receive: {"wheel_cmd":[v_fl,v_fr,v_rl,v_rr]}
// - Send:    {"enc_vel":[v_fl,v_fr,v_rl,v_rr]}
// Encoder data is mocked (no real hardware).

#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// Use UART0 as main serial port (matches /dev/ttyACM0)
#define BRIDGE_SERIAL Serial0

// wheel command from ROS2 [FL, FR, RL, RR] in m/s
static float wheel_cmd[4] = {0, 0, 0, 0};

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

// Update mock encoder counts and compute velocity in m/s,
// then send JSON: {"enc_vel":[v_fl,v_fr,v_rl,v_rr]}
void updateAndSendEncoderVel()
{
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

  // send JSON: {"enc_vel":[v_fl,v_fr,v_rl,v_rr]}
  BRIDGE_SERIAL.print("{\"enc_vel\":[");
  BRIDGE_SERIAL.print(enc_vel_mps[0], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[1], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[2], 4);
  BRIDGE_SERIAL.print(",");
  BRIDGE_SERIAL.print(enc_vel_mps[3], 4);
  BRIDGE_SERIAL.println("]}");
}


void setup()
{
  BRIDGE_SERIAL.begin(115200);
  delay(1000);
  BRIDGE_SERIAL.println("{\"info\":\"ESP32 comm mock started\"}");
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

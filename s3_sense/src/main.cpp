#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_NeoPixel.h>
#include <MS5607.h>
#include <Adafruit_LIS3MDL.h>
#include <pid.h>

//i2c for IMU, BARO
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C1_SDA 12
#define I2C1_SCL 11

Adafruit_LSM6DSOX imu;
Adafruit_LIS3MDL mag;
extern TwoWire Wire1;

/* ===============================
    SENSOR DATA & ORIENTATION
================================ */

struct Quaternion {
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
};

Quaternion quat;
float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
unsigned long lastFilterTime = 0;

const float gyroMeasError = M_PI * (40.0f / 180.0f);
const float beta = sqrtf(3.0f / 4.0f) * gyroMeasError;

/* ===============================
    ANGULAR RATE CONTROL (PID)
================================ */

float target_roll_rate  = 0.0f;
float target_pitch_rate = 0.0f;
float target_yaw_rate   = 0.0f;

float target_roll_deg  = 0.0f;
float target_pitch_deg = 0.0f;
float target_yaw_deg   = 0.0f;  // CHANGED: this is now treated as absolute yaw setpoint (deg)

float measured_roll_rate  = 0.0f;
float measured_pitch_rate = 0.0f;
float measured_yaw_rate   = 0.0f;

const float rateFilterAlpha = 0.2f;

// ---- Inner loop: angular rate → servo µs deflection ----
PIDController rollRatePID (1.0f,  0.0f, 0.0f);
PIDController pitchRatePID(1.0f,  0.0f, 0.0f);
PIDController yawRatePID  (0.0f,  0.0f, 0.0f);

// ---- Outer loop: inline PI for roll & pitch angle → Euler rate (deg/s) ----
// Tune via serial: RAP / RAI for roll, PAP / PAI for pitch
float angleKp_roll  = 5.0f;   // proportional gain  (start here — matches your working hardcode)
float angleKi_roll  = 0.2f;   // integral gain      (add slowly: 0.05–0.2)
float angleInt_roll = 0.0f;   // running integral
const float ANGLE_INT_LIMIT = 30.0f;  // deg·s — prevents windup; tune alongside Ki

float angleKp_pitch  = 5.0f;
float angleKi_pitch  = 0.2f;
float angleInt_pitch = 0.0f;

unsigned long lastAngleLoopTime = 0;  // for dt in the integral

// Yaw angle loop still uses PIDController (rate only, no change)
PIDController yawAnglePID(0.0f, 0.0f, 0.0f);

// CHANGED: default to angle control mode (was false)
bool useAngleControl = true;

// Output rate limits from angle loop → inner rate setpoint
// ADDED: explicit constants so they're easy to find and tune
const float MAX_RATE_FROM_ANGLE = 250.0f;  // deg/s — outer loop rate cap
const float MAX_YAW_RATE        =  90.0f;  // deg/s — yaw usually needs less authority

/* ===============================
    PWM CONFIG
================================ */
const int freq      = 50;
const int resolution = 14;
const int minPulse  = 1000;
const int maxPulse  = 2000;
const int servoZeroPulse = 1500;
const int periodUs  = 20000;

const int servoPins[]    = {5, 6, 7, 15};
const int numServos      = 4;
const int escPin         = 4;
const int servoChannels[] = {0, 1, 2, 3};
const int escChannel      = 4;

int currentThrottle = minPulse;

unsigned long lastSerialActivity = 0;
const unsigned long USB_HEARTBEAT_TIMEOUT = 10000;

/* ===============================
    SERVO CALIBRATION (TRIM)
================================ */
const int servoOffsets[] = {-15, -20, -80, 30};

const float mixCoeffs[numServos][3] = {
  {  1.0f,  0.0f, -1.0f },  // S0
  {  0.0f,  1.0f, -1.0f },  // S1
  {  0.0f, -1.0f, -1.0f },  // S2
  { -1.0f,  0.0f, -1.0f }   // S3
};

/* ===============================
    UTILITY
================================ */
uint32_t pulseToDuty(int microseconds) {
  return (uint32_t)((microseconds * ((1 << resolution) - 1)) / periodUs);
}

float normalizeAngle180(float angle) {
  while (angle >  180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// ADDED: parse a serial command token of the form "KEY:VALUE"
// Returns true and populates value if the line starts with key.
// Used for the extended tuning protocol below.
bool parseKV(const String& line, const char* key, float& value) {
  String k(key);
  if (line.startsWith(k + ":")) {
    value = line.substring(k.length() + 1).toFloat();
    return true;
  }
  return false;
}

/* ===============================
    ORIENTATION ESTIMATION
================================ */
void updateOrientation() {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  mag.read();

  // Coordinate frame remapping (unchanged from original)
  float ax_raw = accel.acceleration.z;
  float ay_raw = accel.acceleration.x;
  float az_raw = accel.acceleration.y;

  float gx_raw = gyro.gyro.z;
  float gy_raw = gyro.gyro.x;
  float gz_raw = gyro.gyro.y;

  float raw_roll  = gx_raw * 180.0f / M_PI;
  float raw_pitch = gy_raw * 180.0f / M_PI;
  float raw_yaw   = gz_raw * 180.0f / M_PI;

  measured_roll_rate  = rateFilterAlpha * raw_roll  + (1.0f - rateFilterAlpha) * measured_roll_rate;
  measured_pitch_rate = rateFilterAlpha * raw_pitch + (1.0f - rateFilterAlpha) * measured_pitch_rate;
  measured_yaw_rate   = rateFilterAlpha * raw_yaw   + (1.0f - rateFilterAlpha) * measured_yaw_rate;

  float mx_raw =  mag.z;
  float my_raw = -mag.y;
  float mz_raw =  mag.x;

  float norm = sqrtf(mx_raw*mx_raw + my_raw*my_raw + mz_raw*mz_raw);
  if (norm > 0) { mx_raw /= norm; my_raw /= norm; mz_raw /= norm; }
  else return;

  norm = sqrtf(ax_raw*ax_raw + ay_raw*ay_raw + az_raw*az_raw);
  if (norm > 0) {
    float ax = ax_raw / norm;
    float ay = ay_raw / norm;
    float az = az_raw / norm;

    float gx = gx_raw, gy = gy_raw, gz = gz_raw;

    unsigned long now = micros();
    if (lastFilterTime == 0) {
      lastFilterTime = now;
      return;
    }
    float dt = (now - lastFilterTime) / 1e6f;
    
    if (dt < 0.001f) return;
    lastFilterTime = now;
    if (dt > 0.1f)  dt = 0.1f;

    float q0 = quat.q0, q1 = quat.q1, q2 = quat.q2, q3 = quat.q3;

    float hx = 2.0f*(q0*q0+q1*q1-0.5f)*mx_raw + 2.0f*(q1*q2-q0*q3)*my_raw + 2.0f*(q1*q3+q0*q2)*mz_raw;
    float hy = 2.0f*(q1*q2+q0*q3)*mx_raw + 2.0f*(q2*q2+q0*q0-0.5f)*my_raw + 2.0f*(q2*q3-q0*q1)*mz_raw;
    float hz = 2.0f*(q1*q3-q0*q2)*mx_raw + 2.0f*(q2*q3+q0*q1)*my_raw + 2.0f*(q0*q0+q3*q3-0.5f)*mz_raw;
    float bx = sqrtf(hx*hx + hy*hy);
    float bz = hz;

    float F[6] = {
      2.0f*(q1*q3 - q0*q2) - ax,
      2.0f*(q0*q1 + q2*q3) - ay,
      2.0f*(0.5f - q1*q1 - q2*q2) - az,
      2.0f*bx*(0.5f-q2*q2-q3*q3) + 2.0f*bz*(q1*q3-q0*q2) - mx_raw,
      2.0f*bx*(q1*q2-q0*q3)      + 2.0f*bz*(q0*q1+q2*q3) - my_raw,
      2.0f*bx*(q0*q2+q1*q3)      + 2.0f*bz*(0.5f-q1*q1-q2*q2) - mz_raw
    };

    float J[6][4] = {
      {-2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1},
      { 2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2},
      {0,        -4.0f*q1, -4.0f*q2,  0},
      {-2.0f*bz*q3, 2.0f*bz*q2, -4.0f*bx*q3-2.0f*bz*q1, -4.0f*bx*q2+2.0f*bz*q0},
      {-2.0f*bx*q3+2.0f*bz*q1, 2.0f*bx*q2+2.0f*bz*q0, 2.0f*bx*q1+2.0f*bz*q3, -2.0f*bx*q0+2.0f*bz*q2},
      { 2.0f*bx*q2, 2.0f*bx*q3-4.0f*bz*q1, 2.0f*bx*q0-4.0f*bz*q2, 2.0f*bx*q1}
    };

    float step[4] = {0, 0, 0, 0};
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 4; j++)
        step[j] += J[i][j] * F[i];

    norm = sqrtf(step[0]*step[0]+step[1]*step[1]+step[2]*step[2]+step[3]*step[3]);
    if (norm > 0) { step[0]/=norm; step[1]/=norm; step[2]/=norm; step[3]/=norm; }

    float dq0 = -0.5f*(q1*gx + q2*gy + q3*gz) - beta*step[0];
    float dq1 =  0.5f*(q0*gx + q2*gz - q3*gy) - beta*step[1];
    float dq2 =  0.5f*(q0*gy - q1*gz + q3*gx) - beta*step[2];
    float dq3 =  0.5f*(q0*gz + q1*gy - q2*gx) - beta*step[3];

    q0 += dq0*dt; q1 += dq1*dt; q2 += dq2*dt; q3 += dq3*dt;

    norm = sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    if (norm > 0) { q0/=norm; q1/=norm; q2/=norm; q3/=norm; }

    quat.q0=q0; quat.q1=q1; quat.q2=q2; quat.q3=q3;

    roll_deg  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2)) * 180.0f/M_PI;
    pitch_deg = asinf (2.0f*(q0*q2-q3*q1))                           * 180.0f/M_PI;
    yaw_deg   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3)) * 180.0f/M_PI;
  }
}

/* ===============================
    RATE COMMAND COMPUTATION
    (outer angle loop → inner rate loop)
================================ */
void computeRateCommands(int &roll_cmd, int &pitch_cmd, int &yaw_cmd) {

  // dt for the angle integrators — use wall time so it's independent
  // of how often this function is called (outer loop is throttled to 100 Hz)
  unsigned long now = micros();
  float dt = (lastAngleLoopTime == 0) ? 0.0f : (now - lastAngleLoopTime) / 1e6f;
  lastAngleLoopTime = now;
  if (dt > 0.05f) dt = 0.05f;  // clamp: ignore stale dt on first call or after pause

  if (useAngleControl) {
    // ------------------------------------------------------------------
    // OUTER LOOP: inline PI  →  Euler rate command (deg/s)
    // P term: immediate proportional response to angle error
    // I term: eliminates steady-state offset (e.g. CG bias, trim error)
    // ------------------------------------------------------------------
    float rollError  = target_roll_deg  - roll_deg;
    float pitchError = target_pitch_deg - pitch_deg;

    // Accumulate integrals with anti-windup clamp
    if (dt > 0.0f) {
      angleInt_roll  = constrain(angleInt_roll  + rollError  * dt, -ANGLE_INT_LIMIT, ANGLE_INT_LIMIT);
      angleInt_pitch = constrain(angleInt_pitch + pitchError * dt, -ANGLE_INT_LIMIT, ANGLE_INT_LIMIT);
    }

    float euler_roll_rate  = constrain(angleKp_roll  * rollError  + angleKi_roll  * angleInt_roll,
                                       -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);
    float euler_pitch_rate = constrain(angleKp_pitch * pitchError + angleKi_pitch * angleInt_pitch,
                                       -MAX_RATE_FROM_ANGLE, MAX_RATE_FROM_ANGLE);

    // Yaw: shortest-path error, PIDController unchanged
    float yawError = normalizeAngle180(target_yaw_deg - yaw_deg);
    target_yaw_rate = constrain(
      yawAnglePID.update(yawError, 0.0f),
      -MAX_YAW_RATE, MAX_YAW_RATE
    );

    // ------------------------------------------------------------------
    // Transform Euler rates → body-frame rate setpoints
    // ------------------------------------------------------------------
    float phi   = roll_deg  * M_PI / 180.0f;
    float theta = pitch_deg * M_PI / 180.0f;

    target_roll_rate  = euler_roll_rate  - sinf(theta) * target_yaw_rate;
    target_pitch_rate = cosf(phi) * euler_pitch_rate + sinf(phi) * cosf(theta) * target_yaw_rate;
  }

  // ------------------------------------------------------------------
  // INNER LOOP: body-frame rate error → servo µs deflection
  // ------------------------------------------------------------------
  float roll_output  = rollRatePID .update(target_roll_rate,  measured_roll_rate);
  float pitch_output = pitchRatePID.update(target_pitch_rate, measured_pitch_rate);
  float yaw_output   = yawRatePID  .update(target_yaw_rate,   measured_yaw_rate);

  roll_cmd  = (int)constrain(roll_output,  -500, 500);
  pitch_cmd = (int)constrain(pitch_output, -500, 500);
  yaw_cmd   = (int)constrain(yaw_output,   -500, 500);
}

/* ===============================
    SERIAL COMMAND PARSER
================================ */
// ADDED: Extended serial protocol with two input formats:
//
//  1. FLIGHT COMMAND (space-separated integers, same as before):
//       <throttle> <roll> <pitch> <yaw>
//     In rate mode  → roll/pitch/yaw are deg/s targets
//     In angle mode → roll/pitch/yaw are degree targets
//     Example:  1300 0 0 0
//
//  2. MODE SWITCH (keyword):
//       MODE:ANGLE    — switch to angle control
//       MODE:RATE     — switch to raw rate control
//       MODE:QUERY    — print current mode
//
//  3. LIVE PID TUNING (keyword:float):
//       RP:1.5        — Roll  rate  PID Kp
//       RI:0.01       — Roll  rate  PID Ki
//       RD:0.05       — Roll  rate  PID Kd
//       PP:1.5        — Pitch rate  PID Kp  (etc.)
//       PI:0.01
//       PD:0.05
//       YP:1.5        — Yaw   rate  PID Kp
//       YI:0.01
//       YD:0.05
//       RAP:5.0       — Roll  angle PID Kp
//       PAP:5.0       — Pitch angle PID Kp
//       YAP:3.0       — Yaw   angle PID Kp
//       YAI:0.01
//       YAD:0.02
//
// All tuning changes print a confirmation line.

void handleSerial(bool &didSerial, bool &usbConnected) {
  unsigned long currentTime = millis();
  didSerial = false;

  if (!Serial.available()) return;

  didSerial = true;
  lastSerialActivity = currentTime;
  usbConnected = true;

  // Read the full line
  String line = Serial.readStringUntil('\n');
  line.trim();

  // ---- Mode switch commands ----
  if (line.startsWith("MODE:")) {
    String mode = line.substring(5);
    mode.trim();
    if (mode == "ANGLE") {
      useAngleControl = true;
      target_yaw_deg   = yaw_deg;
      target_roll_deg  = roll_deg;
      target_pitch_deg = pitch_deg;
      angleInt_roll  = 0.0f;   // reset angle integrators
      angleInt_pitch = 0.0f;
      lastAngleLoopTime = 0;
      yawAnglePID.reset();
      Serial.println(">> Switched to ANGLE CONTROL mode.");
    } else if (mode == "RATE") {
      useAngleControl = false;
      target_roll_rate  = 0.0f;
      target_pitch_rate = 0.0f;
      target_yaw_rate   = 0.0f;
      Serial.println(">> Switched to RATE CONTROL mode.");
    } else if (mode == "QUERY") {
      Serial.print(">> Current mode: ");
      Serial.println(useAngleControl ? "ANGLE" : "RATE");
    }
    return;
  }

  // ---- Live PID tuning ----
  float v = 0;
  if      (parseKV(line,"RP",  v)) { rollRatePID .Kp = v; Serial.print(">> Roll  rate  Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"RI",  v)) { rollRatePID .Ki = v; Serial.print(">> Roll  rate  Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"RD",  v)) { rollRatePID .Kd = v; Serial.print(">> Roll  rate  Kd = "); Serial.println(v); return; }
  else if (parseKV(line,"PP",  v)) { pitchRatePID.Kp = v; Serial.print(">> Pitch rate  Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"PI",  v)) { pitchRatePID.Ki = v; Serial.print(">> Pitch rate  Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"PD",  v)) { pitchRatePID.Kd = v; Serial.print(">> Pitch rate  Kd = "); Serial.println(v); return; }
  else if (parseKV(line,"YP",  v)) { yawRatePID  .Kp = v; Serial.print(">> Yaw   rate  Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"YI",  v)) { yawRatePID  .Ki = v; Serial.print(">> Yaw   rate  Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"YD",  v)) { yawRatePID  .Kd = v; Serial.print(">> Yaw   rate  Kd = "); Serial.println(v); return; }
  else if (parseKV(line,"RAP", v)) { angleKp_roll  = v; Serial.print(">> Roll  angle Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"RAI", v)) { angleKi_roll  = v; Serial.print(">> Roll  angle Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"PAP", v)) { angleKp_pitch = v; Serial.print(">> Pitch angle Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"PAI", v)) { angleKi_pitch = v; Serial.print(">> Pitch angle Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"YAP", v)) { yawAnglePID  .Kp = v; Serial.print(">> Yaw   angle Kp = "); Serial.println(v); return; }
  else if (parseKV(line,"YAI", v)) { yawAnglePID  .Ki = v; Serial.print(">> Yaw   angle Ki = "); Serial.println(v); return; }
  else if (parseKV(line,"YAD", v)) { yawAnglePID  .Kd = v; Serial.print(">> Yaw   angle Kd = "); Serial.println(v); return; }

  // ---- Standard flight command: "throttle roll pitch yaw" ----
  int throttle = line.toInt();                          // first integer
  int spIdx    = line.indexOf(' ');
  int tgt1 = 0, tgt2 = 0, tgt3 = 0;
  if (spIdx >= 0) {
    String rest = line.substring(spIdx + 1);
    tgt1 = rest.toInt();
    int s2 = rest.indexOf(' ');
    if (s2 >= 0) {
      rest = rest.substring(s2 + 1);
      tgt2 = rest.toInt();
      int s3 = rest.indexOf(' ');
      if (s3 >= 0) {
        tgt3 = rest.substring(s3 + 1).toInt();
      }
    }
  }

  if (useAngleControl) {
    target_roll_deg  = (float)tgt1;
    target_pitch_deg = (float)tgt2;
    target_yaw_deg   = (float)tgt3;
  } else {
    target_roll_rate  = (float)tgt1;
    target_pitch_rate = (float)tgt2;
    target_yaw_rate   = (float)tgt3;
  }

  currentThrottle = constrain(throttle, minPulse, maxPulse);
}

/* ===============================
    SETUP
================================ */
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("=== ESP32-S3 Thrust-Vectoring Monocopter ===");

  Wire.setPins(I2C_SDA, I2C_SCL);   Wire.begin();
  Wire1.setPins(I2C1_SDA, I2C1_SCL); Wire1.begin();

  const uint8_t LSM6DSOX_CUSTOM_ADDR = 0x6B;
  if (!imu.begin_I2C(LSM6DSOX_CUSTOM_ADDR)) {
    Serial.println("Failed to find LSM6DSOX at 0x6B!");
    while (1) delay(10);
  }
  Serial.println("LSM6DSOX OK at 0x6B");

  const uint8_t LIS3MDL_CUSTOM_ADDR = 0x1C;
  if (!mag.begin_I2C(LIS3MDL_CUSTOM_ADDR, &Wire1)) {
    Serial.println("LIS3MDL Failed.");
  } else {
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    Serial.println("LIS3MDL OK.");
  }

  for (int i = 0; i < numServos; i++) {
    ledcSetup(servoChannels[i], freq, resolution);
    ledcAttachPin(servoPins[i], servoChannels[i]);
    ledcWrite(servoChannels[i], pulseToDuty(servoZeroPulse + servoOffsets[i]));
  }
  ledcSetup(escChannel, freq, resolution);
  ledcAttachPin(escPin, escChannel);
  ledcWrite(escChannel, pulseToDuty(minPulse));
  delay(5000);

  // FIX Bug 4: Set integralLimit for angle PIDs to match their output range (deg/s)
  yawAnglePID.integralLimit = MAX_YAW_RATE;  // 90 deg/s

  Serial.println();
  Serial.println("=== Control mode: ANGLE (cascade PID) ===");
  Serial.println("Flight cmd  : <throttle> <roll_deg> <pitch_deg> <yaw_deg>");
  Serial.println("Mode switch : MODE:ANGLE | MODE:RATE | MODE:QUERY");
  Serial.println("Live tuning : RP:Kp  RI:Ki  RD:Kd  (roll rate)");
  Serial.println("              PP:Kp  PI:Ki  PD:Kd  (pitch rate)");
  Serial.println("              YP:Kp  YI:Ki  YD:Kd  (yaw rate)");
  Serial.println("              RAP:Kp RAI:Ki RAD:Kd (roll angle)");
  Serial.println("              PAP:Kp PAI:Ki PAD:Kd (pitch angle)");
  Serial.println("              YAP:Kp YAI:Ki YAD:Kd (yaw angle)");
  Serial.println();
}

/* ===============================
    MAIN LOOP
================================ */
void loop() {
  static unsigned long lastLoopTime = 0;
  unsigned long currentTimeMicros = micros();

  // Enforce a strict 400Hz control loop (2500 microseconds)
  if (currentTimeMicros - lastLoopTime < 2500) return;
  lastLoopTime = currentTimeMicros;
  
  unsigned long currentTime = millis();

  updateOrientation();

  // USB disconnect watchdog
  bool usbConnected = true;
  if (lastSerialActivity > 0 && (currentTime - lastSerialActivity) > USB_HEARTBEAT_TIMEOUT) {
    usbConnected = false;
  }

  // Parse serial input
  bool didSerial = false;
  handleSerial(didSerial, usbConnected);

  // FIX Design Issue: Throttle outer loop to 100 Hz (run every 4th inner-loop cycle at 400 Hz)
  // This gives the inner rate loop time to settle before outer angle loop re-evaluates.
  static int outerLoopCounter = 0;
  outerLoopCounter++;
  bool runOuterLoop = (useAngleControl && (outerLoopCounter % 4 == 0));

  // Safety: cut throttle if USB lost
  int throttleToApply = usbConnected ? currentThrottle : minPulse;
  ledcWrite(escChannel, pulseToDuty(throttleToApply));

  // Compute commands through cascade (outer loop throttled if angle mode)
  int roll, pitch, yaw;
  if (useAngleControl && !runOuterLoop) {
    // Inner rate loop only; use previous target rates
    roll = rollRatePID.update(target_roll_rate, measured_roll_rate);
    pitch = pitchRatePID.update(target_pitch_rate, measured_pitch_rate);
    yaw = yawRatePID.update(target_yaw_rate, measured_yaw_rate);
  } else {
    // Full cascade (both outer and inner loops)
    computeRateCommands(roll, pitch, yaw);
  }
  roll = constrain(roll, -500, 500);
  pitch = constrain(pitch, -500, 500);
  yaw = constrain(yaw, -500, 500);

  // Mix onto servos
  float rawMix[numServos];
  for (int i = 0; i < numServos; i++)
    rawMix[i] = roll*mixCoeffs[i][0] + pitch*mixCoeffs[i][1] + yaw*mixCoeffs[i][2];

  // Symmetric amplitude scaling
  int symmetricAmp = INT_MAX;
  for (int i = 0; i < numServos; i++) {
    int centerPulse = servoZeroPulse + servoOffsets[i];
    int allowed = min(maxPulse - centerPulse, centerPulse - minPulse);
    if (allowed < symmetricAmp) symmetricAmp = allowed;
  }

  float maxReq = 0.0f;
  for (int i = 0; i < numServos; i++)
    if (fabs(rawMix[i]) > maxReq) maxReq = fabs(rawMix[i]);

  float scale = (maxReq > 1e-6f) ? min(1.0f, (float)symmetricAmp / maxReq) : 0.0f;

  // Debug output on every received command
  if (didSerial) {
    Serial.print("T:"); Serial.print(throttleToApply);
    Serial.print(" | ATT R:"); Serial.print(roll_deg,  1);
    Serial.print("° P:"); Serial.print(pitch_deg, 1);
    Serial.print("° Y:"); Serial.print(yaw_deg,   1);
    Serial.print("° | RATES R:"); Serial.print(measured_roll_rate,  1);
    Serial.print(" P:");          Serial.print(measured_pitch_rate, 1);
    Serial.print(" Y:");          Serial.print(measured_yaw_rate,   1);
    Serial.print("°/s");

    if (useAngleControl) {
      Serial.print(" | TGT-ANG R:"); Serial.print(target_roll_deg,  1);
      Serial.print(" P:");           Serial.print(target_pitch_deg, 1);
      Serial.print(" Y:");           Serial.print(target_yaw_deg,   1);
      Serial.print("°");
      float yawErr = normalizeAngle180(target_yaw_deg - yaw_deg);
      Serial.print(" Y[e]=");             Serial.print(yawErr, 2);
    }

    Serial.print(" | TGT-RATE R:"); Serial.print(target_roll_rate,  1);
    Serial.print(" P:");            Serial.print(target_pitch_rate, 1);
    Serial.print(" Y:");            Serial.print(target_yaw_rate,   1);
    Serial.print("°/s");

    Serial.print(" | RATE-PID R[e]="); Serial.print(rollRatePID.lastError,  2);
    Serial.print(" Rout=");            Serial.print(roll);
    Serial.print(" P[e]=");            Serial.print(pitchRatePID.lastError, 2);
    Serial.print(" Pout=");            Serial.print(pitch);
    Serial.print(" Y[e]=");            Serial.print(yawRatePID.lastError,   2);
    Serial.print(" Yout=");            Serial.print(yaw);

    Serial.print(" | SERVOS: ");
  }

  for (int i = 0; i < numServos; i++) {
    int centerPulse = servoZeroPulse + servoOffsets[i];
    int delta = (int)round(rawMix[i] * scale);
    delta = constrain(delta, -200, 200);
    int finalPulse = constrain(centerPulse + delta, minPulse, maxPulse);
    ledcWrite(servoChannels[i], pulseToDuty(finalPulse));
    if (didSerial) {
      Serial.print(finalPulse);
      if (i < numServos - 1) Serial.print(", ");
    }
  }
  if (didSerial) Serial.println();

  // Periodic status (no serial activity)
  static unsigned long lastPrint = 0;
  if (!didSerial && millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.print(useAngleControl ? "[ANGLE] " : "[RATE]  ");
    Serial.print("R:"); Serial.print(roll_deg,  1);
    Serial.print("° P:"); Serial.print(pitch_deg, 1);
    Serial.print("° Y:"); Serial.print(yaw_deg,   1);
    Serial.print("°  |  tgt R:"); Serial.print(useAngleControl ? target_roll_deg  : target_roll_rate,  1);
    Serial.print(" P:");          Serial.print(useAngleControl ? target_pitch_deg : target_pitch_rate, 1);
    Serial.print(" Y:");          Serial.print(useAngleControl ? target_yaw_deg   : target_yaw_rate,   1);
    Serial.println(useAngleControl ? "°" : "°/s");
  }
}

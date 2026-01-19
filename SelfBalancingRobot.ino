/*
  Self-Balancing Robot (ESP32 + MPU6050 + 2x Stepper Drivers)

  ===== README / Wiring / Tuning Guide =====
  Hardware:
  - ESP32 DevKit (Arduino core)
  - MPU6050 IMU (I2C)
  - 2x NEMA17 stepper motors + A4988/DRV8825 style drivers

  Wiring:
  - I2C: SDA -> PIN_I2C_SDA, SCL -> PIN_I2C_SCL
  - MPU6050 VCC -> 3.3V, GND -> GND
  - Stepper drivers (each):
      STEP -> PIN_Mx_STEP
      DIR  -> PIN_Mx_DIR
      EN   -> PIN_Mx_EN (active LOW by default, shared enable in the diagram)
      VMOT -> motor supply (per driver datasheet)
      GND  -> common ground with ESP32
  - Optional battery ADC: VBAT -> PIN_BATT_ADC (via resistor divider)
  - Optional indicators: buzzer -> PIN_BUZZER, LEDs -> PIN_LED
  - Optional servo: signal -> PIN_SERVO (power from external 5V rail)

  Driver Current & Microstepping:
  - Set current limit on A4988/DRV8825 using Vref per datasheet.
  - Start with 1/16 microstepping to reduce noise. Increase if needed.

  Loop Timing:
  - Control loop at 250 Hz (4 ms). IMU sampling synchronized in loop.

  Tuning Steps:
  1) Start with recommended PID values and keep the robot supported.
  2) Increase Kp until it starts to balance (but oscillates).
  3) Increase Kd to reduce oscillation/overshoot (noise filtered).
  4) Increase Ki slowly to correct steady-state tilt.
  5) Adjust complementary filter alpha: higher alpha = more gyro weight.

  Safety:
  - Motors disable if |tilt| > 30 degrees or IMU fails.
  - Waits for upright (|tilt| < 10 deg) before enabling.

  Serial Tuning:
  - Commands (newline terminated):
      kp <value>
      ki <value>
      kd <value>
      alpha <0..1>
      trim <deg>
      telemetry on/off
      status
  - Telemetry prints: angle, rate, pid terms, motor cmd.
*/

#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "pid.h"
#include "motor.h"

// ===== Pin Mapping =====
static constexpr int PIN_I2C_SDA = 21;
static constexpr int PIN_I2C_SCL = 22;

static constexpr int PIN_M1_STEP = 26;
static constexpr int PIN_M1_DIR  = 27;
static constexpr int PIN_M1_EN   = 25;

static constexpr int PIN_M2_STEP = 14;
static constexpr int PIN_M2_DIR  = 12;
static constexpr int PIN_M2_EN   = 25; // shared enable per reference diagram

static constexpr int PIN_BATT_ADC = 34; // optional
static constexpr int PIN_BUZZER = 33;   // optional per diagram
static constexpr int PIN_LED = 32;      // optional per diagram (2 LEDs in parallel)
static constexpr int PIN_SERVO = 13;    // optional per diagram

// ===== Configurable Parameters =====
static constexpr float CTRL_HZ = 250.0f;
static constexpr float DT_SEC = 1.0f / CTRL_HZ;

static constexpr int STEPS_PER_REV = 200;
static constexpr int MICROSTEPS = 16;
static constexpr float WHEEL_RADIUS_M = 0.035f; // 70 mm diameter

static constexpr float MAX_SPEED_STEPS_S = 3200.0f; // steps/s
static constexpr float MAX_ACCEL_STEPS_S2 = 12000.0f; // acceleration limit

static constexpr float TILT_DISABLE_DEG = 30.0f;
static constexpr float TILT_ENABLE_DEG = 10.0f;

static constexpr bool EN_ACTIVE_LOW = true;

// PID starting values (tune for your build)
static PIDGains pidGains = {
  .kp = 18.0f,
  .ki = 0.8f,
  .kd = 0.6f,
  .integratorLimit = 200.0f,
  .derivativeFilterAlpha = 0.2f
};

static float complementaryAlpha = 0.98f; // gyro weight
static float angleTrimDeg = 0.0f;

// ===== Globals =====
MPU6050IMU imu;
BalancedPID pid(pidGains);
StepperMotor motorLeft;
StepperMotor motorRight;

static bool telemetryEnabled = false;
static uint32_t lastTelemetryMs = 0;
static bool motorsEnabled = false;
static uint32_t lastEnableMs = 0;

// Utility: clamp
static float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static void disableMotors() {
  motorLeft.enable(false);
  motorRight.enable(false);
  motorsEnabled = false;
}

static void enableMotors() {
  motorLeft.enable(true);
  motorRight.enable(true);
  motorsEnabled = true;
  lastEnableMs = millis();
  pid.reset();
}

static void printStatus(float angleDeg, float rateDps, const PIDTerms &terms, float cmd) {
  Serial.printf("angle=%.2f rate=%.2f p=%.2f i=%.2f d=%.2f cmd=%.2f enabled=%d\n",
                angleDeg, rateDps, terms.p, terms.i, terms.d, cmd, motorsEnabled ? 1 : 0);
}

static void handleSerial() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  auto cmd = line;
  cmd.toLowerCase();

  if (cmd.startsWith("kp ")) {
    pidGains.kp = cmd.substring(3).toFloat();
    pid.setGains(pidGains);
  } else if (cmd.startsWith("ki ")) {
    pidGains.ki = cmd.substring(3).toFloat();
    pid.setGains(pidGains);
  } else if (cmd.startsWith("kd ")) {
    pidGains.kd = cmd.substring(3).toFloat();
    pid.setGains(pidGains);
  } else if (cmd.startsWith("alpha ")) {
    complementaryAlpha = clampf(cmd.substring(6).toFloat(), 0.0f, 1.0f);
  } else if (cmd.startsWith("trim ")) {
    angleTrimDeg = cmd.substring(5).toFloat();
  } else if (cmd == "telemetry on") {
    telemetryEnabled = true;
  } else if (cmd == "telemetry off") {
    telemetryEnabled = false;
  } else if (cmd == "status") {
    Serial.printf("kp=%.3f ki=%.3f kd=%.3f alpha=%.3f trim=%.2f\n",
                  pidGains.kp, pidGains.ki, pidGains.kd, complementaryAlpha, angleTrimDeg);
  } else {
    Serial.println("Unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  motorLeft.begin(PIN_M1_STEP, PIN_M1_DIR, PIN_M1_EN, 0, EN_ACTIVE_LOW);
  motorRight.begin(PIN_M2_STEP, PIN_M2_DIR, PIN_M2_EN, 1, EN_ACTIVE_LOW);

  disableMotors();

  if (!imu.begin(Wire)) {
    Serial.println("IMU init failed. Check wiring.");
  } else {
    imu.calibrate(200);
  }

  Serial.println("Self-Balancing Robot Ready");
}

void loop() {
  static uint32_t lastLoopUs = micros();
  static float angleDeg = 0.0f;

  handleSerial();

  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastLoopUs;
  if (elapsedUs < (1000000.0f / CTRL_HZ)) {
    return;
  }
  lastLoopUs = nowUs;

  IMUReading imuData{};
  if (!imu.read(imuData)) {
    disableMotors();
    return;
  }

  float accAngleDeg = atan2f(imuData.accY, imuData.accZ) * 180.0f / PI;
  float gyroRateDps = imuData.gyroX;

  angleDeg = complementaryAlpha * (angleDeg + gyroRateDps * DT_SEC) +
             (1.0f - complementaryAlpha) * accAngleDeg;
  angleDeg += angleTrimDeg;

  bool tiltUnsafe = fabsf(angleDeg) > TILT_DISABLE_DEG;
  if (tiltUnsafe) {
    disableMotors();
  } else if (!motorsEnabled && fabsf(angleDeg) < TILT_ENABLE_DEG) {
    enableMotors();
  }

  float cmd = 0.0f;
  PIDTerms terms{};
  if (motorsEnabled) {
    cmd = pid.update(0.0f, angleDeg, DT_SEC, terms);

    // Soft-start: ramp output limit after enabling
    float ramp = clampf((millis() - lastEnableMs) / 1000.0f, 0.0f, 1.0f);
    float maxCmd = MAX_SPEED_STEPS_S * ramp;
    cmd = clampf(cmd, -maxCmd, maxCmd);
  }

  motorLeft.setTargetSpeed(cmd, MAX_ACCEL_STEPS_S2, DT_SEC);
  motorRight.setTargetSpeed(cmd, MAX_ACCEL_STEPS_S2, DT_SEC);

  if (telemetryEnabled && millis() - lastTelemetryMs >= 20) {
    lastTelemetryMs = millis();
    terms = pid.lastTerms();
    printStatus(angleDeg, gyroRateDps, terms, cmd);
  }
}

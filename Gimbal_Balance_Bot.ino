#include <SimpleFOC.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ============================================================
// ESP32 WROOM
// TWO C2208-100T MOTORS + TWO MT6701 SSI + TWO DRV8313 + BMI160
// TWO-MOTOR BALANCE TEST
//
// MT6701 #1:
//   DO  -> GPIO 19
//   CLK -> GPIO 18
//   CS  -> GPIO 32
//
// DRV8313 #1:
//   IN1 -> GPIO 25
//   IN2 -> GPIO 26
//   IN3 -> GPIO 27
//   EN  -> GPIO 14
//
// MT6701 #2:
//   DO  -> GPIO 23 (not GPIO34; 34 is reserved for the pot)
//   CLK -> GPIO 33
//   CS  -> GPIO 13
//
// DRV8313 #2:
//   IN1 -> GPIO 5
//   IN2 -> GPIO 16
//   IN3 -> GPIO 17
//   EN  -> GPIO 4
//
// BMI160 I2C:
//   SDA -> GPIO 21
//   SCL -> GPIO 22
//   3.3V -> ESP32 3.3V
//   GND  -> GND
//   CS   -> 3.3V
//   SA0  -> GND       (address 0x68)
//   VIN  -> leave disconnected
//
// FAST BALANCE + HM-10 SPEED COMMAND + LEARNED ZERO-ANGLE.
// No live telemetry while balancing.
// Based on supplied PI_Authority(1) sketch; all original motor and speed PI gains retained.
// 10k optional manual trim pot: 3.3V -- pot -- GND, wiper -> GPIO34.
// NO FIXED 10-DEGREE OFFSET. Loads SAVED electrical motor alignment from
// NVS at boot; NO wheel rotation for motor alignment. Calibrate the IMU
// while still, hold near the physical balance point and press GPIO15. After a 1 s stability check, the current
// pitch becomes the initial balance reference. Wheel-speed PI automatically
// refines that reference during quiet, zero-command balancing so changes
// to the batteries or other loads do not require adjusting a fixed offset.
// Ground contact is NOT measurable with existing sensors: only press the
// GPIO15 ARM button AFTER placing on ground. 'f' = normal zero-RPM stop;
// 'x' = coast/disarm.
// ============================================================


// ------------------------------------------------------------
// PINS
// ------------------------------------------------------------

// MT6701 encoders
const int SSI1_CLK = 18;
const int SSI2_CLK = 33;

const int SSI1_CS = 32;
const int SSI1_DO = 19;

const int SSI2_CS = 13;
const int SSI2_DO = 23; // Encoder 2 data; GPIO34 is the pot
// DRV8313 motor #1
const int M1_PWM_A = 25;
const int M1_PWM_B = 26;
const int M1_PWM_C = 27;
const int M1_DRIVER_EN = 14;

// DRV8313 motor #2
const int M2_PWM_A = 5;
const int M2_PWM_B = 16;
const int M2_PWM_C = 17;
const int M2_DRIVER_EN = 4;

const int BMI_SDA = 21;
const int BMI_SCL = 22;

// Balance / drive command potentiometer                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
// GPIO34 is ADC1, input-only. NEVER use it for motor 2 encoder data.
const int BALANCE_POT_PIN = 34;

// Physical LEARN + ARM pushbutton.
// GPIO15 is otherwise unused on this ESP32 wiring. Use a normally-open
// pushbutton from GPIO15 to GND; INPUT_PULLUP keeps it HIGH when released.
// Do not hold the button while resetting/powering the ESP32 because GPIO15
// is a boot-strapping pin.
const int BALANCE_ARM_BUTTON_PIN = 15;
const uint32_t ARM_BUTTON_DEBOUNCE_MS = 35;

// GPIO2: onboard LED on many ESP32-WROOM DevKit boards. If absent, connect
// an external LED: GPIO2 -> 330-ohm resistor -> LED anode; LED cathode -> GND.
// Do not connect the LED to 3.3V; this sketch expects active-HIGH indication.
const int STATUS_LED_PIN = 2;
enum class StatusCode : uint8_t {
  Starting, Ready, MissingCalibration, Foc1Failed, Foc2Failed,
  ImuFailed, CapturePending, CaptureTimeout, Armed, Fallen
};
StatusCode statusCode = StatusCode::Starting;
uint32_t statusChangedMs = 0;
bool statusLedLast = false;
uint32_t statusLedUpdateMs = 0;

// The press starts a bounded capture window. No pre-press 1-second test.
bool armRequested = false;
uint32_t armRequestedMs = 0;
const uint32_t ARM_REQUEST_TIMEOUT_MS = 6000;


static_assert(BALANCE_POT_PIN != SSI1_DO && BALANCE_POT_PIN != SSI2_DO,
              "Pot GPIO34 must not be used for either encoder data pin");

HardwareSerial BTSerial(2);

const int BT_RX_PIN = 35;

// HM-10 / Simple BLE Joystick control
//   a = forward (latched)
//   c = reverse (latched)
//   b = steer right while held/repeating
//   d = steer left while held/repeating
//   e = speed up
//   g = speed down
//   f = ALL STOP
//   h = unused (physical GPIO15 button performs LEARN + ARM)
//   x = COAST/DISARM (support robot; it will fall if released)
float commandedRPM = 0.0f;
float commandedSteerRPM = 0.0f;
bool bluetoothDriveEnabled = false;

float btDriveRPM = 40.0f;
const float BT_MIN_RPM = 10.0f;
const float BT_MAX_RPM = 60.0f;
const float BT_SPEED_STEP = 5.0f;
const float BT_STEER_RPM = 10.0f;
const float BT_STEER_DIRECTION = +1.0f;

const uint32_t STEER_TIMEOUT_MS = 150;
uint32_t lastSteerMillis = 0;

// Smooth Bluetooth speed changes so start/stop do not suddenly command
// a large lean angle.  At 60 RPM/s, 25 -> 0 RPM takes about 0.42 s.
const float BT_RPM_SLEW_PER_SEC = 100.0f;

// The app repeats characters while a button is held.  Debounce e/g so
// holding a speed button changes speed in useful 5 RPM steps.
const uint32_t SPEED_BUTTON_REPEAT_MS = 250;
uint32_t lastSpeedButtonMillis = 0;

// ============================================================
// BMI160 REGISTERS
// ============================================================

const uint8_t BMI160_ADDR = 0x68;

const uint8_t REG_CHIP_ID   = 0x00;
const uint8_t REG_GYR_DATA  = 0x0C;
const uint8_t REG_ACC_CONF  = 0x40;
const uint8_t REG_ACC_RANGE = 0x41;
const uint8_t REG_GYR_CONF  = 0x42;
const uint8_t REG_GYR_RANGE = 0x43;
const uint8_t REG_CMD       = 0x7E;


// ============================================================
// MT6701 BIT-BANGED SSI
// ============================================================

float readMT6701AngleOnPins(int clkPin, int csPin, int doPin) {
  uint32_t data = 0;

  digitalWrite(csPin, LOW);

  for (int i = 0; i < 24; i++) {
    digitalWrite(clkPin, LOW);
    delayMicroseconds(1);

    digitalWrite(clkPin, HIGH);
    delayMicroseconds(1);

    digitalWrite(clkPin, LOW);
    delayMicroseconds(1);

    data <<= 1;
    if (digitalRead(doPin))
      data |= 1;
  }

  digitalWrite(clkPin, HIGH);
  digitalWrite(csPin, HIGH);

  uint16_t raw = (data >> 10) & 0x3FFF;

  return raw * (2.0f * PI / 16384.0f);
}

float readMT6701Angle1() {
  return readMT6701AngleOnPins(SSI1_CLK, SSI1_CS, SSI1_DO);
}

float readMT6701Angle2() {
  return readMT6701AngleOnPins(SSI2_CLK, SSI2_CS, SSI2_DO);
}


// ============================================================
// SIMPLEFOC
// ============================================================

GenericSensor sensor1 = GenericSensor(readMT6701Angle1);
GenericSensor sensor2 = GenericSensor(readMT6701Angle2);

// C2208: 7 pole pairs
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);

BLDCDriver3PWM driver1 =
  BLDCDriver3PWM(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_DRIVER_EN);

BLDCDriver3PWM driver2 =
  BLDCDriver3PWM(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_DRIVER_EN);


// ============================================================
// BALANCE SETTINGS
// ============================================================

// Two Li-ion cells in series are about 8.4 V full / 7.4 V nominal.
// 8.0 V is a good nominal value for this temporary test.
// If using the old 12 V bench supply, change this back to 12.0f.
const float POWER_SUPPLY_VOLTAGE = 8.0f;

// Direct voltage/torque balance control.
// In this mode the balance controller directly commands q-axis motor voltage,
// while the MT6701 encoders are still used by loopFOC() for commutation.
const float MAX_VOLTAGE = 5.0f;

// ------------------------------------------------------------
// MID-AIR BENCH TEST MODE
//
// true  = GPIO15 immediately captures the CURRENT pitch and enables
//         angle+gyro control. No 1-second stillness gate, wheel-speed PI,
//         auto-trim, or wheel-sync. Intended only to verify that both
//         wheels react in the proper direction while held in the air.
// false = normal ground-balancing behavior.
// ------------------------------------------------------------
const bool BENCH_MIDAIR_TEST = false;
const float BENCH_MAX_VOLTAGE = 3.0f;

// Inner stabilization gains for voltage/torque control.
// KP units: volts per degree of pitch error.
// KD units: volts per degree/second of body rotation.
// Inner loop test: closer to the gain ratio used by the working
// SimpleFOC BLDC balancer.  More angle authority, much less gyro chatter.
const float KP_ANGLE = 0.45f;
const float KD_GYRO  = 0.0025f;

// ------------------------------------------------------------
// LEARNED BALANCE REFERENCE + OPTIONAL MANUAL POT TRIM
//
// No hard-coded balance bias. At ARM the current held-ground pitch is
// captured, minus the pot's *present* trim so the first motor command
// is approximately zero even if the pot isn't centered.
// Later changes to the knob adjust +/-3 degrees relative to that setting.
// ------------------------------------------------------------
// Keep the same manual pot authority as the supplied PI_Authority sketch.
// At ARM, capture current held angle and subtract the present pot setting,
// so twisting the knob afterward adjusts pitch without a startup jerk.
const float POT_BALANCE_RANGE_DEG = 8.0f;
const float POT_DIRECTION = +1.0f;
const int   POT_CENTER_ADC = 2048;
const int   POT_CENTER_DEADBAND = 20;
const uint32_t POT_INTERVAL_US = 10000;        // 100 Hz outer loop
const float POT_FILTER_ALPHA = 0.15f;

// ------------------------------------------------------------
// SIMPLEFOC-STYLE OUTER VELOCITY LOOP
//
// Keep this simple:
//     measured wheel RPM - filtered requested RPM
//              |
//              v
//        velocity P controller
//              |
//              v
//        requested body lean
//              |
//              v
//       800 Hz balance loop
//
// STOP is NOT a separate mode.  F simply makes commandedRPM = 0 and
// this same velocity loop supplies the opposite lean needed to stop.
//
// The present wheel signs already worked for forward/reverse motion.
// If a hand test ever proves the complete measured-speed sign is wrong,
// change this one constant from +1 to -1.
const float SPEED_FEEDBACK_SIGN = +1.0f;

// About the same practical gain that worked well earlier on this robot.
const float SPEED_KP_DEG_PER_RPM = 0.060f;

// Small integral term:
// - builds extra lean if the bot stays below commanded speed
// - helps push through small barriers
// - removes steady-state forward/reverse speed mismatch
// It is tightly limited to avoid the runaway/windup we saw earlier.
const float SPEED_KI_DEG_PER_RPM_SEC = 0.025f; //0.025
const float MAX_SPEED_I_DEG = 3.0f;

// Total velocity-loop lean ceiling.  Raising this does NOT make normal
// cruising lean harder unless there is a large/persistent speed error.
const float MAX_SPEED_LEAN_DEG = 8.0f;

// The velocity I term is NOT bled away at rest: its sustained correction
// supplies the evidence that the learned neutral angle is imperfect.
// When near zero commanded speed, low wheel speed and stable pitch persist,
// slowly TRANSFER existing velocity I into the learned neutral angle.
// Adding the transfer to neutral AND subtracting it from speed I avoids
// suddenly changing the total target angle. Do not learn during driving.
const float AUTO_MAX_TRIM_DEG = 10.0f;    // relative to angle captured at ARM
const float AUTO_TRANSFER_TAU_SEC = 2.0f;
const float AUTO_MAX_RATE_DEG_SEC = 0.40f;
const uint32_t AUTO_QUIET_MS = 1500;      // no learning on brief pauses
const float AUTO_MAX_WHEEL_RPM = 8.0f;
const float AUTO_MAX_GYRO_DPS = 4.0f;
const float AUTO_MAX_ANGLE_ERR_DEG = 4.0f;

// SimpleFOC balancer idea: filter throttle slowly and filter commanded
// body lean more quickly.  These are time constants, not fixed alphas.
const float THROTTLE_FILTER_TAU_SEC = 0.50f;
const float LEAN_FILTER_TAU_SEC     = 0.07f;

const float WHEEL_RPM_FILTER_ALPHA = 0.15f;

// Left/right wheel-speed synchronization.
// This correction is symmetric: it reduces voltage to the faster wheel
// and adds the same amount to the slower wheel, so average balance torque
// remains nearly unchanged.
const float WHEEL_SYNC_KP_V_PER_RPM = 0.010f;
const float MAX_WHEEL_SYNC_V = 0.50f;
const float WHEEL_SYNC_ENABLE_RPM = 5.0f;

// IMPORTANT:
// If the motor makes a small lean WORSE instead of pushing back toward
// upright, change +1.0f to -1.0f. Do not change the BMI160 signs.
const float MOTOR_SIGN = -1.0f;

// Motor #2 is mounted on the opposite side of the chassis.
// Start with -1 so both wheels drive the robot in the same direction.
// If the two wheels fight each other during the supported test,
// change only this value to +1.0f.
const float MOTOR2_DIRECTION = +1.0f;

// Disable both motors completely if the body falls this far.
const float FALL_ANGLE_DEG = 30.0f;

// FAST balance loop: BMI160 and balance controller at 800 Hz.
// 1 / 800 s = 1250 us.
const uint32_t IMU_INTERVAL_US = 1250;

// Preserve roughly the same complementary-filter time constant as
// the old 0.98/0.02 filter at 100 Hz (~0.5 second).
const float COMPLEMENTARY_TAU_SEC = 0.50f;


// ============================================================
// BALANCE STATE
// ============================================================

float gyroBiasX = 0.0f;
float uprightOffset = 0.0f;
float balanceAngle = 0.0f;
float gyroX = 0.0f;
float targetVoltage = 0.0f;
float potBalanceBiasDeg = 0.0f;  // optional pot trim only
float learnedNeutralDeg = 0.0f;   // angle captured at each ground ARM
float learnedAutoTrimDeg = 0.0f; // slowly adapted each balancing session
float targetPitchDeg = 0.0f;
float potFilteredRaw = 2048.0f;

float targetRPM = 0.0f;
float filteredTargetRPM = 0.0f;
float avgWheelRPM = 0.0f;
float filteredWheelRPM = 0.0f;
float speedErrorRPM = 0.0f;
float speedIntegralDeg = 0.0f;
float rawSpeedLeanDeg = 0.0f;
float speedLeanDeg = 0.0f;

// Wheel-sync state
float wheel1RPM = 0.0f;
float wheel2RPM = 0.0f;
float wheelSyncVoltage = 0.0f;

// Physical ground contact cannot be inferred reliably from the IMU alone.
// Explicit ARM command is deliberately required after placement.
// The holding angle may differ from the midair-calibrated zero because
// the robot's center of gravity and battery position determine its balance.
const float ARM_HOLD_TOLERANCE_DEG = 1.0f;
const float ARM_MAX_CAPTURE_ANGLE_DEG = 22.0f;
const float ARM_MAX_GYRO_DPS = 3.0f;
const uint32_t ARM_STABLE_MS = 1000;
const uint32_t ARM_IMU_FRESH_MS = 40;

bool fallen = false;
bool balanceArmed = false;

// ARM pushbutton debounce / edge detection.
bool armButtonRaw = HIGH;
bool armButtonStable = HIGH;
uint32_t armButtonChangedMs = 0;

uint32_t nearBalanceSinceMs = 0;
uint32_t lastGoodImuMs = 0;
float armHeldAngleDeg = 0.0f;
bool trackingHeldAngle = false;
uint32_t zeroCommandQuietSinceMs = 0;

uint32_t lastImuMicros = 0;
uint32_t lastPotMicros = 0;


// ============================================================
// SMALL HELPERS
// ============================================================

float wrap180(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void sendLine(const char *text) {
  // Only occasional USB messages at startup, arm/disarm, or after a fall.
  // No continuous live printing while balancing.
  Serial.println(text);
}

// Blink codes are visible without USB. Count flashes in one 2.4-second cycle:
//   1 = ready, awaiting button     2 = missing/invalid saved motor calibration
//   3 = motor 1 FOC failed         4 = motor 2 FOC failed
//   5 = BMI160 problem             6 = fallen
// Fast flashes = button accepted, collecting stable angle.
// Solid = balance armed. Three rapid flashes = capture timed out.
void setStatus(StatusCode s) {
  statusCode = s;
  statusChangedMs = millis();
}

void updateStatusLED() {
  uint32_t now = millis();
  if ((uint32_t)(now - statusLedUpdateMs) < 25) return;
  statusLedUpdateMs = now;
  bool on = false;
  if (statusCode == StatusCode::Armed) on = true;
  else if (statusCode == StatusCode::Starting ||
           statusCode == StatusCode::CapturePending) on = (now % 180) < 90;
  else if (statusCode == StatusCode::CaptureTimeout) {
    if ((uint32_t)(now - statusChangedMs) >= 2400) setStatus(StatusCode::Ready);
    else on = ((now - statusChangedMs) % 220) < 85 &&
              ((now - statusChangedMs) % 2400) < 660;
  } else {
    int flashes = 1;
    if (statusCode == StatusCode::MissingCalibration) flashes = 2;
    if (statusCode == StatusCode::Foc1Failed) flashes = 3;
    if (statusCode == StatusCode::Foc2Failed) flashes = 4;
    if (statusCode == StatusCode::ImuFailed) flashes = 5;
    if (statusCode == StatusCode::Fallen) flashes = 6;
    uint32_t t = now % 2400;
    on = (t < (uint32_t)flashes * 280) && ((t % 280) < 110);
  }
  if (on != statusLedLast) {
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
    statusLedLast = on;
  }
}

// Stop the *drive request* and clear every stored speed-loop state.
// Do not accidentally inherit an earlier Bluetooth forward command at ARM.
void clearDriveState() {
  commandedRPM = 0.0f;
  commandedSteerRPM = 0.0f;
  bluetoothDriveEnabled = false;
  targetRPM = 0.0f;
  filteredTargetRPM = 0.0f;
  avgWheelRPM = 0.0f;
  filteredWheelRPM = 0.0f;
  speedErrorRPM = 0.0f;
  speedIntegralDeg = 0.0f;
  rawSpeedLeanDeg = 0.0f;
  speedLeanDeg = 0.0f;
  wheelSyncVoltage = 0.0f;
  targetVoltage = 0.0f;
  targetPitchDeg = learnedNeutralDeg + learnedAutoTrimDeg + potBalanceBiasDeg;
  zeroCommandQuietSinceMs = 0;
}

// Intended as a deliberate, separate command. 'f' is NOT disarm:
// it commands 0 RPM and lets the speed controller brake normally.
void disarmBalance() {
  if (balanceArmed) {
    motor1.move(0.0f);
    motor2.move(0.0f);
  }
  motor1.disable();
  motor2.disable();
  balanceArmed = false;
  armRequested = false;
  setStatus(StatusCode::Ready);
  clearDriveState();
  nearBalanceSinceMs = 0;
  trackingHeldAngle = false;
  char line[110];
  snprintf(line, sizeof(line), "DISARMED: captured=%.2f deg, learned trim=%.2f deg", 
           learnedNeutralDeg, learnedAutoTrimDeg);
  sendLine(line);
  sendLine(BENCH_MIDAIR_TEST ? "Motors OFF. Hold mid-air and press GPIO15 for immediate recapture." : "Motors OFF. Ground, hold nearly still for 1 s, then press ARM button.");
}

bool tryArmBalance() {
  if (fallen || balanceArmed) return false;
  uint32_t nowMs = millis();

  // ----------------------------------------------------------
  // MID-AIR BENCH MODE
  // ----------------------------------------------------------
  // For a wiring/direction test we do NOT require the robot to be
  // motionless for one second. A button press captures the most recent
  // pitch immediately, provided the IMU data itself is fresh.
  if (BENCH_MIDAIR_TEST) {
    if (lastGoodImuMs == 0 ||
        (uint32_t)(nowMs - lastGoodImuMs) > ARM_IMU_FRESH_MS) {
      return false;
    }

    learnedNeutralDeg = balanceAngle - potBalanceBiasDeg;
    learnedAutoTrimDeg = 0.0f;
    clearDriveState();
    lastPotMicros = micros();
    targetPitchDeg = learnedNeutralDeg + potBalanceBiasDeg;

    char line[110];
    snprintf(line, sizeof(line),
             "BENCH ARM: captured current pitch %.2f deg", balanceAngle);
    sendLine(line);
    sendLine("MID-AIR TEST ACTIVE: tilt by hand; GPIO15 capture is immediate.");

    motor1.enable();
    motor2.enable();
    motor1.move(0.0f);
    motor2.move(0.0f);
    balanceArmed = true;
    armRequested = false;
    setStatus(StatusCode::Armed);
    return true;
  }

  // ----------------------------------------------------------
  // NORMAL GROUND ARM MODE
  // ----------------------------------------------------------
  // A still IMU cannot prove ground contact. The user must press the ARM
  // button only AFTER placing wheels on the ground and holding the body balanced.
  if (lastGoodImuMs == 0 ||
      (uint32_t)(nowMs - lastGoodImuMs) > ARM_IMU_FRESH_MS ||
      !trackingHeldAngle ||
      nearBalanceSinceMs == 0 ||
      (uint32_t)(nowMs - nearBalanceSinceMs) < ARM_STABLE_MS ||
      fabs(wrap180(balanceAngle - armHeldAngleDeg)) > ARM_HOLD_TOLERANCE_DEG ||
      fabs(balanceAngle) > ARM_MAX_CAPTURE_ANGLE_DEG ||
      fabs(gyroX) > ARM_MAX_GYRO_DPS) {
    return false;
  }

  learnedNeutralDeg = armHeldAngleDeg - potBalanceBiasDeg;
  learnedAutoTrimDeg = 0.0f;
  clearDriveState();
  lastPotMicros = micros();
  char line[110];
  snprintf(line, sizeof(line), "ARM: captured ground pitch %.2f deg; auto-trim active",
           armHeldAngleDeg);
  sendLine(line);
  sendLine("Activating balance. Release gently; keep a hand near robot.");
  motor1.enable();
  motor2.enable();
  motor1.move(0.0f);
  motor2.move(0.0f);
  balanceArmed = true;
  armRequested = false;
  setStatus(StatusCode::Armed);
  return true;
}


// ============================================================
// BMI160 LOW-LEVEL I2C
// ============================================================

bool bmiWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t bmiReadReg(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  Wire.requestFrom(BMI160_ADDR, (uint8_t)1);

  if (Wire.available())
    return Wire.read();

  return 0xFF;
}

bool readBMI160(float &gx, float &ax, float &ay, float &az) {
  uint8_t data[12];

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_GYR_DATA);

  if (Wire.endTransmission(false) != 0)
    return false;

  Wire.requestFrom(BMI160_ADDR, (uint8_t)12);

  int i = 0;
  while (Wire.available() && i < 12)
    data[i++] = Wire.read();

  if (i != 12)
    return false;

  int16_t gxRaw =
    (int16_t)((data[1] << 8) | data[0]);

  int16_t axRaw =
    (int16_t)((data[7] << 8) | data[6]);

  int16_t ayRaw =
    (int16_t)((data[9] << 8) | data[8]);

  int16_t azRaw =
    (int16_t)((data[11] << 8) | data[10]);

  // Gyro range +/-500 deg/s
  gx = gxRaw / 65.6f;

  // Accelerometer range +/-2g
  ax = axRaw / 16384.0f;
  ay = ayRaw / 16384.0f;
  az = azRaw / 16384.0f;

  return true;
}


// ============================================================
// BMI160 SETUP
// ============================================================

bool setupBMI160() {
  Wire.begin(BMI_SDA, BMI_SCL);
  Wire.setClock(1000000);

  uint8_t id = bmiReadReg(REG_CHIP_ID);

  Serial.print("BMI160 chip ID = 0x");
  Serial.println(id, HEX);

  if (id != 0xD1)
    return false;

  // Accelerometer normal mode
  bmiWriteReg(REG_CMD, 0x11);
  delay(10);

  // Gyro normal mode
  bmiWriteReg(REG_CMD, 0x15);
  delay(100);

  // FAST IMU configuration.
  // 0x2B = normal bandwidth (bwp=010) + ODR 0xB = 800 Hz.
  // Accelerometer: 800 Hz, +/-2g.
  bmiWriteReg(REG_ACC_CONF, 0x2B);
  bmiWriteReg(REG_ACC_RANGE, 0x03);

  // Gyro: 800 Hz, +/-500 deg/s.
  // The old +/-250 range clipped during large disturbances near 249 deg/s.
  bmiWriteReg(REG_GYR_CONF, 0x2B);
  bmiWriteReg(REG_GYR_RANGE, 0x02);

  delay(100);
  return true;
}


// ============================================================
// CALIBRATION
// Hold the assembled robot still (on ground is fine).
// Learns gyro bias and an arbitrary pitch reference; h later learns balance.
// ============================================================

bool calibrateBMI160() {
  sendLine("");
  sendLine("HOLD ROBOT STILL, SUPPORTED ON THE GROUND");
  sendLine("Calibrating BMI160 for 2 seconds...");

  float gyroTotal = 0.0f;
  float sinTotal = 0.0f;
  float cosTotal = 0.0f;
  int count = 0;

  for (int i = 0; i < 1600; i++) {
    float gx, ax, ay, az;

    if (readBMI160(gx, ax, ay, az)) {
      gyroTotal += gx;

      float a = atan2(ay, az);
      sinTotal += sin(a);
      cosTotal += cos(a);
      count++;
    }

    delayMicroseconds(IMU_INTERVAL_US);
  }

  if (count < 1200)
    return false;

  gyroBiasX = gyroTotal / count;

  uprightOffset =
    atan2(sinTotal / count, cosTotal / count)
    * 180.0f / PI;

  balanceAngle = 0.0f;
  gyroX = 0.0f;

  char line[100];
  snprintf(line, sizeof(line),
           "Gyro bias X = %.3f deg/s", gyroBiasX);
  sendLine(line);

  snprintf(line, sizeof(line),
           "Upright accel angle = %.1f deg", uprightOffset);
  sendLine(line);

  sendLine("Upright is now 0 degrees");
  return true;
}


// ============================================================
// LOAD FOC CALIBRATION SAVED BY C2208_FOC_Calibrate_Save.ino
// Saved ESP32 NVS values survive ordinary firmware uploads.
// FAIL CLOSED: do NOT attempt moving-wheel alignment on the ground.
// ============================================================

bool loadSavedMotorCalibration() {
  Preferences prefs;
  if (!prefs.begin("bal_foc", true)) return false;
  bool valid = prefs.getBool("valid", false);
  uint32_t version = prefs.getUInt("ver", 0);
  float zero1 = prefs.getFloat("m1_z", -1.0f);
  float zero2 = prefs.getFloat("m2_z", -1.0f);
  int d1 = prefs.getChar("m1_d", 0);
  int d2 = prefs.getChar("m2_d", 0);
  prefs.end();

  const float maxAngle = 2.0f * PI + 0.02f;
  if (!valid || version != 1 ||
      !isfinite(zero1) || zero1 < 0 || zero1 > maxAngle ||
      !isfinite(zero2) || zero2 < 0 || zero2 > maxAngle ||
      (d1 != 1 && d1 != -1) || (d2 != 1 && d2 != -1)) {
    return false;
  }
  motor1.zero_electric_angle = zero1;
  motor2.zero_electric_angle = zero2;
  motor1.sensor_direction = d1 == 1 ? Direction::CW : Direction::CCW;
  motor2.sensor_direction = d2 == 1 ? Direction::CW : Direction::CCW;

  Serial.printf("Stored M1 zero=%.6f rad, dir=%s\n", zero1,
                d1 == 1 ? "CW" : "CCW");
  Serial.printf("Stored M2 zero=%.6f rad, dir=%s\n", zero2,
                d2 == 1 ? "CW" : "CCW");
  return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  setStatus(StatusCode::Starting);
  delay(500);

  BTSerial.begin(9600, SERIAL_8N1, BT_RX_PIN, -1);
  
  sendLine("");
  sendLine("====================================");
  sendLine("C2208 FAST 800 Hz TORQUE BALANCER");
  sendLine("====================================");


  // ----------------------------------------------------------
  // BMI160
  // ----------------------------------------------------------

  if (!setupBMI160()) {
    sendLine("BMI160 NOT FOUND");
    setStatus(StatusCode::ImuFailed);
    while (1) { updateStatusLED(); delay(10); }
  }

  sendLine("BMI160 found");


  // ----------------------------------------------------------
  // BALANCE / DRIVE POTENTIOMETER
  // 10k pot: one end to 3.3V, other end to GND, wiper to GPIO34.
  // ----------------------------------------------------------

  pinMode(BALANCE_POT_PIN, INPUT);

  // Physical LEARN + ARM button: normally-open switch from GPIO15 to GND.
  pinMode(BALANCE_ARM_BUTTON_PIN, INPUT_PULLUP);
  armButtonRaw = digitalRead(BALANCE_ARM_BUTTON_PIN);
  armButtonStable = armButtonRaw;
  armButtonChangedMs = millis();

  analogReadResolution(12);
  analogSetPinAttenuation(BALANCE_POT_PIN, ADC_11db);

  // Read GPIO34 before motor initialization; this is an ADC input only.
  // Do not let encoder input 2 share this pin.
  potFilteredRaw = (float)analogRead(BALANCE_POT_PIN);
  Serial.printf("Pin check: POT GPIO%d, initial ADC=%.0f; MT6701 #2 DO GPIO%d, CLK GPIO%d\n",
                BALANCE_POT_PIN, potFilteredRaw, SSI2_DO, SSI2_CLK);


  // ----------------------------------------------------------
  // TWO MT6701 encoders
  // Each encoder has its own CLK, CS, and DO.
  // MT1 CLK = GPIO18; MT2 CLK = GPIO33.
  // ----------------------------------------------------------

  pinMode(SSI1_CLK, OUTPUT);
  pinMode(SSI2_CLK, OUTPUT);

  pinMode(SSI1_CS, OUTPUT);
  pinMode(SSI1_DO, INPUT);

  pinMode(SSI2_CS, OUTPUT);
  pinMode(SSI2_DO, INPUT);

  digitalWrite(SSI1_CS, HIGH);
  digitalWrite(SSI2_CS, HIGH);
  digitalWrite(SSI1_CLK, HIGH);
  digitalWrite(SSI2_CLK, HIGH);

  sensor1.init();
  sensor1.update();
  motor1.linkSensor(&sensor1);

  sensor2.init();
  sensor2.update();
  motor2.linkSensor(&sensor2);


  // ----------------------------------------------------------
  // TWO DRV8313 drivers
  // ----------------------------------------------------------

  driver1.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver1.voltage_limit = POWER_SUPPLY_VOLTAGE;

  driver2.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver2.voltage_limit = POWER_SUPPLY_VOLTAGE;

  if (!driver1.init()) {
    sendLine("DRIVER 1 INIT FAILED");
    setStatus(StatusCode::Foc1Failed);
    while (1) { updateStatusLED(); delay(10); }
  }

  if (!driver2.init()) {
    sendLine("DRIVER 2 INIT FAILED");
    setStatus(StatusCode::Foc2Failed);
    while (1) { updateStatusLED(); delay(10); }
  }

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);


  // ----------------------------------------------------------
  // DIRECT VOLTAGE/TORQUE CONTROL
  // No velocity PID.  The balance loop directly commands q-axis
  // voltage.  Encoders remain active for FOC commutation.
  // ----------------------------------------------------------

  motor1.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.voltage_limit = MAX_VOLTAGE;
  motor1.voltage_sensor_align = 3.0f;
  motor1.foc_modulation = FOCModulationType::SinePWM;

  motor2.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.voltage_limit = MAX_VOLTAGE;
  motor2.voltage_sensor_align = 3.0f;
  motor2.foc_modulation = FOCModulationType::SinePWM;


  // ----------------------------------------------------------
  // GROUND-SAFE FOC INITIALIZATION (ABSOLUTE MT6701 SENSORS)
  // Previously saved electrical zero + direction skip all wheel motion.
  // Fail CLOSED when no saved calibration; do not align on the ground.
  // ----------------------------------------------------------

  if (!loadSavedMotorCalibration()) {
    sendLine("NO VALID STORED MOTOR CALIBRATION. BOTH MOTORS OFF.");
    sendLine("Lift wheels and run C2208_FOC_Calibrate_Save.ino first.");
    motor1.disable();
    motor2.disable();
    setStatus(StatusCode::MissingCalibration);
    while (1) { updateStatusLED(); delay(10); }
  }

  sendLine("Motor 1 init using saved FOC values (no alignment)...");
  int init1 = motor1.init();
  int status1 = init1 ? motor1.initFOC() : 0;
  motor1.disable();

  if (!status1) {
    sendLine("MOTOR 1 FOC INITIALIZATION FAILED. STOPPED.");
    motor2.disable();
    setStatus(StatusCode::Foc1Failed);
    while (1) { updateStatusLED(); delay(10); }
  }

  sendLine("Motor 2 init using saved FOC values (no alignment)...");
  int init2 = motor2.init();
  int status2 = init2 ? motor2.initFOC() : 0;
  motor2.disable();

  if (!status2) {
    sendLine("MOTOR 2 FOC INITIALIZATION FAILED. STOPPED.");
    motor1.disable();
    setStatus(StatusCode::Foc2Failed);
    while (1) { updateStatusLED(); delay(10); }
  }

  sendLine("Both motors initialized WITHOUT wheel-rotation alignment.");
  // Both motors now OFF until explicit GPIO15 pushbutton ARM.


  // ----------------------------------------------------------
  // Learn gyro bias and exact upright angle
  // ----------------------------------------------------------

  if (!calibrateBMI160()) {
    sendLine("BMI160 CALIBRATION FAILED");
    setStatus(StatusCode::ImuFailed);
    while (1) { updateStatusLED(); delay(10); }
  }


  // ----------------------------------------------------------
  // WAIT FOR GROUND PLACEMENT AND AN EXPLICIT ARM COMMAND
  // ----------------------------------------------------------
  // Both motors were disabled immediately after saved FOC initialization.
  // In standby we continue IMU sampling and pot trim updates, but
  // motor outputs and wheel-speed PI are inactive.
  balanceArmed = false;
  armRequested = false;
  setStatus(StatusCode::Ready);
  clearDriveState();
  lastImuMicros = micros();
  lastPotMicros = lastImuMicros;

  sendLine("");
  sendLine(BENCH_MIDAIR_TEST ? "BENCH MODE / MOTORS OFF: hold bot in the air." : "STANDBY / MOTORS OFF: hold near balance on ground.");
  sendLine(BENCH_MIDAIR_TEST ? "Press GPIO15: CURRENT pitch is captured immediately; then tilt by hand." : "Place on ground, hold still, and press GPIO15 button to START capture.");
  sendLine("GPIO2 LED: 1 pulse READY; 2 missing saved FOC; 3/4 motor fail; 5 IMU fail.");
  sendLine("f = CONTROLLED ZERO-RPM STOP; x = COAST/DISARM.");
  sendLine("If BOTH motors push a lean farther, reverse MOTOR_SIGN");
  sendLine("If motors fight each other, reverse MOTOR2_DIRECTION");
  sendLine("Fall cutoff = +/-30 deg from balance reference (also absolute 45 deg); limit +/-5 V");
  sendLine("NO FIXED ANGLE: captured at ARM; pot on GPIO34 +/-8 deg");
  sendLine(BENCH_MIDAIR_TEST ? "BENCH: speed PI/auto-trim/wheel-sync OFF; angle+gyro only, 3 V max" : "Outer speed PI at 100 Hz learns slow auto-trim; inner loop 800 Hz");
  sendLine("Automatic trim bounded +/-10 deg; speed PI I bounded +/-3 deg");
  sendLine("FAST MODE: BMI160/control = 800 Hz, I2C = 1 MHz");
  sendLine("HM-10 @9600: a=fwd c=back b=right d=left e/g=speed f=STOP x=COAST");
  sendLine("Gains unchanged: KP=0.45 V/deg, KD=0.0025 V/(deg/s)");
}


// ============================================================
// LOOP
// ============================================================

void loop() {
  updateStatusLED();

  // Keep both FOC loops running as fast as possible while active.
  // Each FOC loop reads its own MT6701 using its own clock pin.
  if (!fallen && balanceArmed) {
    motor1.loopFOC();
    motor2.loopFOC();
  }


  // ----------------------------------------------------------
  // 100 Hz POT + OUTER WHEEL-SPEED P LOOP + WHEEL SYNC
  //
  // Pot is only optional MANUAL trim; Bluetooth commands RPM.
  // Velocity PI corrects drift and quietly learns the true zero-speed
  // neutral angle. The 800 Hz angle/gyro loop performs balancing.
  // ----------------------------------------------------------

  uint32_t nowMicros = micros();

  // ----------------------------------------------------------
  // PHYSICAL LEARN + ARM BUTTON
  // Normally-open pushbutton: GPIO15 -> switch -> GND.
  // Debounced HIGH-to-LOW edge queues the next stable-angle capture.
  // ----------------------------------------------------------
  bool newArmButtonRaw = digitalRead(BALANCE_ARM_BUTTON_PIN);
  uint32_t nowMsForButton = millis();

  if (newArmButtonRaw != armButtonRaw) {
    armButtonRaw = newArmButtonRaw;
    armButtonChangedMs = nowMsForButton;
  }

  if ((uint32_t)(nowMsForButton - armButtonChangedMs) >= ARM_BUTTON_DEBOUNCE_MS &&
      armButtonStable != armButtonRaw) {
    armButtonStable = armButtonRaw;

    if (armButtonStable == LOW && !balanceArmed && !fallen) {
      armRequested = true;
      armRequestedMs = millis();
      nearBalanceSinceMs = 0;
      trackingHeldAngle = false;
      setStatus(StatusCode::CapturePending);
      if (BENCH_MIDAIR_TEST)
        sendLine("BUTTON: immediate mid-air pitch capture requested.");
      else
        sendLine("ARM BUTTON DETECTED. Hold still; capturing balance reference...");
    }
  }

  if (!fallen &&
      (uint32_t)(nowMicros - lastPotMicros) >= POT_INTERVAL_US) {

    float outerDt =
      (nowMicros - lastPotMicros) / 1000000.0f;

    lastPotMicros = nowMicros;

    int potRaw = analogRead(BALANCE_POT_PIN);

    potFilteredRaw +=
      POT_FILTER_ALPHA * ((float)potRaw - potFilteredRaw);

    float centered = potFilteredRaw - (float)POT_CENTER_ADC;

    // Small center deadband makes the balance trim easy to repeat.
    if (fabs(centered) < POT_CENTER_DEADBAND)
      centered = 0.0f;

    float normalized;
    if (centered >= 0.0f)
      normalized = centered / (4095.0f - (float)POT_CENTER_ADC);
    else
      normalized = centered / (float)POT_CENTER_ADC;

    normalized = constrain(normalized, -1.0f, +1.0f);

    // Optional MANUAL trim. No fixed 10-degree bias anywhere.
    potBalanceBiasDeg =
      POT_DIRECTION * normalized * POT_BALANCE_RANGE_DEG;

    // Always keep the pot/target angle current while disarmed.
    // Do not run the wheel-speed PI or use stale encoder speed in standby.
    if (!balanceArmed) {
      targetPitchDeg = learnedNeutralDeg + learnedAutoTrimDeg + potBalanceBiasDeg;
      clearDriveState();
    }
    else {

    // In mid-air bench mode, free-spinning wheel RPM must NOT feed back
    // into requested body lean. Keep the captured pitch fixed (plus pot trim)
    // and disable speed PI, auto-trim and wheel synchronization.
    if (BENCH_MIDAIR_TEST) {
      targetRPM = 0.0f;
      filteredTargetRPM = 0.0f;
      speedErrorRPM = 0.0f;
      speedIntegralDeg = 0.0f;
      rawSpeedLeanDeg = 0.0f;
      speedLeanDeg = 0.0f;
      wheelSyncVoltage = 0.0f;
      learnedAutoTrimDeg = 0.0f;
      targetPitchDeg = learnedNeutralDeg + potBalanceBiasDeg;
    }
    else {

    // Wireless speed request.  Before the first BLE command, requested
    // speed is zero.
    float desiredWirelessRPM =
      bluetoothDriveEnabled ? commandedRPM : 0.0f;

    // SimpleFOC-style throttle low-pass filter.
    // With tau = 0.50 s, starts and STOP are smooth without creating
    // a separate braking state machine.
    float throttleAlpha =
      outerDt / (THROTTLE_FILTER_TAU_SEC + outerDt);

    filteredTargetRPM +=
      throttleAlpha *
      (desiredWirelessRPM - filteredTargetRPM);

    targetRPM = filteredTargetRPM;

    if (!bluetoothDriveEnabled)
      commandedSteerRPM = 0.0f;

    // PHYSICAL wheel RPM for each side. MOTOR2_DIRECTION corrects the
    // second motor's shaft sign so forward travel has the same RPM sign
    // on both wheels.
    wheel1RPM =
      motor1.shaft_velocity * 60.0f / (2.0f * PI);

    wheel2RPM =
      MOTOR2_DIRECTION * motor2.shaft_velocity
      * 60.0f / (2.0f * PI);

    avgWheelRPM = 0.5f * (wheel1RPM + wheel2RPM);

    filteredWheelRPM +=
      WHEEL_RPM_FILTER_ALPHA *
      (avgWheelRPM - filteredWheelRPM);

    // Wheel-speed sync plus steering.
    // With no steering request, desired difference is zero and the two
    // wheels are synchronized.  Steering deliberately asks for a wheel
    // speed difference while keeping the average drive command unchanged.
    float desiredRpmDifference = 2.0f * commandedSteerRPM;
    float actualRpmDifference = wheel1RPM - wheel2RPM;
    float rpmDifferenceError =
      actualRpmDifference - desiredRpmDifference;

    wheelSyncVoltage =
      WHEEL_SYNC_KP_V_PER_RPM * rpmDifferenceError;

    wheelSyncVoltage =
      constrain(wheelSyncVoltage,
                -MAX_WHEEL_SYNC_V,
                +MAX_WHEEL_SYNC_V);

    // Preserve quiet standing balance when there is no steering request.
    if (fabs(avgWheelRPM) < WHEEL_SYNC_ENABLE_RPM &&
        fabs(commandedSteerRPM) < 0.1f)
      wheelSyncVoltage = 0.0f;

    // SimpleFOC-style velocity error:
    //      measured speed - requested speed
    //
    // With the present sign convention:
    //   forward request at rest -> negative lean -> accelerate forward
    //   forward motion + STOP   -> positive lean -> brake
    float signedWheelRPM =
      SPEED_FEEDBACK_SIGN * filteredWheelRPM;

    speedErrorRPM =
      signedWheelRPM - targetRPM;

    speedIntegralDeg +=
      SPEED_KI_DEG_PER_RPM_SEC *
      speedErrorRPM *
      outerDt;

    speedIntegralDeg =
      constrain(speedIntegralDeg,
                -MAX_SPEED_I_DEG,
                +MAX_SPEED_I_DEG);

    // Learn the effective neutral angle ONLY during sustained, quiet,
    // near-zero-command balancing. Steer, drive, large rocking, and fast
    // wheel movement suspend learning. The speed PI itself always works.
    bool quietBalance =
      fabs(commandedRPM) < 0.1f &&
      fabs(targetRPM) < 0.5f &&
      fabs(commandedSteerRPM) < 0.1f &&
      fabs(signedWheelRPM) < AUTO_MAX_WHEEL_RPM &&
      fabs(gyroX) < AUTO_MAX_GYRO_DPS &&
      fabs(wrap180(balanceAngle - targetPitchDeg)) < AUTO_MAX_ANGLE_ERR_DEG;

    if (quietBalance) {
      if (zeroCommandQuietSinceMs == 0)
        zeroCommandQuietSinceMs = millis();
      if ((uint32_t)(millis() - zeroCommandQuietSinceMs) >= AUTO_QUIET_MS) {
        // Transfer existing I correction, conserving (neutral + I).
        // Limit transfer rate and total auto trim to avoid windup.
        float transfer = speedIntegralDeg *
          (outerDt / (AUTO_TRANSFER_TAU_SEC + outerDt));
        transfer = constrain(transfer,
                             -AUTO_MAX_RATE_DEG_SEC * outerDt,
                             +AUTO_MAX_RATE_DEG_SEC * outerDt);
        float newTrim = constrain(learnedAutoTrimDeg + transfer,
                                  -AUTO_MAX_TRIM_DEG,
                                  +AUTO_MAX_TRIM_DEG);
        float moved = newTrim - learnedAutoTrimDeg;
        learnedAutoTrimDeg = newTrim;
        speedIntegralDeg -= moved;
      }
    } else {
      zeroCommandQuietSinceMs = 0;
    }

    rawSpeedLeanDeg =
      SPEED_KP_DEG_PER_RPM *
      speedErrorRPM +
      speedIntegralDeg;

    rawSpeedLeanDeg =
      constrain(rawSpeedLeanDeg,
                -MAX_SPEED_LEAN_DEG,
                +MAX_SPEED_LEAN_DEG);

    // Second SimpleFOC-style filter: smooth the requested pitch/lean.
    float leanAlpha =
      outerDt / (LEAN_FILTER_TAU_SEC + outerDt);

    speedLeanDeg +=
      leanAlpha *
      (rawSpeedLeanDeg - speedLeanDeg);

    targetPitchDeg =
      learnedNeutralDeg + learnedAutoTrimDeg + potBalanceBiasDeg + speedLeanDeg;
    } // normal ground speed loop
    } // armed: outer loop
  }

  // ----------------------------------------------------------
  // HM-10 BLE UART COMMANDS
  // ----------------------------------------------------------
  // Accept BOTH uppercase and lowercase so app capitalization cannot
  // accidentally disable a command.
  //
  //   a/A = forward, latched
  //   c/C = reverse, latched
  //   b/B = right while held/repeating
  //   d/D = left while held/repeating
  //   e/E = slower
  //   g/G = faster
  //   f/F = ALL STOP
  //   h/H = unused; physical GPIO15 pushbutton performs LEARN+ARM
  //   x/X = COAST/DISARM; support bot while using this
  while (BTSerial.available()) {
    char c = BTSerial.read();

    switch (c) {
      case 'a':
      case 'A':
        if (!balanceArmed) break;
        commandedRPM = +btDriveRPM;
        speedIntegralDeg = 0.0f;
        bluetoothDriveEnabled = true;
        break;

      case 'c':
      case 'C':
        if (!balanceArmed) break;
        commandedRPM = -btDriveRPM;
        speedIntegralDeg = 0.0f;
        bluetoothDriveEnabled = true;
        break;

      case 'b':
      case 'B':
        if (!balanceArmed) break;
        commandedSteerRPM =
          +BT_STEER_DIRECTION * BT_STEER_RPM;
        lastSteerMillis = millis();
        bluetoothDriveEnabled = true;
        break;

      case 'd':
      case 'D':
        if (!balanceArmed) break;
        commandedSteerRPM =
          -BT_STEER_DIRECTION * BT_STEER_RPM;
        lastSteerMillis = millis();
        bluetoothDriveEnabled = true;
        break;

      case 'e':
      case 'E':
        // E = faster / more commanded speed
        if ((uint32_t)(millis() - lastSpeedButtonMillis) >=
            SPEED_BUTTON_REPEAT_MS) {
          lastSpeedButtonMillis = millis();

          btDriveRPM += BT_SPEED_STEP;
          btDriveRPM =
            constrain(btDriveRPM, BT_MIN_RPM, BT_MAX_RPM);

          if (commandedRPM > 0.0f)
            commandedRPM = +btDriveRPM;
          else if (commandedRPM < 0.0f)
            commandedRPM = -btDriveRPM;
        }
        bluetoothDriveEnabled = true;
        break;

      case 'g':
      case 'G':
        // G = slower / less commanded speed
        if ((uint32_t)(millis() - lastSpeedButtonMillis) >=
            SPEED_BUTTON_REPEAT_MS) {
          lastSpeedButtonMillis = millis();

          btDriveRPM -= BT_SPEED_STEP;
          btDriveRPM =
            constrain(btDriveRPM, BT_MIN_RPM, BT_MAX_RPM);

          if (commandedRPM > 0.0f)
            commandedRPM = +btDriveRPM;
          else if (commandedRPM < 0.0f)
            commandedRPM = -btDriveRPM;
        }
        bluetoothDriveEnabled = true;
        break;

      case 'f':
      case 'F':
        // SimpleFOC-style STOP:
        // request zero wheel speed and leave the normal velocity loop active.
        // Clear stored drive integral so it cannot keep pushing forward.
        commandedRPM = 0.0f;
        commandedSteerRPM = 0.0f;
        speedIntegralDeg = 0.0f;
        bluetoothDriveEnabled = true;
        break;

      case 'x':
      case 'X':
        disarmBalance();
        break;

      default:
        break;
    }
  }

  // USB-only standby test: p shows GPIO34 pot ADC and current trim.
  // USB prints never run periodically or during active balancing.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') disarmBalance();
    if ((c == 'p' || c == 'P') && !balanceArmed) {
      Serial.printf("POT GPIO%d raw=%d filtered=%.0f trim=%+.2f deg; encoder2 DO GPIO%d\n",
                    BALANCE_POT_PIN, analogRead(BALANCE_POT_PIN),
                    potFilteredRaw, potBalanceBiasDeg, SSI2_DO);
    }
  }

  // b/d repeat continuously while held.  When repeats stop, return
  // steering to straight but preserve the latched forward/reverse speed.
  if ((uint32_t)(millis() - lastSteerMillis) > STEER_TIMEOUT_MS) {
    commandedSteerRPM = 0.0f;
  }
  // ----------------------------------------------------------
  // 800 Hz BMI160 + balance controller
  // ----------------------------------------------------------

  nowMicros = micros();

  if (!fallen &&
      (uint32_t)(nowMicros - lastImuMicros) >= IMU_INTERVAL_US) {

    float dt =
      (nowMicros - lastImuMicros) / 1000000.0f;

    lastImuMicros = nowMicros;

    float gx, ax, ay, az;

    if (readBMI160(gx, ax, ay, az)) {
      lastGoodImuMs = millis();
      gyroX = gx - gyroBiasX;

      float rawAccelAngle =
        atan2(ay, az) * 180.0f / PI;

      float accelAngle =
        wrap180(rawAccelAngle - uprightOffset);

      // Complementary filter with a fixed TIME CONSTANT rather than
      // fixed 0.98/0.02 coefficients.  This preserves the old ~0.5 s
      // behavior even though the loop is now 8x faster.
      const float alpha =
        COMPLEMENTARY_TAU_SEC / (COMPLEMENTARY_TAU_SEC + dt);

      balanceAngle =
        alpha * (balanceAngle + gyroX * dt)
        +
        (1.0f - alpha) * accelAngle;

      balanceAngle = wrap180(balanceAngle);

      // In standby learn whether the held body is genuinely STATIONARY.
      // Do not assume it should be at a coded angle. Update a slowly
      // filtered held angle and restart the timer on sudden pitch changes.
      // It cannot detect ground contact; GPIO15 is the deliberate arm request.
      if (!balanceArmed) {
        if (fabs(balanceAngle) <= ARM_MAX_CAPTURE_ANGLE_DEG &&
            fabs(gyroX) <= ARM_MAX_GYRO_DPS) {
          if (!trackingHeldAngle ||
              fabs(wrap180(balanceAngle - armHeldAngleDeg)) >
                ARM_HOLD_TOLERANCE_DEG) {
            armHeldAngleDeg = balanceAngle;
            nearBalanceSinceMs = lastGoodImuMs;
            trackingHeldAngle = true;
          } else {
            armHeldAngleDeg +=
              0.01f * wrap180(balanceAngle - armHeldAngleDeg);
          }
        } else {
          nearBalanceSinceMs = 0;
          trackingHeldAngle = false;
        }
      }

      // ------------------------------------------------------
      // FALL SAFETY
      // ------------------------------------------------------

      if (balanceArmed &&
          (fabs(wrap180(balanceAngle -
              (learnedNeutralDeg + learnedAutoTrimDeg + potBalanceBiasDeg))) >
              FALL_ANGLE_DEG ||
           fabs(balanceAngle) > 45.0f)) {
        targetVoltage = 0.0f;
        motor1.move(0.0f);
        motor2.move(0.0f);

        motor1.disable();
        motor2.disable();
        fallen = true;
        balanceArmed = false;
        armRequested = false;
        setStatus(StatusCode::Fallen);

        sendLine("*** FALL LIMIT - BOTH MOTORS DISABLED - RESET TO REARM ***");
      }

      else if (balanceArmed) {

        // ----------------------------------------------------
        // CASCADED SPEED PI + BALANCE CONTROL
        //
        // The 100 Hz outer loop has already converted pot-commanded
        // target RPM into targetPitchDeg.  This fast 800 Hz inner loop
        // now does exactly what worked well before: angle + gyro ->
        // direct motor voltage.
        // ----------------------------------------------------

        float angleErrorDeg = balanceAngle - targetPitchDeg;

        targetVoltage =
          MOTOR_SIGN *
          (KP_ANGLE * angleErrorDeg + KD_GYRO * gyroX);

        const float activeVoltageLimit =
          BENCH_MIDAIR_TEST ? BENCH_MAX_VOLTAGE : MAX_VOLTAGE;

        targetVoltage =
          constrain(targetVoltage,
                    -activeVoltageLimit,
                    +activeVoltageLimit);

        // Wheel-speed sync correction. The faster wheel gets slightly
        // less voltage and the slower wheel gets slightly more. Using
        // equal/opposite corrections preserves the average balance torque.
        float motor1Voltage =
          targetVoltage - wheelSyncVoltage;

        float motor2PhysicalVoltage =
          targetVoltage + wheelSyncVoltage;

        motor1Voltage =
          constrain(motor1Voltage,
                    -activeVoltageLimit,
                    +activeVoltageLimit);

        motor2PhysicalVoltage =
          constrain(motor2PhysicalVoltage,
                    -activeVoltageLimit,
                    +activeVoltageLimit);

        motor1.move(motor1Voltage);
        motor2.move(MOTOR2_DIRECTION * motor2PhysicalVoltage);
      }
    }
  }


  // Process a pending button press AFTER the latest IMU update.
  // No serial/LED activity runs continuously when balancing is active.
  if (armRequested && !balanceArmed && !fallen) {
    if ((uint32_t)(millis() - armRequestedMs) >= ARM_REQUEST_TIMEOUT_MS) {
      armRequested = false;
      setStatus(StatusCode::CaptureTimeout);
      if (BENCH_MIDAIR_TEST)
        sendLine("BENCH ARM TIMED OUT: no fresh BMI160 sample.");
      else
        sendLine("ARM CAPTURE TIMED OUT: no sufficiently still 1-second interval.");
    } else {
      tryArmBalance();
    }
  }

}

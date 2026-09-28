#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>
#include <math.h>

/*
  ONE-TIME C2208 FOC CALIBRATION ONLY -- NO IMU, NO BALANCING
  ESP32 WROOM, 2x MT6701 SSI absolute sensors, 2x DRV8313.

  Lift robot so BOTH WHEELS SPIN FREELY, then open USB Serial at 115200.
  Type C to calibrate BOTH motors, check their offsets, and save in
  ESP32 NVS namespace "bal_foc". Type R to read the saved values.
  The sketch NEVER drives the wheels continuously or balances the robot.

  Saved NVS values survive ordinary Arduino sketch uploads and power-off.
  Erase All Flash, changed encoder mounting, phase wiring, or different
  motors require re-calibration. A ground-start balancer must LOAD them.
*/

const int SSI1_CLK = 18, SSI1_CS = 32, SSI1_DO = 19;
const int SSI2_CLK = 33, SSI2_CS = 13, SSI2_DO = 23;
const int M1_PWM_A = 25, M1_PWM_B = 26, M1_PWM_C = 27, M1_EN = 14;
const int M2_PWM_A = 5, M2_PWM_B = 16, M2_PWM_C = 17, M2_EN = 4;

const float SUPPLY_V = 8.0f;  // Match your 2-cell battery supply; CHANGE for 12-V testing.
const float ALIGN_V  = 3.0f;  // Same voltage_sensor_align as current balance sketch.
const uint32_t FORMAT_VERSION = 1;
const char* NVS_NAMESPACE = "bal_foc";

uint16_t readMT6701Raw(int clkPin, int csPin, int doPin) {
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
    if (digitalRead(doPin)) data |= 1;
  }
  digitalWrite(clkPin, HIGH);
  digitalWrite(csPin, HIGH);
  return (data >> 10) & 0x3FFF;
}

float readMT1() {
  return readMT6701Raw(SSI1_CLK, SSI1_CS, SSI1_DO) * (2.0f * PI / 16384.0f);
}
float readMT2() {
  return readMT6701Raw(SSI2_CLK, SSI2_CS, SSI2_DO) * (2.0f * PI / 16384.0f);
}

GenericSensor sensor1(readMT1);
GenericSensor sensor2(readMT2);
BLDCMotor motor1(7);
BLDCMotor motor2(7);
BLDCDriver3PWM driver1(M1_PWM_A, M1_PWM_B, M1_PWM_C, M1_EN);
BLDCDriver3PWM driver2(M2_PWM_A, M2_PWM_B, M2_PWM_C, M2_EN);

void stopBoth() {
  motor1.disable();
  motor2.disable();
}

int dirCode(Direction d) {
  if (d == Direction::CW) return 1;
  if (d == Direction::CCW) return -1;
  return 0;
}

bool goodAngle(float v) { return isfinite(v) && v >= 0.0f && v <= (2.0f * PI + 0.02f); }

void printStored() {
  Preferences p;
  if (!p.begin(NVS_NAMESPACE, true)) {
    Serial.println("NVS unavailable.");
    return;
  }
  bool valid = p.getBool("valid", false);
  unsigned version = p.getUInt("ver", 0);
  float z1 = p.getFloat("m1_z", -1.0f);
  float z2 = p.getFloat("m2_z", -1.0f);
  int d1 = p.getChar("m1_d", 0);
  int d2 = p.getChar("m2_d", 0);
  p.end();
  if (!valid || version != FORMAT_VERSION || !goodAngle(z1) || !goodAngle(z2) ||
      (d1 != 1 && d1 != -1) || (d2 != 1 && d2 != -1)) {
    Serial.println("No complete, valid motor calibration in NVS.");
    return;
  }
  Serial.printf("SAVED M1: zero=%.6f rad, dir=%s\n", z1, d1 == 1 ? "CW" : "CCW");
  Serial.printf("SAVED M2: zero=%.6f rad, dir=%s\n", z2, d2 == 1 ? "CW" : "CCW");
  Serial.println("NVS namespace: bal_foc (version 1)");
}

void runCalibration() {
  // No surprise startup motion: calibration runs only after explicit C.
  Serial.println("Starting MOTOR 1 alignment; keep both wheels off ground.");
  if (!motor1.init()) { stopBoth(); Serial.println("M1 init failed; nothing saved."); return; }
  int ok1 = motor1.initFOC();
  stopBoth();
  if (!ok1 || !goodAngle(motor1.zero_electric_angle) || !dirCode(motor1.sensor_direction)) {
    Serial.println("M1 calibration failed. Existing NVS values unchanged.");
    return;
  }
  Serial.printf("M1: zero=%.6f rad, dir=%s\n", motor1.zero_electric_angle,
                dirCode(motor1.sensor_direction) == 1 ? "CW" : "CCW");

  Serial.println("Starting MOTOR 2 alignment.");
  if (!motor2.init()) { stopBoth(); Serial.println("M2 init failed; nothing saved."); return; }
  int ok2 = motor2.initFOC();
  stopBoth();
  if (!ok2 || !goodAngle(motor2.zero_electric_angle) || !dirCode(motor2.sensor_direction)) {
    Serial.println("M2 calibration failed. Existing NVS values unchanged.");
    return;
  }
  Serial.printf("M2: zero=%.6f rad, dir=%s\n", motor2.zero_electric_angle,
                dirCode(motor2.sensor_direction) == 1 ? "CW" : "CCW");

  Preferences p;
  if (!p.begin(NVS_NAMESPACE, false)) {
    Serial.println("Cannot open NVS: NOT SAVED."); return;
  }
  // Write the 'valid' marker LAST. An interrupted write cannot be mistaken
  // for a finished pair of calibrations.
  p.putBool("valid", false);
  bool saved = p.putUInt("ver", FORMAT_VERSION) == sizeof(uint32_t);
  saved &= p.putFloat("m1_z", motor1.zero_electric_angle) == sizeof(float);
  saved &= p.putChar("m1_d", (int8_t)dirCode(motor1.sensor_direction)) == 1;
  saved &= p.putFloat("m2_z", motor2.zero_electric_angle) == sizeof(float);
  saved &= p.putChar("m2_d", (int8_t)dirCode(motor2.sensor_direction)) == 1;
  if (saved) saved = p.putBool("valid", true) == 1;
  p.end();
  if (saved) Serial.println("BOTH MOTOR CALIBRATIONS SAVED TO ESP32 NVS.");
  else Serial.println("NVS WRITE FAILED. Do not use the ground-start balancer.");
  printStored();
  Serial.println("Both drivers remain DISABLED. You may upload the balance sketch.");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(SSI1_CLK, OUTPUT); pinMode(SSI2_CLK, OUTPUT);
  pinMode(SSI1_CS, OUTPUT);  pinMode(SSI2_CS, OUTPUT);
  pinMode(SSI1_DO, INPUT);  pinMode(SSI2_DO, INPUT);
  digitalWrite(SSI1_CLK, HIGH); digitalWrite(SSI2_CLK, HIGH);
  digitalWrite(SSI1_CS, HIGH); digitalWrite(SSI2_CS, HIGH);

  sensor1.init(); sensor1.update(); motor1.linkSensor(&sensor1);
  sensor2.init(); sensor2.update(); motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = SUPPLY_V;
  driver2.voltage_power_supply = SUPPLY_V;
  driver1.voltage_limit = SUPPLY_V;
  driver2.voltage_limit = SUPPLY_V;
  bool driver1Ready = driver1.init();
  bool driver2Ready = driver2.init();
  if (!driver1Ready || !driver2Ready) {
    if (driver1Ready) driver1.disable();
    if (driver2Ready) driver2.disable();
    Serial.println("Driver initialization failed; check power/wiring.");
    while (true) delay(1000);
  }
  driver1.disable(); driver2.disable();
  motor1.linkDriver(&driver1); motor2.linkDriver(&driver2);
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;
  motor1.voltage_limit = 5.0f; motor2.voltage_limit = 5.0f;
  motor1.voltage_sensor_align = ALIGN_V;
  motor2.voltage_sensor_align = ALIGN_V;
  motor1.foc_modulation = FOCModulationType::SinePWM;
  motor2.foc_modulation = FOCModulationType::SinePWM;

  Serial.println("\nC2208 ONE-TIME FOC ALIGN/SAVE (NO BALANCE CONTROL)");
  printStored();
  Serial.println("LIFT BOTH WHEELS; enter C to calibrate and save; R to read NVS.");
}

void loop() {
  static bool attemptedThisBoot = false;
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r' || c == 'R') printStored();
  if (c == 'c' || c == 'C') {
    if (attemptedThisBoot) {
      Serial.println("Reset the ESP32 before repeating C (forces full realignment).");
    } else {
      attemptedThisBoot = true;
      runCalibration();
    }
  }
}

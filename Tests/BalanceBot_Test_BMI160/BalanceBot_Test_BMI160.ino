#include <Wire.h>
#include <math.h>

// ============================================================
// BALANCE BOT - BMI160 STANDALONE TEST
// ESP32 WROOM + BMI160
//
// Wiring from Gimbal_Balance_Bot.ino:
//   BMI160 SDA -> GPIO 21
//   BMI160 SCL -> GPIO 22
//   BMI160 3.3V -> ESP32 3.3V
//   BMI160 GND -> GND
//   BMI160 CS -> 3.3V
//   BMI160 SA0 -> GND   (I2C address 0x68)
//
// Open Serial Monitor at 115200 baud.
// No motors or encoders are used by this test.
// ============================================================

const int BMI_SDA = 21;
const int BMI_SCL = 22;
const uint8_t BMI160_ADDR = 0x68;

const uint8_t REG_CHIP_ID   = 0x00;
const uint8_t REG_GYR_DATA  = 0x0C;
const uint8_t REG_ACC_CONF  = 0x40;
const uint8_t REG_ACC_RANGE = 0x41;
const uint8_t REG_GYR_CONF  = 0x42;
const uint8_t REG_GYR_RANGE = 0x43;
const uint8_t REG_CMD       = 0x7E;

float gyroBiasX = 0.0f;

bool bmiWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

uint8_t bmiReadReg(uint8_t reg) {
  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;

  Wire.requestFrom(BMI160_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

bool readBMI160(float &gx, float &ax, float &ay, float &az) {
  uint8_t data[12];

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_GYR_DATA);
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(BMI160_ADDR, (uint8_t)12);

  int i = 0;
  while (Wire.available() && i < 12) data[i++] = Wire.read();
  if (i != 12) return false;

  int16_t gxRaw = (int16_t)((data[1]  << 8) | data[0]);
  int16_t axRaw = (int16_t)((data[7]  << 8) | data[6]);
  int16_t ayRaw = (int16_t)((data[9]  << 8) | data[8]);
  int16_t azRaw = (int16_t)((data[11] << 8) | data[10]);

  // Same ranges used by the balance-bot program.
  gx = gxRaw / 65.6f;       // +/-500 deg/s
  ax = axRaw / 16384.0f;    // +/-2 g
  ay = ayRaw / 16384.0f;
  az = azRaw / 16384.0f;
  return true;
}

bool setupBMI160() {
  Wire.begin(BMI_SDA, BMI_SCL);
  Wire.setClock(400000);  // conservative diagnostic speed
  delay(50);

  uint8_t id = bmiReadReg(REG_CHIP_ID);
  Serial.printf("BMI160 chip ID = 0x%02X  ", id);
  if (id != 0xD1) {
    Serial.println("FAIL (expected 0xD1)");
    return false;
  }
  Serial.println("PASS");

  // Accelerometer normal mode.
  if (!bmiWriteReg(REG_CMD, 0x11)) return false;
  delay(10);

  // Gyro normal mode.
  if (!bmiWriteReg(REG_CMD, 0x15)) return false;
  delay(100);

  // Same 800 Hz / ranges as main balance sketch.
  bmiWriteReg(REG_ACC_CONF, 0x2B);
  bmiWriteReg(REG_ACC_RANGE, 0x03); // +/-2 g
  bmiWriteReg(REG_GYR_CONF, 0x2B);
  bmiWriteReg(REG_GYR_RANGE, 0x02); // +/-500 deg/s
  delay(100);

  return true;
}

void measureGyroBias() {
  Serial.println("Hold BMI160 still for 2 seconds...");

  float total = 0.0f;
  int good = 0;
  uint32_t start = millis();

  while (millis() - start < 2000) {
    float gx, ax, ay, az;
    if (readBMI160(gx, ax, ay, az)) {
      total += gx;
      good++;
    }
    delay(2);
  }

  if (good > 100) {
    gyroBiasX = total / good;
    Serial.printf("Gyro X bias = %.3f deg/s\n", gyroBiasX);
  } else {
    Serial.println("Not enough valid BMI160 samples.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("========================================");
  Serial.println("BALANCE BOT - BMI160 STANDALONE TEST");
  Serial.println("========================================");

  if (!setupBMI160()) {
    Serial.println();
    Serial.println("BMI160 NOT FOUND.");
    Serial.println("Check 3.3V, GND, SDA=21, SCL=22, CS=3.3V, SA0=GND.");
    while (true) delay(1000);
  }

  measureGyroBias();
  Serial.println();
  Serial.println("Move/tilt the board. Values should change smoothly.");
  Serial.println("At rest, acceleration magnitude should be near 1.00 g.");
  Serial.println();
}

void loop() {
  float gx, ax, ay, az;

  if (!readBMI160(gx, ax, ay, az)) {
    Serial.println("READ ERROR");
    delay(250);
    return;
  }

  gx -= gyroBiasX;

  float pitchDeg = atan2f(ay, az) * 180.0f / PI;
  float gTotal = sqrtf(ax * ax + ay * ay + az * az);

  Serial.printf("GX=%+7.2f dps   AX=%+6.3f  AY=%+6.3f  AZ=%+6.3f   |A|=%5.3f g   pitch=%+7.2f deg\n",
                gx, ax, ay, az, gTotal, pitchDeg);

  delay(100); // 10 lines/sec, easy to read
}

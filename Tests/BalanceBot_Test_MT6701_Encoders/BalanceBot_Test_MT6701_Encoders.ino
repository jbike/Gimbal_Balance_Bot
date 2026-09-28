#include <Arduino.h>

// ============================================================
// BALANCE BOT - TWO MT6701 ENCODER STANDALONE TEST
// ESP32 WROOM + two MT6701 encoders in SSI mode
//
// Wiring from Gimbal_Balance_Bot.ino:
//
// MT6701 #1:
//   DO  -> GPIO 19
//   CLK -> GPIO 18
//   CS  -> GPIO 32
//
// MT6701 #2:
//   DO  -> GPIO 23
//   CLK -> GPIO 33
//   CS  -> GPIO 13
//
// IMPORTANT: GPIO34 is the balance pot, NOT encoder #2 DO.
//
// Open Serial Monitor at 115200 baud.
// No BMI160 or motor drivers are used by this test.
// ============================================================

const int SSI1_CLK = 18;
const int SSI1_CS  = 32;
const int SSI1_DO  = 19;

const int SSI2_CLK = 33;
const int SSI2_CS  = 13;
const int SSI2_DO  = 23;

uint16_t readMT6701Raw(int clkPin, int csPin, int doPin) {
  uint32_t data = 0;

  digitalWrite(csPin, LOW);
  delayMicroseconds(1);

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

  return (data >> 10) & 0x3FFF; // 14-bit absolute angle
}

float rawToDegrees(uint16_t raw) {
  return raw * (360.0f / 16384.0f);
}

void setup() {
  Serial.begin(115200);
  delay(800);

  pinMode(SSI1_CLK, OUTPUT);
  pinMode(SSI1_CS, OUTPUT);
  pinMode(SSI1_DO, INPUT);

  pinMode(SSI2_CLK, OUTPUT);
  pinMode(SSI2_CS, OUTPUT);
  pinMode(SSI2_DO, INPUT);

  digitalWrite(SSI1_CLK, HIGH);
  digitalWrite(SSI2_CLK, HIGH);
  digitalWrite(SSI1_CS, HIGH);
  digitalWrite(SSI2_CS, HIGH);

  Serial.println();
  Serial.println("========================================");
  Serial.println("BALANCE BOT - MT6701 ENCODER TEST");
  Serial.println("========================================");
  Serial.println("Turn each wheel slowly by hand.");
  Serial.println("Each RAW value should sweep through 0..16383 and wrap once/revolution.");
  Serial.println("A value stuck at 0, 1, or 16383 usually means a wiring/mode problem.");
  Serial.println();
}

void loop() {
  uint16_t raw1 = readMT6701Raw(SSI1_CLK, SSI1_CS, SSI1_DO);
  uint16_t raw2 = readMT6701Raw(SSI2_CLK, SSI2_CS, SSI2_DO);

  float deg1 = rawToDegrees(raw1);
  float deg2 = rawToDegrees(raw2);

  Serial.printf("ENC1 raw=%5u  deg=%7.2f     ENC2 raw=%5u  deg=%7.2f\n",
                raw1, deg1, raw2, deg2);

  delay(100);
}

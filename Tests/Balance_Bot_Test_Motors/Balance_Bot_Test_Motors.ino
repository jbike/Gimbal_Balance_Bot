#include <SimpleFOC.h>

// ============================================================
// BALANCE BOT - SIMPLEFOC PHASE ROTATION TEST
//
// ESP32 + DRV8313 + MOTOR ONLY
//
// NO BMI160
// NO ENCODERS
// NO initFOC()
// NO saved calibration
//
// KEEP BOTH WHEELS OFF THE GROUND
// ============================================================

const float SUPPLY_VOLTAGE = 8.0f;
const float TEST_VOLTAGE   = 3.0f;

// C2208 = 7 pole pairs
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);

// Same pins as working Balance Bot program
BLDCDriver3PWM driver1(25, 26, 27, 14);
BLDCDriver3PWM driver2(5, 16, 17, 4);


// ------------------------------------------------------------
// Rotate one ELECTRICAL revolution.
//
// setPhaseVoltage() is SimpleFOC's own phase-generation
// routine.  We simply feed it changing electrical angles.
// ------------------------------------------------------------

void electricalRevolution(BLDCMotor &motor, bool reverse)
{
  const int STEPS = 120;

  if (!reverse)
  {
    for (int i = 0; i < STEPS; i++)
    {
      float angle =
        2.0f * PI * (float)i / (float)STEPS;

      motor.setPhaseVoltage(
        TEST_VOLTAGE,   // Uq
        0.0f,           // Ud
        angle
      );

      delay(8);
    }
  }
  else
  {
    for (int i = STEPS - 1; i >= 0; i--)
    {
      float angle =
        2.0f * PI * (float)i / (float)STEPS;

      motor.setPhaseVoltage(
        TEST_VOLTAGE,
        0.0f,
        angle
      );

      delay(8);
    }
  }
}


void stopMotor(BLDCMotor &motor)
{
  motor.setPhaseVoltage(0.0f, 0.0f, 0.0f);
}


void runMotorForward(BLDCMotor &motor, const char *name)
{
  Serial.print(name);
  Serial.println(" FORWARD");

  // 7 electrical revolutions = about one mechanical revolution
  for (int n = 0; n < 14; n++)
  {
    electricalRevolution(motor, false);
  }

  stopMotor(motor);
  delay(1000);
}


void runMotorReverse(BLDCMotor &motor, const char *name)
{
  Serial.print(name);
  Serial.println(" REVERSE");

  for (int n = 0; n < 14; n++)
  {
    electricalRevolution(motor, true);
  }

  stopMotor(motor);
  delay(1000);
}


void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("======================================");
  Serial.println(" SimpleFOC direct phase rotation test");
  Serial.println(" KEEP WHEELS OFF THE GROUND");
  Serial.println("======================================");
  Serial.println();


  // ----------------------
  // Driver 1
  // ----------------------

  driver1.voltage_power_supply = SUPPLY_VOLTAGE;
  driver1.voltage_limit = SUPPLY_VOLTAGE;

  if (!driver1.init())
  {
    Serial.println("Driver 1 init FAILED");
    while (1);
  }

  Serial.println("Driver 1 init OK");


  // ----------------------
  // Driver 2
  // ----------------------

  driver2.voltage_power_supply = SUPPLY_VOLTAGE;
  driver2.voltage_limit = SUPPLY_VOLTAGE;

  if (!driver2.init())
  {
    Serial.println("Driver 2 init FAILED");
    while (1);
  }

  Serial.println("Driver 2 init OK");


  // ----------------------
  // Motors
  // ----------------------

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.voltage_limit = TEST_VOLTAGE;
  motor2.voltage_limit = TEST_VOLTAGE;

  motor1.foc_modulation =
    FOCModulationType::SinePWM;

  motor2.foc_modulation =
    FOCModulationType::SinePWM;


  if (!motor1.init())
  {
    Serial.println("Motor 1 init FAILED");
    while (1);
  }

  Serial.println("Motor 1 init OK");


  if (!motor2.init())
  {
    Serial.println("Motor 2 init FAILED");
    while (1);
  }

  Serial.println("Motor 2 init OK");

  Serial.println();
  Serial.println("Starting in 2 seconds...");
  delay(2000);
}


void loop()
{
  Serial.println();
  Serial.println("Testing MOTOR 1");

  runMotorForward(motor1, "Motor 1");
  runMotorReverse(motor1, "Motor 1");


  Serial.println();
  Serial.println("Testing MOTOR 2");

  runMotorForward(motor2, "Motor 2");
  runMotorReverse(motor2, "Motor 2");


  stopMotor(motor1);
  stopMotor(motor2);

  Serial.println();
  Serial.println("Cycle finished");
  Serial.println("Repeating in 3 seconds");

  delay(3000);
}

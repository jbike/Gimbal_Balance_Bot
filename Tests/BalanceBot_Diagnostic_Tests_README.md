# Balance Bot Standalone Diagnostic Tests

These sketches use the pin assignments from `Gimbal_Balance_Bot.ino` and intentionally test one subsystem at a time.

## 1. `BalanceBot_Test_BMI160.ino`

Tests only the BMI160 over I2C.

Expected behavior:
- Chip ID should be `0xD1`.
- At rest, acceleration magnitude `|A|` should be near `1.0 g`.
- `GX` should be near zero after the startup bias measurement.
- Tilting the board should change the accelerometer values and calculated pitch smoothly.

If chip ID is `0xFF`, first check BMI160 power and I2C wiring.

## 2. `BalanceBot_Test_MT6701_Encoders.ino`

Tests only the two MT6701 absolute encoders.

Expected behavior:
- Turning wheel 1 changes ENC1 from 0 through 16383 and 0 through 360 degrees.
- Turning wheel 2 changes ENC2 independently.
- The value wraps once per mechanical revolution.

A reading stuck at 0, 1, or 16383 usually points to DO/CLK/CS wiring or the encoder mode-pad configuration.

Current pins:
- Encoder 1: DO 19, CLK 18, CS 32
- Encoder 2: DO 23, CLK 33, CS 13
- GPIO34 remains reserved for the balance potentiometer.

## 3. `BalanceBot_Test_Motors_OpenLoop.ino`

Tests only the two C2208 motors and DRV8313 drivers. It does **not** use the encoders or BMI160.

**Lift both wheels off the ground before running it.**

Sequence repeats automatically:
1. Motor 1 forward
2. Motor 1 stop
3. Motor 1 reverse
4. Motor 1 stop
5. Motor 2 forward
6. Motor 2 stop
7. Motor 2 reverse
8. Motor 2 stop
9. pause

The test uses only 2 V motor voltage and about 76 RPM open-loop speed. Both motors should behave similarly. If one motor fails here while the other works, the problem is downstream of the ESP32 control logic: motor wiring, DRV8313, enable wiring, power, or the motor itself.

Because this is an open-loop diagnostic, its direction does not establish the final balance-bot motor direction. Final direction is handled by the full balance program.

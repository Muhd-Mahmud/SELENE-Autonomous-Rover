# Safety and test boundaries

[← Back to README](../README.md)

SELENE is a development prototype. Motor wiring, energy storage, and autonomous control require independent validation.

## Electrical and battery checklist

- **Power OFF** when changing wiring. Prevent shorts and use a battery with appropriate protection and a reachable physical disconnect.
- A 2S Li-ion battery reaches **8.4 V**. Typical yellow TT motors may be rated **3–6 V**: confirm the actual motor label/datasheet before connecting VM. Use a separate motor-rated regulator as needed.
- `TB6612FNG.VCC` is the **logic supply**, typically 3.3 V with this controller; `TB6612FNG.VM` is the **motor supply**. They are different rails.
- An Arduino PWM duty limit does not eliminate 8.4 V peaks at the motor terminals.
- The 5 V buck output feeds the ESP32 board's specified 5 V/VIN input, not its 3.3 V pin. Confirm the actual DevKitC variant.
- Ensure **common ground** between the controller, motor driver and all module signals.
- Never apply 5 V to an ESP32 GPIO. Verify LM393 D0 and tracker output voltage when powered.

## Motion checklist

- Start with `SELENE_ENABLE_MOTOR_TESTS 0` in private config.
- Confirm motor voltage, polarity and wiring before turning short tests on.
- Secure the robot so **both wheels are raised** for automated encoder tests.
- Keep clothing and hands clear and a manual power disconnect within reach.
- Keep `SELENE_ENABLE_GUARDED_CRAWL 0` and `SELENE_DRIVETRAIN_VERIFIED 0` until the documented validation milestones are complete.
- A single forward VL53L0X does not detect side/rear hazards, drop-offs or reliably classify all obstacles. No unsupervised navigation.

## Data credibility

- Provisional odometry does **not** establish localization, SLAM or reliable dead reckoning.
- LM393 single-channel encoders do not directly measure wheel direction.
- If `tof_ok` is false, do not treat `tof_mm` as a valid distance.

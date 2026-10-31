# Development roadmap

[← Back to README](../README.md) · [Changelog](CHANGELOG.md)

The roadmap records demonstrated functionality separately from experimental and future capabilities. A completed code change is not the same thing as a field-validated feature.

## Phase 0 — Bench platform ✅
- [x] ESP32-S3 PlatformIO firmware.
- [x] Battery-powered Wi-Fi AP and embedded browser dashboard.
- [x] TB6612FNG motor drive and guarded short test pulses.
- [x] MPU6500 register-level readout and live IMU telemetry.
- [x] LM393 optical pulse counters on both wheels.
- [x] Automated three-burst, raised-wheel encoder repeatability tests.
- [ ] Motor power rail independently verified against exact motor voltage rating.

## Phase 1 — Odometry calibration 🚧
- [x] Add provisional wheel diameter, track width, encoder PPR in v4.3 config.
- [x] Exclude lifted-wheel pulses from estimated chassis motion.
- [ ] Measure actual pulses across one complete physical wheel revolution (3+ repeats/side).
- [ ] Measure exact wheel diameter and tire-center track width.
- [ ] Validate distance with a ruler over a fixed floor path.
- [ ] Validate in-place heading estimation against a measured angle.
- [ ] Establish reliable ToF readings and stop behavior.

## Phase 2 — Controlled mobility ⏳
- [ ] Motor speed calibration at safe regulated VM voltage.
- [ ] Wheel speed feedback and closed-loop PID.
- [ ] Straight-line tracking and in-place turning.
- [ ] Motion fault detection (stalled wheel / missing encoder / watchdog).
- [ ] Controlled, supervised first floor drive.

## Phase 3 — Perception and local autonomy ⏳
- [ ] Add lateral obstacle sensing (ToF servo scan or other suitable sensor).
- [ ] Drop-off / edge detection and safer reverse coverage.
- [ ] Implement local obstacle avoidance with fail-closed behavior.
- [ ] Record sensor and odometry logs for tuning.

## Phase 4 — Mapping & autonomous navigation ⏳
- [ ] Choose an appropriate mapping sensor / localization approach.
- [ ] Occupancy map visualization and pose confidence reporting.
- [ ] Route planning and goal-directed navigation.
- [ ] Field testing and documented performance evaluation.

> **Engineering gate:** Do not enable autonomous floor exploration from the current single forward-facing ToF and unverified one-channel wheel odometry alone.

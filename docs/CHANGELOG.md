# Changelog

This log describes prototype development. Dates are approximate unless tied to a recorded release.

## [v4.3] — 2026-10 — Current experimental firmware
- Added experimental wheel-geometry settings (65 mm diameter, 118 mm track width, 20 pulses/rev **unverified**).
- Improved odometry bookkeeping to exclude raised-wheel test pulses from estimated chassis travel.
- Added explicit unvalidated-geometry telemetry status and excluded-pulse counters.
- Preserved the working IMU, ToF initialization, Wi-Fi and automated encoder test logic.

## [v4.2.3] — 2026-10 — Encoder repeatability
- Automated three consecutive 350 ms raised-wheel motor tests on each side.
- Exposed per-run pulse counts and running status in browser dashboard.
- Maintained short arming window, STOP/E-stop behavior and guarded test configuration.

## [v4.2.2] — 2026-10 — IMU decoder correction
- Fixed Arduino `word(...)` macro conflict by renaming the 16-bit decoder to `decodeS16`.
- Restored real MPU6500 acceleration, gyro and temperature readings.

## Earlier prototypes — v4.0–v4.2
- ESP32-S3 Wi-Fi access point and browser telemetry.
- MPU6500 and VL53L0X I²C integrations.
- Driver controls, encoder counters and initial odometry scaffolding.

#pragma once
// SELENE v4.3 - edit this file before first motor test.
// Firmware defaults are intentionally conservative.

// ESP32-S3 creates its own Wi-Fi AP. Change this password (8+ characters).
#define SELENE_AP_SSID "SELENE_ROVER"
#define SELENE_AP_PASSWORD "SELENE_Admin"

// Begin with 0. Only turn to 1 after motor supply / pin polarity checks,
// AND only for short tests with BOTH WHEELS OFF THE GROUND.
#define SELENE_ENABLE_MOTOR_TESTS 0

// Raised-wheel only: three short fixed-PWM encoder measurements per side.
// These are PULSES PER TIMED TEST, not pulses per physical wheel revolution.
// Keep SELENE_ENABLE_MOTOR_TESTS=0 until motor voltage and wiring are verified.
#define AUTO_ENCODER_RUNS 3
#define AUTO_ENCODER_PULSE_MS 350
#define AUTO_ENCODER_REST_MS 1000
#define AUTO_ENCODER_LEASE_MS 1200

// Experimental guarded straight-ahead forward crawl (NO automatic turning).
// KEEP 0 until LEFT/RIGHT direction, encoder motion and ToF failsafe work.
#define SELENE_ENABLE_GUARDED_CRAWL 0

// Set to 1 only AFTER physical motion direction/encoder calibration is verified.
#define SELENE_DRIVETRAIN_VERIFIED 0

// Measure these numbers on YOUR assembled rover before assigning values.
// A single encoder output gives pulse count, NOT physical direction.
// Encoder pulses/rev must be measured at the WHEEL, not assumed from the motor shaft.
// 0.0 means UNKNOWN: odometry stays unavailable, never invent defaults.
// Experimentally estimated pose is NOT reliable for autonomous exploration.
// v4.3 PROVISIONAL research estimates; NOT validated for navigation.
// Wheel diameter 65mm from matching TT chassis; wheel center track 118mm
// inferred from 143mm outside-to-outside and estimated 25mm tire width.
// 20 PPR assumes a 20-slot disc rotates once per wheel revolution and
// firmware counts ONE rising edge per slot; MUST verify physically.
#define SELENE_WHEEL_DIAMETER_MM 65.0f
#define SELENE_TRACK_WIDTH_MM 118.0f
#define SELENE_ENCODER_PULSES_PER_WHEEL_REV 20.0f

// GPIOs per prior Selene wiring. DO NOT apply 5V logic to any ESP32 GPIO.
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9
#define PIN_TRACKER 18
#define PIN_ENCODER_L 10
#define PIN_ENCODER_R 12
#define PIN_AIN1 4
#define PIN_AIN2 5
#define PIN_PWMA 6
#define PIN_BIN1 15
#define PIN_BIN2 16
#define PIN_PWMB 7
#define PIN_STBY 17

// The polarity may be reversed physically. Verify before using crawl mode.
#define LEFT_FORWARD_HIGH 1
#define RIGHT_FORWARD_HIGH 1

// No autonomous turning/mapping/SLAM in this version.
// TB6612 duty is NOT a regulated motor voltage. Confirm motor voltage rating.
#define TEST_PWM 65           // 0..255
#define GUARDED_PWM 55        // slow, tune AFTER testing
#define MOTOR_PULSE_MS 300    // test duration
#define GUARDED_MAX_MS 3000   // one supervised crawl interval maximum
#define STOP_DISTANCE_MM 400  // stop sooner if needed after measuring speed
#define REMOTE_LEASE_MS 1600  // stop crawl on loss of browser heartbeats

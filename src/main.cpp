/*
 * SELENE AUTONOMOUS FOUNDATION v4.2.3 AUTOMATED ENCODER PULSE TEST
 * PlatformIO / ESP32-S3 DevKitC-1 / Arduino-ESP32 2.x
 *
 * Battery-powered AP dashboard: connect to SELENE_ROVER, visit 192.168.4.1
 * Serial debug: UART0 over CH343 USB-UART bridge at 115200 (USB CDC OFF).
 *
 * Safety: motors OFF at boot, locked at build time by default, short arm-once
 * pulses, E-stop latched, browser lease + fresh ToF + timeout on optional
 * guarded straight crawl. v4.2: SAFE motor calibration and unverified encoder odometry.
 * NO automatic turning, NO mapping, NO SLAM.
 * IMU: uses the proven /api/imu_diag repeated-START burst transport in live telemetry.
 * Decoder renamed to decodeS16 to avoid Arduino.h word(...) macro collision.
 * Does not poll INT_STATUS immediately before the 14-byte data burst.
 *
 * SINGLE forward VL53L0X cannot sense edges, side/rear obstacles or drop-offs.
 * Operate only under supervision in a cleared, flat, controlled test area.
 * ALWAYS keep a physical power switch within reach.
 */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L0X.h>
#include "selene_config.h"
#include "dashboard_html.h"

static_assert(TEST_PWM <= 255 && GUARDED_PWM <= 255, "PWM duty must be 0..255");
static_assert(MOTOR_PULSE_MS <= 400, "motor test must be short");
static_assert(AUTO_ENCODER_RUNS == 3, "dashboard expects 3 calibration samples");
static_assert(AUTO_ENCODER_PULSE_MS >= 100 && AUTO_ENCODER_PULSE_MS <= 400, "auto motor pulse capped at 400 ms");
static_assert(AUTO_ENCODER_REST_MS >= 800, "cool down at least 800ms");
static_assert(AUTO_ENCODER_LEASE_MS >= 750 && AUTO_ENCODER_LEASE_MS <= 1500, "safe heartbeat bound");
static_assert(GUARDED_MAX_MS <= 3000, "guarded forward limit must be 3 seconds or less");
static_assert(SELENE_ENABLE_GUARDED_CRAWL == 0 || SELENE_ENABLE_MOTOR_TESTS == 1,
              "Verify motor tests first; enable motor tests before guarded crawl");

constexpr uint8_t PWM_CH_L = 0;
constexpr uint8_t PWM_CH_R = 1;
constexpr uint16_t PWM_FREQ = 10000;
constexpr uint8_t PWM_BITS = 8;
constexpr uint32_t SENSOR_PERIOD_MS = 150;
constexpr uint32_t SERIAL_PERIOD_MS = 500;
constexpr uint32_t ARM_WINDOW_MS = 8000;

Adafruit_MPU6050 imu; // Genuine MPU6050 (WHO_AM_I 0x68) only.
Adafruit_VL53L0X tof;

enum class ImuType : uint8_t { NONE, MPU6050, MPU6500 };
ImuType imuType = ImuType::NONE;
// MPU-6500 register-address definitions, per InvenSense register map.
constexpr uint8_t MPU_REG_SMPLRT_DIV = 0x19;
constexpr uint8_t MPU_REG_CONFIG = 0x1A;
constexpr uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t MPU_REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t MPU_REG_ACCEL_DATA = 0x3B; // 14 consecutive bytes: accel, temp, gyro
constexpr uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t MPU_REG_INT_STATUS = 0x3A;

WebServer web(80);

volatile uint32_t pulseLeft = 0;
volatile uint32_t pulseRight = 0;
void IRAM_ATTR leftIRQ() { ++pulseLeft; }
void IRAM_ATTR rightIRQ() { ++pulseRight; }

bool imuReady = false, tofReady = false, imuOk = false, tofOk = false;
bool i2cTofAck = false, i2cImu68Ack = false, i2cImu69Ack = false;
bool tofAckAfterImu = false;
int imuWhoAmI = -1;
bool stopLatched = false;
uint8_t imuAddr = 0;
int tofMm = -1, tofStatus = -1;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0, tempC=0;
uint32_t lastDistanceMs = 0;
uint32_t prevSense = 0, prevSerial = 0;
uint32_t prevRateTime = 0, lastL = 0, lastR = 0;
uint32_t encL = 0, encR = 0;
float rateL = 0, rateR = 0;
uint32_t armUntil = 0, motorUntil = 0, lastLeaseMs = 0;
uint32_t guardStartedMs = 0, guardInitialL=0, guardInitialR=0;
int motorL=0, motorR=0; // PWM as signed command; GPIO direction interpreted by configuration.
String modeName = "IDLE", lastEvent = "Boot complete. Motors locked by default.";
bool apStarted = false;

// v4.2: single-channel encoders count edges but DO NOT encode wheel direction.
// Signed displacement is inferred ONLY while a corresponding motor is commanded.
// Encoder pulses observed while motors are OFF are excluded from odometry.
// This is an experimental DEAD-RECKONING trace, not measured position or SLAM.
float poseXMm = 0.0f, poseYMm = 0.0f, poseThetaRad = 0.0f;
float distanceLeftMm = 0.0f, distanceRightMm = 0.0f;
uint32_t odomLastL = 0, odomLastR = 0;
uint32_t ignoredManualL = 0, ignoredManualR = 0;
// Raised-wheel motor tests produce shaft pulses but not chassis displacement.
uint32_t excludedLiftedTestL = 0, excludedLiftedTestR = 0;
uint32_t pulseStartL = 0, pulseStartR = 0;
uint32_t testDeltaL = 0, testDeltaR = 0;
uint32_t pulseEvents = 0;

// v4.2.3 raised-wheel encoder repeatability test.  This cannot determine
// pulses per true WHEEL revolution without an independent wheel index.
enum class AutoCalPhase : uint8_t { NONE, PULSE, REST };
AutoCalPhase autoPhase = AutoCalPhase::NONE;
uint8_t autoSide = 0; // 1=left, 2=right
uint8_t autoCompleted = 0;
uint32_t autoDeadline = 0, autoStartL=0, autoStartR=0;
uint32_t autoDeltaL[AUTO_ENCODER_RUNS] = {0};
uint32_t autoDeltaR[AUTO_ENCODER_RUNS] = {0};
uint32_t autoLastCompletedMs = 0;

float accelMagnitude = 0.0f;
bool imuGravityPlausible = false;
// IMU diagnostic data: a successful I2C read is not proof of valid measurements.
int imuPwr1=-1, imuPwr2=-1, imuAccelConfig=-1, imuGyroConfig=-1, imuIntStatus=-1;
int16_t rawAx=0,rawAy=0,rawAz=0,rawTemp=0,rawGx=0,rawGy=0,rawGz=0;
uint32_t imuEmptyFrames=0, imuImplausibleFrames=0, imuReadFailures=0, imuValidFrames=0;
const char* imuFault = "NOT_INITIALIZED";
// Last raw 14-byte motion frame for comparing /api/state against /api/imu_diag.
uint8_t lastImuMotionBytes[14] = {};
uint32_t lastImuFrameMs = 0;
uint32_t lastImuFrameNumber = 0;
bool lastImuFrameAvailable = false;

bool geometryConfigured() {
  return SELENE_WHEEL_DIAMETER_MM > 0.0f &&
         SELENE_TRACK_WIDTH_MM > 0.0f &&
         SELENE_ENCODER_PULSES_PER_WHEEL_REV > 0.0f;
}

void readEncoderCounts(uint32_t &l, uint32_t &r) {
  noInterrupts(); l = pulseLeft; r = pulseRight; interrupts();
}

void resetOdomAtCurrentEncoderCounts() {
  readEncoderCounts(odomLastL, odomLastR);
  poseXMm=0.0f; poseYMm=0.0f; poseThetaRad=0.0f;
  distanceLeftMm=0.0f; distanceRightMm=0.0f;
  ignoredManualL=0; ignoredManualR=0;
  excludedLiftedTestL=0; excludedLiftedTestR=0;
}

// Read counts at each change in drive state. Calling before disabling STBY
// credits the last motor-commanded encoder edges to the outgoing command.
void integrateOdom() {
  uint32_t l, r; readEncoderCounts(l,r);
  const uint32_t dl = l - odomLastL, dr = r - odomLastR;
  odomLastL = l; odomLastR = r;
  // All TEST and CALIBRATING commands require WHEELS_LIFTED.
  // Never represent rotation in the air as physical rover displacement.
  // Raw encoder totals / auto-test sample counts are preserved separately.
  if (modeName == "TEST" || modeName == "CALIBRATING") {
    excludedLiftedTestL += dl;
    excludedLiftedTestR += dr;
    return;
  }
  if (motorL == 0) ignoredManualL += dl;
  if (motorR == 0) ignoredManualR += dr;
  if (!geometryConfigured() || (dl==0 && dr==0)) return;
  const float mmPerPulse = 3.14159265358979323846f * SELENE_WHEEL_DIAMETER_MM /
                           SELENE_ENCODER_PULSES_PER_WHEEL_REV;
  const float dlmm = (motorL>0 ? 1.0f : motorL<0 ? -1.0f : 0.0f) * dl * mmPerPulse;
  const float drmm = (motorR>0 ? 1.0f : motorR<0 ? -1.0f : 0.0f) * dr * mmPerPulse;
  distanceLeftMm += dlmm;
  distanceRightMm += drmm;
  const float ds = 0.5f * (dlmm + drmm);
  const float dth = (drmm - dlmm) / SELENE_TRACK_WIDTH_MM;
  poseXMm += ds * cosf(poseThetaRad + dth * 0.5f);
  poseYMm += ds * sinf(poseThetaRad + dth * 0.5f);
  poseThetaRad = atan2f(sinf(poseThetaRad+dth), cosf(poseThetaRad+dth));
}


bool elapsed(uint32_t since, uint32_t interval) {
  return (uint32_t)(millis() - since) >= interval;
}

void event(const char *message) {
  lastEvent = message;
  Serial.print("[SELENE] "); Serial.println(message);
}

// Cancels the software state, NOT just the physical motor output.
void cancelAutoCalibration() {
  autoPhase = AutoCalPhase::NONE;
  autoDeadline = 0;
}

void driverOff() {
  // Disable bridge first; do not rely only on PWM=0.
  digitalWrite(PIN_STBY, LOW);
  integrateOdom(); // Capture edges from the outgoing command before clearing it.
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW);
  motorL = 0; motorR = 0; motorUntil = 0;
}

void idleMotors(const char *reason) {
  driverOff();
  cancelAutoCalibration();
  armUntil = 0;
  modeName = stopLatched ? "ESTOP" : "IDLE";
  if (reason) event(reason);
}

void hardStop(const char *reason) {
  stopLatched = true;
  driverOff();
  cancelAutoCalibration(); armUntil = 0;
  modeName = "ESTOP";
  event(reason);
}

void setDrive(int leftDuty, int rightDuty) {
  if (stopLatched) return;
  integrateOdom(); // Do not assign earlier edges to the next direction.
  leftDuty = constrain(leftDuty, -255, 255);
  rightDuty = constrain(rightDuty, -255, 255);
  // Set STBY low during every direction change to avoid shoot-through.
  digitalWrite(PIN_STBY, LOW);
  auto driveSide = [](int duty, uint8_t a, uint8_t b, uint8_t pwmChan, bool forwardHigh) {
    if (duty == 0) {
      digitalWrite(a, LOW); digitalWrite(b, LOW);
      ledcWrite(pwmChan, 0);
    } else {
      const bool dirA = (duty > 0) ? forwardHigh : !forwardHigh;
      digitalWrite(a, dirA ? HIGH : LOW);
      digitalWrite(b, dirA ? LOW : HIGH);
      ledcWrite(pwmChan, abs(duty));
    }
  };
  driveSide(leftDuty, PIN_AIN1, PIN_AIN2, PWM_CH_L, LEFT_FORWARD_HIGH != 0);
  driveSide(rightDuty, PIN_BIN1, PIN_BIN2, PWM_CH_R, RIGHT_FORWARD_HIGH != 0);
  motorL = leftDuty; motorR = rightDuty;
  digitalWrite(PIN_STBY, (leftDuty || rightDuty) ? HIGH : LOW);
}

// Nonblocking motor sequencer. The normal loop keeps serving Wi-Fi,
// IMU, ToF and emergency-stop requests during the test.
void startAutoPulse() {
  if (stopLatched || autoCompleted >= AUTO_ENCODER_RUNS) return;
  readEncoderCounts(autoStartL, autoStartR);
  setDrive(autoSide == 1 ? TEST_PWM : 0,
           autoSide == 2 ? TEST_PWM : 0);
  autoPhase = AutoCalPhase::PULSE;
  autoDeadline = millis() + AUTO_ENCODER_PULSE_MS;
}

void finishAutoPulse() {
  if (autoPhase != AutoCalPhase::PULSE) return;
  uint32_t l, r;
  readEncoderCounts(l, r);
  // Capture while the outgoing motor command is still active.
  const uint32_t dl = l - autoStartL;
  const uint32_t dr = r - autoStartR;
  driverOff();
  autoDeltaL[autoCompleted] = dl;
  autoDeltaR[autoCompleted] = dr;
  testDeltaL = dl;
  testDeltaR = dr;
  ++pulseEvents;
  ++autoCompleted;
  if (autoCompleted >= AUTO_ENCODER_RUNS) {
    cancelAutoCalibration();
    autoLastCompletedMs = millis();
    armUntil = 0;
    modeName = "IDLE";
    event("Raised-wheel 3-run test finished. Check counts and opposite encoder.");
  } else {
    autoPhase = AutoCalPhase::REST;
    autoDeadline = millis() + AUTO_ENCODER_REST_MS;
    event("Auto encoder test: motor OFF for cooldown before next run.");
  }
}

void advanceAutoCalibration() {
  if (modeName != "CALIBRATING" || autoPhase == AutoCalPhase::NONE) return;
  if (!SELENE_ENABLE_MOTOR_TESTS || WiFi.softAPgetStationNum() == 0 ||
      lastLeaseMs == 0 || elapsed(lastLeaseMs, AUTO_ENCODER_LEASE_MS)) {
    hardStop("FAIL SAFE: auto encoder calibration lost dashboard heartbeat.");
    return;
  }
  if (autoPhase == AutoCalPhase::PULSE) {
    uint32_t l, r;
    readEncoderCounts(l, r);
    // Implausibly many edges in one short pulse suggests a signal fault.
    if ((uint32_t)(l-autoStartL) > 15000u || (uint32_t)(r-autoStartR) > 15000u) {
      hardStop("FAIL SAFE: excessive LM393 edges in a short calibration pulse.");
      return;
    }
  }
  if (static_cast<int32_t>(millis()-autoDeadline) < 0) return;
  if (autoPhase == AutoCalPhase::PULSE) finishAutoPulse();
  else if (autoPhase == AutoCalPhase::REST) startAutoPulse();
}

bool i2cResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// MPU6050 identity register is 0x75; a genuine MPU6050 normally returns 0x68.
// This is independent of the Adafruit library's initialization result.
int readImuWhoAmI(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0x75);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// These register operations apply to WHO_AM_I=0x70 (MPU-6500 compatible).
// We do NOT change Adafruit's expected MPU6050 identity globally.
bool mpuWrite(uint8_t reg, uint8_t value) {
  if (!imuAddr) return false;
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t *buffer, size_t count) {
  if (!imuAddr || !buffer || count == 0 || count > 32) return false;
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddr, static_cast<uint8_t>(count)) != count) return false;
  for (size_t i = 0; i < count; ++i) {
    if (!Wire.available()) return false;
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

// Return -1 if the I2C register read fails; 0 is a valid register value.
int mpuRegister(uint8_t reg) {
  uint8_t v=0;
  return mpuReadBytes(reg,&v,1) ? int(v) : -1;
}

bool startMpu6500() {
  // Wake without a full hardware reset. The previous ToF-working firmware
  // used this sequence; preserve it while diagnosing bus interaction.
  imuFault = "INIT_FAILED";
  if (!mpuWrite(MPU_REG_PWR_MGMT_1, 0x01)) return false;
  delay(100);
  if (!mpuWrite(MPU_REG_PWR_MGMT_2, 0x00)) return false;
  if (!mpuWrite(MPU_REG_SMPLRT_DIV, 9)) return false;
  if (!mpuWrite(MPU_REG_CONFIG, 0x03)) return false;
  if (!mpuWrite(MPU_REG_GYRO_CONFIG, 0x08)) return false; // +/- 500 deg/s
  if (!mpuWrite(MPU_REG_ACCEL_CONFIG, 0x08)) return false; // +/- 4g
  if (!mpuWrite(MPU_REG_ACCEL_CONFIG2, 0x03)) return false;
  delay(50);
  imuPwr1 = mpuRegister(MPU_REG_PWR_MGMT_1);
  imuPwr2 = mpuRegister(MPU_REG_PWR_MGMT_2);
  imuAccelConfig = mpuRegister(MPU_REG_ACCEL_CONFIG);
  imuGyroConfig = mpuRegister(MPU_REG_GYRO_CONFIG);
  Serial.printf("[IMU CONFIG] PWR1=0x%02X PWR2=0x%02X ACC=0x%02X GYRO=0x%02X\n",
                imuPwr1,imuPwr2,imuAccelConfig,imuGyroConfig);
  if (imuPwr1 != 0x01 || imuPwr2 != 0x00 || imuAccelConfig != 0x08 || imuGyroConfig != 0x08) {
    imuFault = "REGISTER_READBACK_MISMATCH";
    return false;
  }
  uint8_t test[14];
  if (!mpuReadBytes(MPU_REG_ACCEL_DATA,test,sizeof(test))) {
    imuFault = "INITIAL_DATA_I2C_ERROR";
    return false;
  }
  imuFault = "AWAITING_VALID_SAMPLE";
  return true;
}

// The same transport helper used by the successful /api/imu_diag endpoint.
bool diagRead(uint8_t reg, uint8_t *dest, uint8_t amount, bool repeatedStart);

bool readMpu6500() {
  uint8_t d[14] = {};
  // Match the independently verified read-only diagnostic transaction.
  // Do not read INT_STATUS directly before fetching motion data.
  if (!diagRead(MPU_REG_ACCEL_DATA, d, 14, true)) {
    ++imuReadFailures;
    imuFault = "MOTION_BURST_I2C_ERROR";
    return false;
  }
  memcpy(lastImuMotionBytes, d, sizeof(d));
  lastImuFrameMs = millis();
  ++lastImuFrameNumber;
  lastImuFrameAvailable = true;
  auto decodeS16 = [&](uint8_t pos) -> int16_t {
    return static_cast<int16_t>((uint16_t(d[pos]) << 8) | d[pos+1]);
  };
  rawAx=decodeS16(0);rawAy=decodeS16(2);rawAz=decodeS16(4);rawTemp=decodeS16(6);
  rawGx=decodeS16(8);rawGy=decodeS16(10);rawGz=decodeS16(12);
  // An exact zero raw accel vector is implausible on a stationary rover;
  // v4.2 used to report these frames as READY with temp exactly 21.0C.
  // Fail the sample rather than present fabricated zero measurements.
  if (rawAx==0 && rawAy==0 && rawAz==0) {
    ++imuEmptyFrames;
    imuFault = "ZERO_ACCEL_DATA";
    return false;
  }
  constexpr float ms2PerCount = 9.80665f / 8192.0f;
  constexpr float radiansPerSecondPerCount = 0.01745329252f / 65.5f;
  ax = rawAx * ms2PerCount;
  ay = rawAy * ms2PerCount;
  az = rawAz * ms2PerCount;
  tempC = 21.0f + rawTemp / 333.87f;
  gx = rawGx * radiansPerSecondPerCount;
  gy = rawGy * radiansPerSecondPerCount;
  gz = rawGz * radiansPerSecondPerCount;
  // The test unit previously returned raw 0,2,4,... with near-zero gravity.
  // Successful Wire.requestFrom alone must not count as good IMU telemetry.
  // Keep raw values exposed for debugging but reject obviously invalid samples.
  const float norm = sqrtf(ax*ax + ay*ay + az*az);
  if (norm < 2.0f || norm > 30.0f) {
    ++imuImplausibleFrames;
    imuFault = "ACCEL_MAGNITUDE_IMPLAUSIBLE";
    return false;
  }
  ++imuValidFrames;
  imuFault = "OK";
  return true;
}

void discoverSensors() {
  driverOff(); armUntil=0; modeName=stopLatched?"ESTOP":"IDLE";
  tofReady = false; imuReady = false; tofOk = false; imuOk = false;
  imuType = ImuType::NONE;
  imuFault = "NOT_INITIALIZED";
  imuPwr1=-1;imuPwr2=-1;imuAccelConfig=-1;imuGyroConfig=-1;imuIntStatus=-1;
  imuAddr = 0; imuWhoAmI = -1;
  lastImuFrameAvailable = false; lastImuFrameMs = 0; lastImuFrameNumber = 0;
  i2cTofAck   = i2cResponds(0x29);
  i2cImu68Ack = i2cResponds(0x68);
  i2cImu69Ack = i2cResponds(0x69);
  Serial.printf("[I2C] 0x29(ToF)=%s  0x68(IMU)=%s  0x69(IMU)=%s\n",
                i2cTofAck?"ACK":"NO ACK", i2cImu68Ack?"ACK":"NO ACK",
                i2cImu69Ack?"ACK":"NO ACK");

  // Initialize the VL53L0X FIRST, before touching the IMU power registers.
  // This preserves the known-good ranging initialization and isolates startup.
  if (i2cTofAck) {
    tofReady = tof.begin(0x29, false, &Wire);
    Serial.printf("[TOF FIRST] 0x29 ACK=yes begin=%s\n",
                  tofReady ? "OK" : "FAILED");
  } else {
    Serial.println("[TOF FIRST] 0x29 NO ACK before IMU initialization");
  }

  if (i2cImu68Ack) imuAddr = 0x68;
  else if (i2cImu69Ack) imuAddr = 0x69;
  if (imuAddr) {
    imuWhoAmI = readImuWhoAmI(imuAddr);
    if (imuWhoAmI < 0) Serial.println("[I2C] IMU WHO_AM_I read failed");
    else Serial.printf("[I2C] IMU WHO_AM_I=0x%02X\n", imuWhoAmI);
    if (imuWhoAmI == 0x70) {
      imuType = ImuType::MPU6500;
      imuReady = startMpu6500();
      Serial.printf("[SELENE] MPU-6500-compatible IMU %s\n", imuReady ? "READY" : "INIT FAILED");
    } else if (imuWhoAmI == 0x68) {
      imuType = ImuType::MPU6050;
      imuReady = imu.begin(imuAddr, &Wire);
      if (imuReady) {
        imuFault="AWAITING_VALID_SAMPLE";
        imu.setAccelerometerRange(MPU6050_RANGE_4_G);
        imu.setGyroRange(MPU6050_RANGE_500_DEG);
        imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      }
    } else {
      Serial.println("[SELENE] Unsupported IMU identity. Leaving IMU offline.");
    }
  }
  // Check for an ACK again AFTER configuring the IMU.  These two snapshots
  // will tell us whether the ToF was missing from boot or disappeared later.
  tofAckAfterImu = i2cResponds(0x29);
  Serial.printf("[I2C ORDER CHECK] ToF before IMU=%d after IMU=%d initialized=%d\n",
                (int)i2cTofAck, (int)tofAckAfterImu, (int)tofReady);
  event("I2C startup check finished. Compare tof_ack_before_imu / tof_ack_after_imu.");
  Serial.printf("[SELENE] IMU=%s  VL53L0X=%s\n",imuReady?"READY":"NOT INITIALIZED",tofReady?"READY":"NOT INITIALIZED");
}

void updateSensors() {
  if (imuReady && imuType == ImuType::MPU6500) {
    imuOk = readMpu6500();
  } else if (imuReady && imuType == ImuType::MPU6050) {
    sensors_event_t a, g, t;
    imuOk = imu.getEvent(&a, &g, &t);
    if (imuOk) {
      imuFault="OK";
      ax = a.acceleration.x; ay = a.acceleration.y; az = a.acceleration.z;
      gx = g.gyro.x; gy = g.gyro.y; gz = g.gyro.z; tempC = t.temperature;
    } else imuFault="MPU6050_DRIVER_READ_ERROR";
  } else imuOk = false;
  // Gravity-magnitude sanity check is meaningful while rover is stationary.
  // It detects an IMU falsely reporting a complete zero vector despite READY.
  accelMagnitude = imuOk ? sqrtf(ax*ax + ay*ay + az*az) : 0.0f;
  imuGravityPlausible = imuOk && (accelMagnitude >= 6.0f && accelMagnitude <= 14.0f);
  if (tofReady) {
    VL53L0X_RangingMeasurementData_t d = {};
    VL53L0X_Error result=tof.rangingTest(&d,false);
    tofStatus=(int)d.RangeStatus;
    // Accept valid RangeStatus=0 only. Value is forward distance, not 2D map.
    tofOk=(result == VL53L0X_ERROR_NONE && d.RangeStatus==0 &&
           d.RangeMilliMeter >= 30 && d.RangeMilliMeter <= 2000);
    tofMm=tofOk?static_cast<int>(d.RangeMilliMeter):-1;
    if(tofOk)lastDistanceMs=millis();
  } else {tofOk=false;tofMm=-1;tofStatus=-1;}
  noInterrupts();uint32_t l=pulseLeft,r=pulseRight;interrupts();
  encL=l;encR=r;
  integrateOdom();
  uint32_t ms=millis();
  if(prevRateTime && elapsed(prevRateTime, 200)) {
    float seconds=(ms-prevRateTime)/1000.0f;
    rateL=(l-lastL)/seconds;rateR=(r-lastR)/seconds;
    prevRateTime=ms;lastL=l;lastR=r;
  } else if(!prevRateTime) {prevRateTime=ms;lastL=l;lastR=r;}
}

bool validFront() {
  return tofReady && tofOk && elapsed(lastDistanceMs, 300)==false && tofMm >= STOP_DISTANCE_MM;
}

bool calibrated() {
  return SELENE_DRIVETRAIN_VERIFIED && SELENE_WHEEL_DIAMETER_MM > 0 &&
         SELENE_TRACK_WIDTH_MM > 0 && SELENE_ENCODER_PULSES_PER_WHEEL_REV > 0;
}

bool guardedAllowed() {
  return SELENE_ENABLE_MOTOR_TESTS && SELENE_ENABLE_GUARDED_CRAWL && calibrated();
}

void checkMotionSafety() {
  advanceAutoCalibration();
  uint32_t now=millis();
  if (modeName == "TEST") {
    if (static_cast<int32_t>(now-motorUntil)>=0) {
      // Capture raw counts BEFORE the motor-control state is cleared.
      uint32_t l,r; readEncoderCounts(l,r);
      testDeltaL=l-pulseStartL; testDeltaR=r-pulseStartR;
      ++pulseEvents;
      idleMotors("Motor pulse finished; check last-test encoder deltas.");
    }
  } else if (modeName == "GUARDED") {
    if(elapsed(guardStartedMs, GUARDED_MAX_MS)) {
      idleMotors("Guarded interval completed; motors OFF.");
    } else if(elapsed(lastLeaseMs, REMOTE_LEASE_MS)) {
      hardStop("FAIL SAFE: browser heartbeat lost. E-stop latched.");
    } else if(!validFront()) {
      hardStop("FAIL SAFE: obstacle, ToF fault or stale measurement. E-stop latched.");
    } else if(elapsed(guardStartedMs, 900) && encL==guardInitialL && encR==guardInitialR) {
      hardStop("FAIL SAFE: no encoder pulses during forward crawl.");
    }
  }
}

String boolJson(bool b){return b?"true":"false";}
String makeState() {
  String s; s.reserve(1536);
  s="{\"type\":\"telemetry\",\"firmware\":\"selene-v4.3\"";
  s+=",\"ms\":"+String(millis());
  s+=",\"ip\":\""+WiFi.softAPIP().toString()+"\"";
  s+=",\"wifi_clients\":"+String(WiFi.softAPgetStationNum());
  s+=",\"mode\":\""+modeName+"\"";
  s+=",\"estop\":"+boolJson(stopLatched);
  s+=",\"motor_tests_enabled\":"+boolJson(SELENE_ENABLE_MOTOR_TESTS!=0);
  s+=",\"guard_enabled\":"+boolJson(guardedAllowed());
  s+=",\"calibrated\":"+boolJson(calibrated());
  s+=",\"armed\":"+boolJson(armUntil!=0&&static_cast<int32_t>(armUntil-millis())>0);
  s+=",\"tof_present\":"+boolJson(tofReady);
  s+=",\"tof_ok\":"+boolJson(tofOk && !elapsed(lastDistanceMs,300));
  s+=",\"tof_mm\":"+String(tofMm);
  s+=",\"tof_status\":"+String(tofStatus);
  s+=",\"i2c_29\":"+boolJson(i2cTofAck);
  s+=",\"tof_ack_before_imu\":"+boolJson(i2cTofAck);
  s+=",\"tof_ack_after_imu\":"+boolJson(tofAckAfterImu);
  s+=",\"i2c_68\":"+boolJson(i2cImu68Ack);
  s+=",\"i2c_69\":"+boolJson(i2cImu69Ack);
  s+=",\"imu_whoami\":"+String(imuWhoAmI);
  s+=",\"imu_model\":\""+String(imuType == ImuType::MPU6500 ? "MPU6500" : imuType == ImuType::MPU6050 ? "MPU6050" : "UNKNOWN")+"\"";
  s+=",\"imu_fault\":\""+String(imuFault)+"\"";
  s+=",\"imu_pwr1\":"+String(imuPwr1);
  s+=",\"imu_pwr2\":"+String(imuPwr2);
  s+=",\"imu_accel_config\":"+String(imuAccelConfig);
  s+=",\"imu_gyro_config\":"+String(imuGyroConfig);
  s+=",\"imu_int_status\":"+String(imuIntStatus);
  s+=",\"imu_raw_ax\":"+String(rawAx)+",\"imu_raw_ay\":"+String(rawAy)+",\"imu_raw_az\":"+String(rawAz);
  s+=",\"imu_raw_temp\":"+String(rawTemp);
  s+=",\"imu_raw_gx\":"+String(rawGx)+",\"imu_raw_gy\":"+String(rawGy)+",\"imu_raw_gz\":"+String(rawGz);
  s+=",\"imu_implausible_frames\":"+String(imuImplausibleFrames);
  s+=",\"imu_zero_frames\":"+String(imuEmptyFrames);
  s+=",\"imu_i2c_errors\":"+String(imuReadFailures);
  s+=",\"imu_valid_frames\":"+String(imuValidFrames);
  s+=",\"imu_transport\":\"BURST_RESTART\"";
  s+=",\"imu_frame_count\":"+String(lastImuFrameNumber);
  s+=",\"imu_frame_age_ms\":"+String(lastImuFrameAvailable ? (uint32_t)(millis()-lastImuFrameMs) : 0);
  s+=",\"imu_frame_hex\":\"";
  if (lastImuFrameAvailable) {
    const char digits[] = "0123456789ABCDEF";
    for (uint8_t i=0; i<14; i++) {
      uint8_t b=lastImuMotionBytes[i];
      if (i) s+=' ';
      s+=digits[b >> 4];
      s+=digits[b & 0x0F];
    }
  }
  s+="\"";
  s+=",\"mpu_present\":"+boolJson(imuReady);
  s+=",\"imu_ok\":"+boolJson(imuOk);
  s+=",\"ax\":"+String(ax,3)+",\"ay\":"+String(ay,3)+",\"az\":"+String(az,3);
  s+=",\"gx\":"+String(gx,3)+",\"gy\":"+String(gy,3)+",\"gz\":"+String(gz,3);
  s+=",\"temp_c\":"+String(tempC,2);
  s+=",\"accel_magnitude\":"+String(accelMagnitude,2);
  s+=",\"imu_gravity_plausible\":"+boolJson(imuGravityPlausible);
  s+=",\"tracker\":"+String(digitalRead(PIN_TRACKER));
  s+=",\"enc_l\":"+String(encL)+",\"enc_r\":"+String(encR);
  s+=",\"left_rate\":"+String(rateL,1)+",\"right_rate\":"+String(rateR,1);
  s+=",\"auto_cal_side\":\""+String(autoSide == 1 ? "LEFT" : autoSide == 2 ? "RIGHT" : "NONE")+"\"";
  s+=",\"auto_cal_active\":"+boolJson(modeName == "CALIBRATING");
  s+=",\"auto_cal_stage\":\""+String(autoPhase == AutoCalPhase::PULSE ? "SPIN" : autoPhase == AutoCalPhase::REST ? "COOLDOWN" : "IDLE")+"\"";
  s+=",\"auto_cal_done\":"+String(autoCompleted);
  s+=",\"auto_cal_last_ms\":"+String(autoLastCompletedMs);
  s+=",\"auto_cal_left_pulses\":[";
  for (uint8_t i=0; i<AUTO_ENCODER_RUNS; ++i) {if(i) s+=","; s+=String(autoDeltaL[i]);}
  s+="]";
  s+=",\"auto_cal_right_pulses\":[";
  for (uint8_t i=0; i<AUTO_ENCODER_RUNS; ++i) {if(i) s+=","; s+=String(autoDeltaR[i]);}
  s+="]";
  s+=",\"last_test_left_pulses\":"+String(testDeltaL);
  s+=",\"last_test_right_pulses\":"+String(testDeltaR);
  s+=",\"test_count\":"+String(pulseEvents);
  s+=",\"odom_configured\":"+boolJson(geometryConfigured());
  s+=",\"odom_direction_source\":\"MOTOR_COMMAND_INFERRED\"";
  s+=",\"odom_quality\":\"PROVISIONAL_GEOMETRY_NOT_VALIDATED_NO_SLAM\"";
  s+=",\"odom_x_mm\":"+String(poseXMm,1);
  s+=",\"odom_y_mm\":"+String(poseYMm,1);
  s+=",\"odom_theta_rad\":"+String(poseThetaRad,4);
  s+=",\"odom_left_mm\":"+String(distanceLeftMm,1);
  s+=",\"odom_right_mm\":"+String(distanceRightMm,1);
  s+=",\"ignored_manual_left\":"+String(ignoredManualL);
  s+=",\"ignored_manual_right\":"+String(ignoredManualR);
  s+=",\"excluded_lifted_test_left\":"+String(excludedLiftedTestL);
  s+=",\"excluded_lifted_test_right\":"+String(excludedLiftedTestR);
  s+=",\"geometry_verified\":false";
  s+=",\"wheel_diameter_mm\":"+String(SELENE_WHEEL_DIAMETER_MM,2);
  s+=",\"track_width_mm\":"+String(SELENE_TRACK_WIDTH_MM,2);
  s+=",\"encoder_ppr\":"+String(SELENE_ENCODER_PULSES_PER_WHEEL_REV,2);
  s+=",\"motor_left\":"+String(motorL)+",\"motor_right\":"+String(motorR);
  s+=",\"motor_pwm\":"+String(abs(motorL) > abs(motorR) ? abs(motorL) : abs(motorR));
  s+=",\"last_event\":\""+lastEvent+"\"";
  s+="}";
  return s;
}

void sendResponse(bool success, const char *message) {
  String j="{\"ok\":"+boolJson(success)+",\"message\":\""+String(message)+"\"}";
  web.send(success?200:409,"application/json",j);
}

void handleCommand(const String &cmd,const String &ack) {
  if(cmd=="stop") {hardStop("Remote STOP; E-stop latched.");sendResponse(true,"E-stop latched; motors OFF");return;}
  if(cmd=="reset_estop") {
    driverOff();armUntil=0;stopLatched=false;modeName="IDLE";
    event("E-stop explicitly cleared; motors remain OFF.");sendResponse(true,"E-stop cleared; motors OFF");return;
  }
  if(stopLatched) {sendResponse(false,"E-stop latched. Clear before tests.");return;}
  if(modeName=="GUARDED" || modeName=="TEST" || modeName=="CALIBRATING") {
    sendResponse(false,"Motor operation active; press STOP or wait for auto-off.");return;
  }
  if(cmd=="scan") {discoverSensors();sendResponse(true,"I2C scan completed");return;}
  if(cmd=="reset_odom") {
    if (modeName!="IDLE" || motorL!=0 || motorR!=0) {
      sendResponse(false,"Reset odometry only while motors are idle.");return;
    }
    resetOdomAtCurrentEncoderCounts();
    event("Experimental odometry origin reset; raw encoder totals unchanged.");
    sendResponse(true,"Odometry reset; motor remains OFF");return;
  }
  if(cmd=="arm") {
    if(!SELENE_ENABLE_MOTOR_TESTS || ack!="WHEELS_LIFTED") {
      sendResponse(false,"Tests locked. Verify wiring, edit config and lift wheels.");return;
    }
    driverOff();armUntil=millis()+ARM_WINDOW_MS;modeName="ARMED";
    event("Wheel-lifted motor test armed for 8 seconds.");sendResponse(true,"Armed for one 300ms pulse");return;
  }
  if (cmd=="auto_left" || cmd=="auto_right") {
    // Arm once, explicitly acknowledge the lifted wheels, then run 3 pulses.
    // Browser must continue sending /api/ping or the hardware E-stop latches.
    if (!SELENE_ENABLE_MOTOR_TESTS || ack!="WHEELS_LIFTED" ||
        WiFi.softAPgetStationNum()==0 || lastLeaseMs==0 ||
        elapsed(lastLeaseMs, AUTO_ENCODER_LEASE_MS) ||
        armUntil==0 || static_cast<int32_t>(armUntil-millis())<=0 ||
        modeName!="ARMED") {
      sendResponse(false,"Auto test rejected: arm, lift wheels and keep browser connected.");return;
    }
    armUntil=0;
    autoSide=(cmd=="auto_left")?1:2;
    autoCompleted=0;
    for (uint8_t i=0; i<AUTO_ENCODER_RUNS; ++i) {autoDeltaL[i]=0;autoDeltaR[i]=0;}
    modeName="CALIBRATING";
    startAutoPulse();
    event("Raised-wheel encoder auto test: three 350ms pulses, auto-off between runs.");
    sendResponse(true,"Three supervised motor pulses started; watch counts and STOP button.");
    return;
  }
  if(cmd=="pulse_left"||cmd=="pulse_right"||cmd=="pulse_both") {
    if(!SELENE_ENABLE_MOTOR_TESTS || ack!="WHEELS_LIFTED" ||
       armUntil==0 || static_cast<int32_t>(armUntil-millis())<=0 || modeName!="ARMED") {
      sendResponse(false,"Pulse rejected: arm again with wheels lifted.");return;
    }
    armUntil=0;
    int l=(cmd=="pulse_right")?0:TEST_PWM;
    int r=(cmd=="pulse_left")?0:TEST_PWM;
    readEncoderCounts(pulseStartL,pulseStartR);
    testDeltaL=0;testDeltaR=0;
    setDrive(l,r);motorUntil=millis()+MOTOR_PULSE_MS;modeName="TEST";
    event("Short motor pulse started, auto-off in 300ms.");sendResponse(true,"One pulse commanded");return;
  }
  if(cmd=="guard") {
    if(!guardedAllowed()||ack!="FLOOR_CLEAR"||modeName=="TEST"||!validFront()||
       elapsed(lastLeaseMs,REMOTE_LEASE_MS)) {
      sendResponse(false,"Crawl locked; require verification, valid ToF and live browser lease.");return;
    }
    armUntil=0;
    guardStartedMs=millis(); guardInitialL=encL;guardInitialR=encR;
    setDrive(GUARDED_PWM,GUARDED_PWM);modeName="GUARDED";
    event("GUARDED forward only: 3s limit, ToF and wireless watchdog.");
    sendResponse(true,"Guarded forward started, max 3 seconds");return;
  }
  sendResponse(false,"Unknown command");
}


// READ-ONLY MPU6500 register-transport diagnostic.
// Does not reset either sensor, does not reinitialize I2C, never drives motors.
// Each snapshot reads registers 0x3B..0x48 in a different transaction pattern.
bool diagRead(uint8_t reg, uint8_t *dest, uint8_t amount, bool repeatedStart) {
  if (!imuAddr || !dest || amount == 0 || amount > 14) return false;
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(!repeatedStart) != 0) return false;
  if (Wire.requestFrom(imuAddr, amount) != amount) return false;
  for (uint8_t i = 0; i < amount; ++i) {
    if (!Wire.available()) return false;
    dest[i] = (uint8_t)Wire.read();
  }
  return true;
}

String diagHex(uint8_t start, uint8_t len, bool repeatedStart, bool separateRegisters) {
  String result;
  result.reserve(len * 3 + 16);
  uint8_t bytes[14] = {};
  if (separateRegisters) {
    for (uint8_t i = 0; i < len; ++i) {
      if (!diagRead(start + i, &bytes[i], 1, repeatedStart)) {
        return String("ERROR_AT_0x") + String(start+i, HEX);
      }
    }
  } else if (!diagRead(start, bytes, len, repeatedStart)) {
    return "READ_ERROR";
  }
  const char digits[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < len; ++i) {
    if (i) result += ' ';
    result += digits[bytes[i] >> 4];
    result += digits[bytes[i] & 0x0F];
  }
  return result;
}

void handleImuDiagnostic() {
  // Protect the real-time control loop: this endpoint is read-only and idle-only.
  if (modeName != "IDLE" && modeName != "ESTOP") {
    web.send(409, "application/json", "{\"error\":\"Stop motor tests before IMU diagnosis\"}");
    return;
  }
  if (imuAddr == 0) {
    web.send(409, "application/json", "{\"error\":\"No IMU I2C address detected\"}");
    return;
  }
  String result;
  result.reserve(550);
  result = "{\"imu_addr\":\"0x" + String(imuAddr, HEX) + "\"";
  result += ",\"whoami\":" + String(mpuRegister(0x75));
  result += ",\"pwr1\":" + String(mpuRegister(0x6B));
  result += ",\"config_accel\":" + String(mpuRegister(0x1C));
  result += ",\"burst_restart\":\"" + diagHex(0x3B, 14, true, false) + "\"";
  result += ",\"burst_stop\":\"" + diagHex(0x3B, 14, false, false) + "\"";
  result += ",\"single_restart\":\"" + diagHex(0x3B, 14, true, true) + "\"";
  result += ",\"single_stop\":\"" + diagHex(0x3B, 14, false, true) + "\"";
  result += "}";
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json", result);
}

void initializeWeb() {
  web.on("/", HTTP_GET, [](){web.sendHeader("Cache-Control","no-store");web.send_P(200,"text/html; charset=utf-8",SELENE_PAGE);});
  web.on("/api/state", HTTP_GET, [](){web.sendHeader("Cache-Control","no-store");web.send(200,"application/json",makeState());});
  web.on("/api/imu_diag", HTTP_GET, handleImuDiagnostic);
  web.on("/api/ping", HTTP_POST, [](){lastLeaseMs=millis();web.send(200,"application/json","{\"ok\":true}");});
  web.on("/api/command",HTTP_POST, [](){handleCommand(web.arg("cmd"),web.arg("ack"));});
  web.onNotFound([](){web.send(404,"text/plain","Not found. Try / or /api/state");});
  web.begin();
}

void setupMotorPins() {
  // Control lines LOW before exposing any motor power.
  digitalWrite(PIN_STBY,LOW);
  pinMode(PIN_STBY,OUTPUT);
  const uint8_t directionPins[] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};
  for (uint8_t p : directionPins) {
    digitalWrite(p, LOW); pinMode(p, OUTPUT);
  }
  ledcSetup(PWM_CH_L,PWM_FREQ,PWM_BITS);
  ledcSetup(PWM_CH_R,PWM_FREQ,PWM_BITS);
  ledcAttachPin(PIN_PWMA,PWM_CH_L);
  ledcAttachPin(PIN_PWMB,PWM_CH_R);
  driverOff();
}

void pollSerial() {
  // Emergency stop on serial UART0. Never start motors from an open serial port.
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='X'||c=='x'||c=='!') hardStop("UART emergency STOP; E-stop latched.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[SELENE] v4.3 starting -- CH343 USB-UART; USB CDC off");
  setupMotorPins();
  event("TB6612 driver in standby, motion locked by configuration.");
  pinMode(PIN_TRACKER, INPUT); // digital level only; floating input if disconnected
  pinMode(PIN_ENCODER_L, INPUT_PULLUP);
  pinMode(PIN_ENCODER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L),leftIRQ,RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R),rightIRQ,RISING);
  resetOdomAtCurrentEncoderCounts();
  WiFi.mode(WIFI_AP);
  apStarted=WiFi.softAP(SELENE_AP_SSID,SELENE_AP_PASSWORD);
  if(apStarted) {
    Serial.print("[SELENE] Wi-Fi AP ");Serial.print(SELENE_AP_SSID);
    Serial.print(" => http://");Serial.println(WiFi.softAPIP().toString());
    initializeWeb();
  } else event("Wi-Fi AP FAILED; use UART diagnostics.");
  Wire.begin(PIN_I2C_SDA,PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(70);
  delay(150); // Allow sensor modules to finish booting after power application.
  discoverSensors();
  Serial.println("[SELENE] Ready. v4.2.3: three-run raised-wheel encoder tests locked by default.");
}

void loop() {
  checkMotionSafety();
  pollSerial();
  if(apStarted) web.handleClient();
  if(elapsed(prevSense,SENSOR_PERIOD_MS)) {
    prevSense=millis();updateSensors();checkMotionSafety();
  }
  // Arming expires independently of browser and serial availability.
  if(modeName=="ARMED"&&armUntil&&static_cast<int32_t>(millis()-armUntil)>=0) {
    idleMotors("Motor arm expired without test pulse.");
  }
  if(elapsed(prevSerial,SERIAL_PERIOD_MS)) {
    prevSerial=millis();Serial.println(makeState());
  }
  delay(2);
}

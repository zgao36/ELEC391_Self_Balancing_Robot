#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <ctype.h>
#include <string.h>

class FastImuClass : public BoschSensorClass {
public:
  FastImuClass(TwoWire& wire) : BoschSensorClass(wire) {}

  bool readAccelerationAndGyroscope(float& accelX, float& accelY, float& accelZ,
                                    float& gyroX, float& gyroY, float& gyroZ) {
    if (bmi2Dev == nullptr) {
      return false;
    }

    struct bmi2_sens_data sensorData = {};
    int8_t result = bmi2_get_sensor_data(&sensorData, bmi2Dev);
    if (result != BMI2_OK) {
      return false;
    }

    const float int16ToG = 8192.0;
    const float int16ToDps = 16.384;

    #ifdef TARGET_ARDUINO_NANO33BLE
    accelX = -sensorData.acc.y / int16ToG;
    accelY = -sensorData.acc.x / int16ToG;
    gyroX = -sensorData.gyr.y / int16ToDps;
    gyroY = -sensorData.gyr.x / int16ToDps;
    #else
    accelX = sensorData.acc.x / int16ToG;
    accelY = sensorData.acc.y / int16ToG;
    gyroX = sensorData.gyr.x / int16ToDps;
    gyroY = sensorData.gyr.y / int16ToDps;
    #endif

    accelZ = sensorData.acc.z / int16ToG;
    gyroZ = sensorData.gyr.z / int16ToDps;
    return true;
  }

protected:
  int8_t configure_sensor(struct bmi2_dev *dev) override {
    bmi2Dev = dev;

    int8_t result;
    uint8_t sensorList[2] = { BMI2_ACCEL, BMI2_GYRO };

    struct bmi2_int_pin_config intPinConfig = {};
    intPinConfig.pin_type = BMI2_INT1;
    intPinConfig.int_latch = BMI2_INT_NON_LATCH;
    intPinConfig.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    intPinConfig.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    intPinConfig.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    intPinConfig.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;

    struct bmi2_sens_config sensorConfig[2] = {};
    sensorConfig[0].type = BMI2_ACCEL;
    sensorConfig[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    sensorConfig[0].cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    sensorConfig[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    sensorConfig[0].cfg.acc.range = BMI2_ACC_RANGE_4G;

    sensorConfig[1].type = BMI2_GYRO;
    sensorConfig[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    sensorConfig[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
    sensorConfig[1].cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    sensorConfig[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    sensorConfig[1].cfg.gyr.ois_range = BMI2_GYR_OIS_2000;

    result = bmi2_set_int_pin_config(&intPinConfig, dev);
    if (result != BMI2_OK) return result;

    result = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev);
    if (result != BMI2_OK) return result;

    result = bmi2_set_sensor_config(sensorConfig, 2, dev);
    if (result != BMI2_OK) return result;

    result = bmi2_sensor_enable(sensorList, 2, dev);
    if (result != BMI2_OK) return result;

    return result;
  }

private:
  struct bmi2_dev *bmi2Dev = nullptr;
};

FastImuClass FastIMU(Wire1);
#undef IMU
#define IMU FastIMU

// ---------------- BLE command interface ----------------
#define BLE_BUFFER_SIZE 20

BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC",
  BLERead | BLEWrite | BLENotify,
  BLE_BUFFER_SIZE,
  false
);

const char BLE_LOCAL_NAME[] = "BLE-DEVICE111";
const bool BLE_ENABLED = true;
const uint16_t BLE_CONTROL_LOOP_POLL_INTERVAL = 20;

const float BLE_MOVE_TARGET_COUNTS = 3820.0;

bool bleReady = false;
bool bleCentralConnected = false;
uint16_t controlLoopsSinceBlePoll = 0;

// ---------------- Motor pins ----------------
const uint8_t LEFT_IN1 = 9;
const uint8_t LEFT_IN2 = 10;
const uint8_t RIGHT_IN1 = 4;
const uint8_t RIGHT_IN2 = 6;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long IMU_I2C_CLOCK_HZ = 1000000;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ---------------- Encoder pins ----------------
const uint8_t LEFT_ENCODER_A = 11;
const uint8_t LEFT_ENCODER_B = 12;
const uint8_t RIGHT_ENCODER_A = 7;
const uint8_t RIGHT_ENCODER_B = 8;

const long LEFT_ENCODER_CPR = 1920;

const int LEFT_ENCODER_SIGN = -1;
const int RIGHT_ENCODER_SIGN = 1;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

// ---------------- Motor deadband ----------------
int leftDeadband = 41;
int rightDeadband = 42;
const int RIGHT_BACKWARD_PWM_COMPENSATION = 0;

// ---------------- IMU filter ----------------
const int GYRO_CALIBRATION_SAMPLES = 500;
const float GYRO_CALIBRATION_ACCEL_MAG_TOLERANCE_G = 0.12;
const float GYRO_CALIBRATION_MAX_STILL_DPS = 5.0;
const float FILTER_K = 0.98;

// ---------------- Upright inner-loop PID ----------------
const float BASE_TARGET_ANGLE_DEG = -0.5;
const float MAX_SAFE_ANGLE_DEG = 35.0;
const float SAFETY_RESTART_ANGLE_DEG = 4.0;
const float START_ANGLE_WINDOW_DEG = 1.0;
const float START_GYRO_WINDOW_DPS = 25.0;
const unsigned long ARM_STABLE_TIME_MS = 300;
const float BALANCE_I_OUTPUT_LIMIT_PWM = 255.0;

float balanceKp = 8.5;
float balanceKi = 65.0;
float balanceKd = 0.5;

const int MOTOR_DIRECTION_SIGN = -1;

// ---------------- Position outer-loop PID ----------------
const unsigned long POSITION_LOOP_INTERVAL_MS = 50;
const float MAX_POSITION_ANGLE_OFFSET_DEG = 3.0;
const float POSITION_INTEGRAL_LIMIT = 2000.0;
const bool BLE_POSITION_TARGET_RAMP_ENABLED = true;
const float BLE_POSITION_TARGET_RAMP_SPEED_CM_PER_SEC = 12.0;
const float WHEEL_DIAMETER_CM = 8.0;
const float ENCODER_COUNTS_PER_CM =
  LEFT_ENCODER_CPR / (PI * WHEEL_DIAMETER_CM);
const float BLE_POSITION_TARGET_RAMP_COUNTS_PER_SEC =
  BLE_POSITION_TARGET_RAMP_SPEED_CM_PER_SEC * ENCODER_COUNTS_PER_CM;
const int POSITION_LOOP_DIRECTION_SIGN = 1;

float targetPositionCounts = 0.0;
float bleCommandedPositionTargetCounts = 0.0;
bool blePositionTargetRampActive = false;
float positionKp = 0.0016;
float positionKi = 0.0000;
float positionKd = 0.0007;

// ---------------- Turning loop ----------------
const bool TURNING_LOOP_ENABLED = true;
const float BLE_TURN_TARGET_DEG = 45.0;
float turningKp = 0.4;
const float TURNING_DEADBAND_DEG = 1.0;
const float MAX_TURNING_PWM = 35.0;
const int TURNING_LOOP_DIRECTION_SIGN = 1;

// ---------------- Ramp sequence ----------------
enum RampSequenceState {
  RAMP_SEQUENCE_IDLE,
  RAMP_SEQUENCE_ASCEND,
  RAMP_SEQUENCE_HOLD_TOP,
  RAMP_SEQUENCE_DESCEND
};

RampSequenceState rampSequenceState = RAMP_SEQUENCE_IDLE;
const float RAMP_TRAVEL_COUNTS = 4000.0;
const unsigned long RAMP_HOLD_TIME_MS = 30000;
const float RAMP_POSITION_TOLERANCE_COUNTS = 230.0;
float rampStartPositionCounts = 0.0;
unsigned long rampHoldStartMs = 0;

const unsigned long IMU_SAMPLE_TIMEOUT_MS = 100;
const unsigned long CONTROL_LOOP_IDLE_DELAY_MS = 0;
const unsigned long CONTROL_LOOP_TARGET_PERIOD_US = 5556;
const unsigned long DEBUG_PRINT_INTERVAL_MS = 50;
const bool DEBUG_PRINT_ENABLED = false;
const bool POSITION_PLOTTER_TELEMETRY_ENABLED = false;
const bool CONTROL_FREQ_PRINT_ENABLED = true;
const unsigned long CONTROL_FREQ_PRINT_INTERVAL_MS = 1000;

// ---------------- Runtime state ----------------
float balanceIntegralError = 0.0;
bool safetyStopActive = false;
bool robotArmed = false;
unsigned long stableStartMs = 0;

float measuredPositionCounts = 0.0;
float lastMeasuredPositionCounts = 0.0;
float measuredPositionVelocityCountsPerSec = 0.0;
float positionErrorCounts = 0.0;
float positionIntegralError = 0.0;
float positionPAngleDeg = 0.0;
float positionIAngleDeg = 0.0;
float positionDAngleDeg = 0.0;
float positionAngleOffsetDeg = 0.0;
unsigned long lastPositionSampleMs = 0;

float yawRateDps = 0.0;
float yawAngleDeg = 0.0;
float yawTargetAngleDeg = 0.0;
float turningErrorDeg = 0.0;
float turningPwm = 0.0;

float ax, ay, az;
float gx, gy, gz;

float thetaA = 0.0;
float thetaN = 0.0;
float thetaOffset = 0.0;
float gyroOffset = 0.0;
float yawGyroOffset = 0.0;

unsigned long lastImuTime = 0;
unsigned long lastTiltSampleMs = 0;
bool imuTimeoutStopActive = false;
unsigned long controlFreqWindowStartMs = 0;
unsigned long controlFreqSampleCount = 0;
float controlFreqMaxDtMs = 0.0;
unsigned long blePollCount = 0;
unsigned long blePollMaxDurationUs = 0;
unsigned long nextControlLoopStartUs = 0;

// ---------------- Small helpers ----------------
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

float wrapAngleDeg(float angleDeg) {
  while (angleDeg > 180.0) {
    angleDeg -= 360.0;
  }

  while (angleDeg < -180.0) {
    angleDeg += 360.0;
  }

  return angleDeg;
}

int clampPwm(int pwm) {
  if (pwm < PWM_MIN) return PWM_MIN;
  if (pwm > PWM_MAX) return PWM_MAX;
  return pwm;
}

// ---------------- Motor functions ----------------
void driveForward(uint8_t in1, uint8_t in2, int pwm) {
  analogWrite(in1, clampPwm(pwm));
  digitalWrite(in2, LOW);
}

void driveBackward(uint8_t in1, uint8_t in2, int pwm) {
  digitalWrite(in1, LOW);
  analogWrite(in2, clampPwm(pwm));
}

void driveSigned(uint8_t in1, uint8_t in2, int command) {
  if (command > 0) {
    driveForward(in1, in2, command);
  } else if (command < 0) {
    driveBackward(in1, in2, -command);
  } else {
    coastMotor(in1, in2);
  }
}

void coastMotor(uint8_t in1, uint8_t in2) {
  analogWrite(in1, 0);
  analogWrite(in2, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void coastAllMotors() {
  coastMotor(LEFT_IN1, LEFT_IN2);
  coastMotor(RIGHT_IN1, RIGHT_IN2);
}

int addDeadband(int command, int deadband) {
  if (command == 0) return 0;

  int sign = command > 0 ? 1 : -1;
  int magnitude = abs(command);
  if (magnitude < 3) return 0;

  magnitude = clampPwm(magnitude);

  int compensated =
    deadband
    + (long)magnitude
    * (PWM_MAX - deadband)
    / PWM_MAX;

  compensated = clampPwm(compensated);

  return sign * compensated;
}

int applyRightBackwardPwmCompensation(int command) {
  if (command >= 0) return command;

  int magnitude = abs(command);
  magnitude = clampPwm(magnitude + RIGHT_BACKWARD_PWM_COMPENSATION);
  return -magnitude;
}

// ---------------- Encoder functions ----------------
uint8_t readEncoderState(uint8_t pinA, uint8_t pinB) {
  return (digitalRead(pinA) << 1) | digitalRead(pinB);
}

int encoderStep(uint8_t oldState, uint8_t newState) {
  if (oldState == newState) return 0;

  if ((oldState == 0 && newState == 2) ||
      (oldState == 2 && newState == 3) ||
      (oldState == 3 && newState == 1) ||
      (oldState == 1 && newState == 0)) {
    return 1;
  }

  if ((oldState == 0 && newState == 1) ||
      (oldState == 1 && newState == 3) ||
      (oldState == 3 && newState == 2) ||
      (oldState == 2 && newState == 0)) {
    return -1;
  }

  return 0;
}

void updateEncoderCount(volatile long *encoderCount, volatile uint8_t *encoderState,
                        uint8_t pinA, uint8_t pinB, int encoderSign) {
  uint8_t currentState = readEncoderState(pinA, pinB);
  *encoderCount += encoderSign * encoderStep(*encoderState, currentState);
  *encoderState = currentState;
}

void updateLeftEncoder() {
  updateEncoderCount(&leftEncoderCount, &leftEncoderState,
                     LEFT_ENCODER_A, LEFT_ENCODER_B, LEFT_ENCODER_SIGN);
}

void updateRightEncoder() {
  updateEncoderCount(&rightEncoderCount, &rightEncoderState,
                     RIGHT_ENCODER_A, RIGHT_ENCODER_B, RIGHT_ENCODER_SIGN);
}

void readEncoderCounts(long *leftCount, long *rightCount) {
  noInterrupts();
  *leftCount = leftEncoderCount;
  *rightCount = rightEncoderCount;
  interrupts();
}

void resetEncoderCounts() {
  noInterrupts();
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  leftEncoderState = readEncoderState(LEFT_ENCODER_A, LEFT_ENCODER_B);
  rightEncoderState = readEncoderState(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
  interrupts();
}

void setupEncoders() {
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  resetEncoderCounts();

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), updateRightEncoder, CHANGE);
}

float averageEncoderCounts(long leftCount, long rightCount) {
  return ((float)leftCount + (float)rightCount) * 0.5;
}

// ---------------- IMU functions ----------------
bool waitForAccelerationSample() {
  unsigned long startTime = millis();
  while (!IMU.accelerationAvailable()) {
    if (millis() - startTime > 1000) return false;
    delay(1);
  }
  return true;
}

bool waitForGyroscopeSample() {
  unsigned long startTime = millis();
  while (!IMU.gyroscopeAvailable()) {
    if (millis() - startTime > 1000) return false;
    delay(1);
  }
  return true;
}

bool calibrateGyroOffset() {
  float pitchGyroSum = 0.0;
  float yawGyroSum = 0.0;
  int stableSamples = 0;

  Serial.println("# Automatic gyro offset calibration starting.");
  Serial.println("# Keep robot completely still. Upright is not required.");

  while (stableSamples < GYRO_CALIBRATION_SAMPLES) {
    if (!waitForAccelerationSample() || !waitForGyroscopeSample()) {
      Serial.println("# Gyro calibration timeout.");
      return false;
    }

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    bool accelerationStable =
      fabs(accelMagnitude - 1.0) < GYRO_CALIBRATION_ACCEL_MAG_TOLERANCE_G;
    bool gyroStable =
      fabs(gx) < GYRO_CALIBRATION_MAX_STILL_DPS &&
      fabs(gy) < GYRO_CALIBRATION_MAX_STILL_DPS &&
      fabs(gz) < GYRO_CALIBRATION_MAX_STILL_DPS;

    if (accelerationStable && gyroStable) {
      pitchGyroSum += gx;
      yawGyroSum += gz;
      stableSamples++;
    } else {
      pitchGyroSum = 0.0;
      yawGyroSum = 0.0;
      stableSamples = 0;
    }

    delay(5);
  }

  gyroOffset = pitchGyroSum / GYRO_CALIBRATION_SAMPLES;
  yawGyroOffset = yawGyroSum / GYRO_CALIBRATION_SAMPLES;
  thetaOffset = 0.0;

  if (!waitForAccelerationSample()) {
    Serial.println("# Initial angle sample timeout.");
    return false;
  }

  IMU.readAcceleration(ax, ay, az);
  thetaA = atan2(ay, az) * 180.0 / PI - thetaOffset;
  thetaN = thetaA;
  lastImuTime = micros();

  Serial.print("# Pitch gyro offset = ");
  Serial.println(gyroOffset, 3);
  Serial.print("# Yaw gyro offset = ");
  Serial.println(yawGyroOffset, 3);
  Serial.print("# Initial gravity angle = ");
  Serial.println(thetaN, 3);
  Serial.println("# Automatic gyro offset calibration complete.");

  return true;
}

void waitForControlLoopSlot() {
  if (CONTROL_LOOP_TARGET_PERIOD_US == 0) {
    return;
  }

  unsigned long now = micros();
  if (nextControlLoopStartUs == 0) {
    nextControlLoopStartUs = now;
    return;
  }

  nextControlLoopStartUs += CONTROL_LOOP_TARGET_PERIOD_US;

  long remainingUs = (long)(nextControlLoopStartUs - micros());
  if (remainingUs < -(long)CONTROL_LOOP_TARGET_PERIOD_US) {
    nextControlLoopStartUs = micros();
    return;
  }

  while ((long)(nextControlLoopStartUs - micros()) > 1000) {
    delay(1);
  }

  remainingUs = (long)(nextControlLoopStartUs - micros());
  if (remainingUs > 0) {
    delayMicroseconds((unsigned int)remainingUs);
  }
}

bool readTilt(float *tiltAngleDeg, float *gyroRateOut, float *dtOut) {
  if (!IMU.readAccelerationAndGyroscope(ax, ay, az, gx, gy, gz)) {
    return false;
  }

  unsigned long now = micros();
  float dt = (now - lastImuTime) / 1000000.0;
  lastImuTime = now;

  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.005;
  }

  float rawThetaA = atan2(ay, az) * 180.0 / PI;
  thetaA = rawThetaA - thetaOffset;

  float gyroRate = -(gx - gyroOffset);
  thetaN = FILTER_K * (thetaN + gyroRate * dt) + (1.0 - FILTER_K) * thetaA;

  *tiltAngleDeg = thetaN;
  *gyroRateOut = gyroRate;
  *dtOut = dt;

  return true;
}

// ---------------- PID memory ----------------
void resetBalancePidMemory() {
  balanceIntegralError = 0.0;
}

void resetPositionLoopMemory() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  measuredPositionCounts = averageEncoderCounts(leftCount, rightCount);
  lastMeasuredPositionCounts = measuredPositionCounts;
  measuredPositionVelocityCountsPerSec = 0.0;
  bleCommandedPositionTargetCounts = targetPositionCounts;
  blePositionTargetRampActive = false;
  positionErrorCounts = 0.0;
  positionIntegralError = 0.0;
  positionPAngleDeg = 0.0;
  positionIAngleDeg = 0.0;
  positionDAngleDeg = 0.0;
  positionAngleOffsetDeg = 0.0;
  lastPositionSampleMs = millis();
}

void resetPositionLoopStateForCurrentTarget() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  measuredPositionCounts = averageEncoderCounts(leftCount, rightCount);
  lastMeasuredPositionCounts = measuredPositionCounts;
  measuredPositionVelocityCountsPerSec = 0.0;
  positionErrorCounts = targetPositionCounts - measuredPositionCounts;
  positionIntegralError = 0.0;
  positionPAngleDeg = 0.0;
  positionIAngleDeg = 0.0;
  positionDAngleDeg = 0.0;
  positionAngleOffsetDeg = 0.0;
  lastPositionSampleMs = millis();
}

void moveBlePositionTargetRelativeCounts(float deltaCounts) {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentPositionCounts =
    averageEncoderCounts(leftCount, rightCount);

  bleCommandedPositionTargetCounts = currentPositionCounts + deltaCounts;

  if (BLE_POSITION_TARGET_RAMP_ENABLED) {
    targetPositionCounts = currentPositionCounts;
    blePositionTargetRampActive = true;
  } else {
    targetPositionCounts = bleCommandedPositionTargetCounts;
    blePositionTargetRampActive = false;
  }

  resetPositionLoopStateForCurrentTarget();
}

void moveBlePositionTargetAbsoluteCounts(float absoluteTargetCounts) {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentPositionCounts =
    averageEncoderCounts(leftCount, rightCount);

  bleCommandedPositionTargetCounts = absoluteTargetCounts;

  if (BLE_POSITION_TARGET_RAMP_ENABLED) {
    targetPositionCounts = currentPositionCounts;
    blePositionTargetRampActive = true;
  } else {
    targetPositionCounts = bleCommandedPositionTargetCounts;
    blePositionTargetRampActive = false;
  }

  resetPositionLoopStateForCurrentTarget();
}

void holdBlePositionAtCurrentCounts() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentPositionCounts =
    averageEncoderCounts(leftCount, rightCount);

  targetPositionCounts = currentPositionCounts;
  bleCommandedPositionTargetCounts = currentPositionCounts;
  blePositionTargetRampActive = false;
  resetPositionLoopStateForCurrentTarget();
}

void cancelRampSequence() {
  rampSequenceState = RAMP_SEQUENCE_IDLE;
  rampHoldStartMs = 0;
}

void holdCurrentHeading();

void startRampSequence() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  rampStartPositionCounts = averageEncoderCounts(leftCount, rightCount);
  rampSequenceState = RAMP_SEQUENCE_ASCEND;
  rampHoldStartMs = 0;
  holdCurrentHeading();
  moveBlePositionTargetAbsoluteCounts(rampStartPositionCounts + RAMP_TRAVEL_COUNTS);
}

bool positionTargetReached(float toleranceCounts) {
  return !blePositionTargetRampActive && fabs(positionErrorCounts) <= toleranceCounts;
}

void updateRampSequence() {
  if (rampSequenceState == RAMP_SEQUENCE_IDLE) {
    return;
  }

  if (rampSequenceState == RAMP_SEQUENCE_ASCEND) {
    if (positionTargetReached(RAMP_POSITION_TOLERANCE_COUNTS)) {
      rampSequenceState = RAMP_SEQUENCE_HOLD_TOP;
      rampHoldStartMs = millis();
      holdBlePositionAtCurrentCounts();
    }
    return;
  }

  if (rampSequenceState == RAMP_SEQUENCE_HOLD_TOP) {
    if (millis() - rampHoldStartMs >= RAMP_HOLD_TIME_MS) {
      rampSequenceState = RAMP_SEQUENCE_DESCEND;
      moveBlePositionTargetAbsoluteCounts(rampStartPositionCounts);
    }
    return;
  }

  if (rampSequenceState == RAMP_SEQUENCE_DESCEND) {
    if (positionTargetReached(RAMP_POSITION_TOLERANCE_COUNTS)) {
      holdBlePositionAtCurrentCounts();
      cancelRampSequence();
    }
  }
}

void resetTurningLoopMemory() {
  yawRateDps = 0.0;
  yawAngleDeg = 0.0;
  yawTargetAngleDeg = 0.0;
  turningErrorDeg = 0.0;
  turningPwm = 0.0;
}

void resetRampSequenceMemory() {
  rampSequenceState = RAMP_SEQUENCE_IDLE;
  rampStartPositionCounts = 0.0;
  rampHoldStartMs = 0;
}

void resetControlMemory() {
  resetPositionLoopMemory();
  resetTurningLoopMemory();
  resetRampSequenceMemory();
  resetBalancePidMemory();
}

bool shouldAccumulateBalanceIntegral() {
  return robotArmed && !safetyStopActive;
}

float constrainIntegralByIOutput(float integralError, float ki, float maxAbsoluteIOutput) {
  if (fabs(ki) <= 0.000001) {
    return 0.0;
  }

  float integralLimit = maxAbsoluteIOutput / fabs(ki);
  return clampFloat(integralError, -integralLimit, integralLimit);
}

// ---------------- Position outer-loop PID ----------------
void updatePositionLoop() {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastPositionSampleMs;
  if (elapsedMs < POSITION_LOOP_INTERVAL_MS) {
    return;
  }

  float elapsedSeconds = elapsedMs / 1000.0;
  if (blePositionTargetRampActive) {
    float maxTargetStep =
      BLE_POSITION_TARGET_RAMP_COUNTS_PER_SEC * elapsedSeconds;
    float targetDelta =
      bleCommandedPositionTargetCounts - targetPositionCounts;

    if (targetDelta > maxTargetStep) {
      targetPositionCounts += maxTargetStep;
    }
    else if (targetDelta < -maxTargetStep) {
      targetPositionCounts -= maxTargetStep;
    }
    else {
      targetPositionCounts = bleCommandedPositionTargetCounts;
      blePositionTargetRampActive = false;
    }
  }

  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  measuredPositionCounts = averageEncoderCounts(leftCount, rightCount);
  positionErrorCounts = targetPositionCounts - measuredPositionCounts;

  measuredPositionVelocityCountsPerSec =
    (measuredPositionCounts - lastMeasuredPositionCounts) / elapsedSeconds;
  lastMeasuredPositionCounts = measuredPositionCounts;
  lastPositionSampleMs = now;

  float previousIntegralError = positionIntegralError;
  positionIntegralError += positionErrorCounts * elapsedSeconds;
  positionIntegralError = clampFloat(positionIntegralError,
                                     -POSITION_INTEGRAL_LIMIT,
                                     POSITION_INTEGRAL_LIMIT);

  positionPAngleDeg = POSITION_LOOP_DIRECTION_SIGN * positionKp * positionErrorCounts;
  positionIAngleDeg = POSITION_LOOP_DIRECTION_SIGN * positionKi * positionIntegralError;
  positionDAngleDeg = POSITION_LOOP_DIRECTION_SIGN * -positionKd * measuredPositionVelocityCountsPerSec;

  float rawAngleOffset = positionPAngleDeg + positionIAngleDeg + positionDAngleDeg;

  positionAngleOffsetDeg = clampFloat(rawAngleOffset,
                                      -MAX_POSITION_ANGLE_OFFSET_DEG,
                                      MAX_POSITION_ANGLE_OFFSET_DEG);

  if (rawAngleOffset != positionAngleOffsetDeg && positionKi != 0.0) {
    positionIntegralError = previousIntegralError;
  }
}

// ---------------- Turning loop ----------------
void updateYawEstimate(float dt) {
  yawRateDps = gz - yawGyroOffset;
  yawAngleDeg = wrapAngleDeg(yawAngleDeg + yawRateDps * dt);
}

void moveTurningTargetRelativeDeg(float deltaDeg) {
  yawTargetAngleDeg = wrapAngleDeg(yawTargetAngleDeg + deltaDeg);
}

void holdCurrentHeading() {
  yawTargetAngleDeg = yawAngleDeg;
  turningErrorDeg = 0.0;
  turningPwm = 0.0;
}

void applyTurningLoopP(int rawCommand, float dt, int *leftRawCommand, int *rightRawCommand) {
  updateYawEstimate(dt);

  *leftRawCommand = rawCommand;
  *rightRawCommand = rawCommand;
  turningPwm = 0.0;

  if (!TURNING_LOOP_ENABLED) {
    turningErrorDeg = 0.0;
    return;
  }

  turningErrorDeg = wrapAngleDeg(yawTargetAngleDeg - yawAngleDeg);

  if (fabs(turningErrorDeg) <= TURNING_DEADBAND_DEG) {
    return;
  }

  turningPwm = TURNING_LOOP_DIRECTION_SIGN * turningKp * turningErrorDeg;
  turningPwm = clampFloat(turningPwm, -MAX_TURNING_PWM, MAX_TURNING_PWM);

  *leftRawCommand = (int)clampFloat(rawCommand - turningPwm, -255.0, 255.0);
  *rightRawCommand = (int)clampFloat(rawCommand + turningPwm, -255.0, 255.0);
}

// ---------------- Upright inner-loop PID ----------------
int balancePidCommand(float targetAngleDeg, float tiltAngleDeg, float gyroRate, float dt) {
  float error = targetAngleDeg - tiltAngleDeg;

  if (fabs(balanceKi) <= 0.000001) {
    balanceIntegralError = 0.0;
  } else if (shouldAccumulateBalanceIntegral()) {
    balanceIntegralError += error * dt;
    balanceIntegralError =
      constrainIntegralByIOutput(
        balanceIntegralError,
        balanceKi,
        BALANCE_I_OUTPUT_LIMIT_PWM
      );
  } else {
    balanceIntegralError = 0.0;
  }

  float output = balanceKp * error +
                 balanceKi * balanceIntegralError +
                 -balanceKd * gyroRate;

  output *= MOTOR_DIRECTION_SIGN;
  float clampedOutput = clampFloat(output, -255.0, 255.0);

  return (int)clampedOutput;
}

void printDebugLine(float tiltAngleDeg, float gyroRate, float targetAngleDeg, int rawCommand,
                    int leftCommand, int rightCommand, float averageControlLoopHz) {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float error = targetAngleDeg - tiltAngleDeg;
  float kpPwm = balanceKp * error * MOTOR_DIRECTION_SIGN;
  float kiPwm = balanceKi * balanceIntegralError * MOTOR_DIRECTION_SIGN;
  float kdPwm = -balanceKd * gyroRate * MOTOR_DIRECTION_SIGN;
  long syncCountError = leftCount - rightCount;

  Serial.print("angle=");
  Serial.print(tiltAngleDeg, 2);
  Serial.print(" target=");
  Serial.print(targetAngleDeg, 2);
  Serial.print(" error=");
  Serial.print(error, 2);
  Serial.print(" ms=");
  Serial.print(millis());
  Serial.print(" Hz=");
  Serial.print(averageControlLoopHz, 1);
  Serial.print(" leftPWM=");
  Serial.print(leftCommand);
  Serial.print(" rightPWM=");
  Serial.print(rightCommand);
  Serial.print(" balancePWM=");
  Serial.print(rawCommand);
  Serial.print(" kpPWM=");
  Serial.print(kpPwm, 2);
  Serial.print(" kiPWM=");
  Serial.print(kiPwm, 2);
  Serial.print(" iErr=");
  Serial.print(balanceIntegralError, 4);
  Serial.print(" kdPWM=");
  Serial.print(kdPwm, 2);
  Serial.print(" posTarget=");
  Serial.print(targetPositionCounts, 1);
  Serial.print(" pos=");
  Serial.print(measuredPositionCounts, 1);
  Serial.print(" posErr=");
  Serial.print(positionErrorCounts, 1);
  Serial.print(" posVel=");
  Serial.print(measuredPositionVelocityCountsPerSec, 1);
  Serial.print(" posAngle=");
  Serial.print(positionAngleOffsetDeg, 2);
  Serial.print(" balanceKp=");
  Serial.print(balanceKp, 4);
  Serial.print(" balanceKi=");
  Serial.print(balanceKi, 4);
  Serial.print(" balanceKd=");
  Serial.print(balanceKd, 4);
  Serial.print(" positionKp=");
  Serial.print(positionKp, 6);
  Serial.print(" positionKi=");
  Serial.print(positionKi, 6);
  Serial.print(" positionKd=");
  Serial.print(positionKd, 6);
  Serial.print(" leftCount=");
  Serial.print(leftCount);
  Serial.print(" rightCount=");
  Serial.print(rightCount);
  Serial.print(" syncCountErr=");
  Serial.print(syncCountError);
  Serial.print(" yaw=");
  Serial.print(yawAngleDeg, 2);
  Serial.print(" yawTarget=");
  Serial.print(yawTargetAngleDeg, 2);
  Serial.print(" turnErr=");
  Serial.print(turningErrorDeg, 2);
  Serial.print(" turnKp=");
  Serial.print(turningKp, 3);
  Serial.print(" turnPWM=");
  Serial.println(turningPwm, 2);
}

void sendTelemetryIfDue(float targetAngleDeg, float tiltAngleDeg, float gyroRate, int rawCommand,
                        int leftCommand, int rightCommand, bool force) {
  static unsigned long lastTelemetryPrint = 0;
  static unsigned long telemetrySampleCount = 0;

  unsigned long now = millis();
  if (lastTelemetryPrint == 0) {
    lastTelemetryPrint = now;
    telemetrySampleCount = 0;
  }

  telemetrySampleCount++;

  unsigned long elapsedMs = now - lastTelemetryPrint;
  if (!force && elapsedMs <= DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  float averageControlLoopHz = 0.0;
  if (elapsedMs > 0) {
    averageControlLoopHz = telemetrySampleCount * 1000.0 / elapsedMs;
  }

  if (DEBUG_PRINT_ENABLED || POSITION_PLOTTER_TELEMETRY_ENABLED) {
    printDebugLine(
      tiltAngleDeg,
      gyroRate,
      targetAngleDeg,
      rawCommand,
      leftCommand,
      rightCommand,
      averageControlLoopHz
    );
  }

  telemetrySampleCount = 0;
  lastTelemetryPrint = now;
}

void printControlFrequencyIfDue(float dt) {
  if (!CONTROL_FREQ_PRINT_ENABLED) {
    return;
  }

  unsigned long now = millis();
  if (controlFreqWindowStartMs == 0) {
    controlFreqWindowStartMs = now;
    controlFreqSampleCount = 0;
    controlFreqMaxDtMs = 0.0;
  }

  controlFreqSampleCount++;

  float dtMs = dt * 1000.0;
  if (dtMs > controlFreqMaxDtMs) {
    controlFreqMaxDtMs = dtMs;
  }

  unsigned long elapsedMs = now - controlFreqWindowStartMs;
  if (elapsedMs < CONTROL_FREQ_PRINT_INTERVAL_MS) {
    return;
  }

  float controlHz = 0.0;
  if (elapsedMs > 0) {
    controlHz = controlFreqSampleCount * 1000.0 / elapsedMs;
  }

  Serial.print("Hz=");
  Serial.print(controlHz, 1);
  Serial.print(" maxDtMs=");
  Serial.print(controlFreqMaxDtMs, 2);
  Serial.print(" blePolls=");
  Serial.print(blePollCount);
  Serial.print(" blePollMaxUs=");
  Serial.println(blePollMaxDurationUs);

  controlFreqWindowStartMs = now;
  controlFreqSampleCount = 0;
  controlFreqMaxDtMs = 0.0;
  blePollCount = 0;
  blePollMaxDurationUs = 0;
}

// ---------------- BLE commands ----------------
void recordBlePollDuration(unsigned long pollStartUs) {
  unsigned long durationUs = micros() - pollStartUs;
  blePollCount++;
  if (durationUs > blePollMaxDurationUs) {
    blePollMaxDurationUs = durationUs;
  }
}

void setupBle() {
  if (!BLE_ENABLED) {
    bleReady = false;
    bleCentralConnected = false;
    return;
  }

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  if (!BLE.begin()) {
    bleReady = false;
    Serial.println("# BLE init failed. Robot will still run without BLE.");
    return;
  }

  BLE.setLocalName(BLE_LOCAL_NAME);
  BLE.setDeviceName(BLE_LOCAL_NAME);

  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);

  customCharacteristic.writeValue("Ready");
  BLE.advertise();

  bleReady = true;

  Serial.print("# BLE advertising as ");
  Serial.println(BLE_LOCAL_NAME);

  Serial.print("# BLE poll interval loops = ");
  Serial.println(BLE_CONTROL_LOOP_POLL_INTERVAL);
}

void handleBleCommandBuffer(const unsigned char* data, int length) {
  char commandText[BLE_BUFFER_SIZE + 1];
  int usableLength = length;
  if (usableLength > BLE_BUFFER_SIZE) {
    usableLength = BLE_BUFFER_SIZE;
  }

  for (int i = 0; i < usableLength; i++) {
    commandText[i] = (char)toupper(data[i]);
  }
  commandText[usableLength] = '\0';

  bool rampCommand =
    commandText[0] == 'P' ||
    commandText[0] == 'M' ||
    (usableLength >= 4 &&
     commandText[0] == 'R' &&
     commandText[1] == 'A' &&
     commandText[2] == 'M' &&
     commandText[3] == 'P');

  if (rampCommand) {
    startRampSequence();
  }
  else if (commandText[0] == 'F' || commandText[0] == 'W') {
    cancelRampSequence();
    moveBlePositionTargetRelativeCounts(BLE_MOVE_TARGET_COUNTS);
  }
  else if (commandText[0] == 'B') {
    cancelRampSequence();
    moveBlePositionTargetRelativeCounts(-BLE_MOVE_TARGET_COUNTS);
  }
  else if (commandText[0] == 'L') {
    cancelRampSequence();
    moveTurningTargetRelativeDeg(-BLE_TURN_TARGET_DEG);
  }
  else if (commandText[0] == 'R') {
    cancelRampSequence();
    moveTurningTargetRelativeDeg(BLE_TURN_TARGET_DEG);
  }
  else if (commandText[0] == 'S' || commandText[0] == 'X' || commandText[0] == '0') {
    cancelRampSequence();
    holdBlePositionAtCurrentCounts();
    holdCurrentHeading();
  }
}

void pollBleCommandAfterControlLoop() {
  if (!BLE_ENABLED || !bleReady) {
    return;
  }

  controlLoopsSinceBlePoll++;
  if (controlLoopsSinceBlePoll < BLE_CONTROL_LOOP_POLL_INTERVAL) {
    return;
  }

  controlLoopsSinceBlePoll = 0;

  unsigned long pollStartUs = micros();
  BLE.poll();

  bool connected = BLE.connected();
  if (connected != bleCentralConnected) {
    bleCentralConnected = connected;
    digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);
  }

  if (!customCharacteristic.written()) {
    recordBlePollDuration(pollStartUs);
    return;
  }

  int length = customCharacteristic.valueLength();
  const unsigned char* data = customCharacteristic.value();
  if (length <= 0 || data == nullptr) {
    recordBlePollDuration(pollStartUs);
    return;
  }

  handleBleCommandBuffer(data, length);
  recordBlePollDuration(pollStartUs);
}

// ---------------- Arduino runtime ----------------
void setup() {
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  coastAllMotors();

  Serial.begin(SERIAL_BAUD);
  while (!Serial && millis() < 4000) {
    delay(10);
  }

  setupBle();

  setupEncoders();

  if (!IMU.begin(BOSCH_ACCELEROMETER_ONLY)) {
    Serial.println("# Failed to initialize IMU!");
    while (1) {
      coastAllMotors();
      delay(1000);
    }
  }

  Wire1.setClock(IMU_I2C_CLOCK_HZ);

  Serial.print("# Accel sample rate = ");
  Serial.println(IMU.accelerationSampleRate(), 1);
  Serial.print("# Gyro sample rate = ");
  Serial.println(IMU.gyroscopeSampleRate(), 1);

  Serial.println("# Ready.");
  delay(500);

  if (!calibrateGyroOffset()) {
    Serial.println("# Gyro calibration failed.");
    while (1) {
      coastAllMotors();
      delay(1000);
    }
  }

  resetEncoderCounts();

  Serial.println("# Start balance-position-turning balancing.");
  Serial.println("# Position loop enabled. Speed loop disabled.");
  Serial.println("# Turning loop enabled.");
  Serial.print("# Control loop target Hz: ");
  Serial.println(1000000.0 / CONTROL_LOOP_TARGET_PERIOD_US, 1);
  Serial.print("# BLE position target ramp cm/s: ");
  Serial.println(BLE_POSITION_TARGET_RAMP_SPEED_CM_PER_SEC, 1);
  Serial.println("# Waiting for arm: hold near target angle and keep still.");

  resetControlMemory();
  robotArmed = false;
  stableStartMs = 0;
  lastTiltSampleMs = millis();
  nextControlLoopStartUs = micros();
}

void loop() {
  float tiltAngleDeg = 0.0;
  float gyroRate = 0.0;
  float dt = 0.005;

  waitForControlLoopSlot();

  if (readTilt(&tiltAngleDeg, &gyroRate, &dt)) {
    lastTiltSampleMs = millis();
    imuTimeoutStopActive = false;
    printControlFrequencyIfDue(dt);

    if (safetyStopActive) {
      coastAllMotors();
      resetControlMemory();

      if (abs(tiltAngleDeg) >= SAFETY_RESTART_ANGLE_DEG) {
        const float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
        sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, false);

        static unsigned long lastSafetyPrint = 0;
        if (millis() - lastSafetyPrint > 500) {
          Serial.print("# Safety stop active. Tilt = ");
          Serial.println(tiltAngleDeg, 2);
          lastSafetyPrint = millis();
        }
        pollBleCommandAfterControlLoop();
        return;
      }

      safetyStopActive = false;
      robotArmed = false;
      stableStartMs = 0;
      resetControlMemory();
      Serial.println("# Safety stop cleared. Control memory reset.");
      pollBleCommandAfterControlLoop();
      return;
    }

    if (abs(tiltAngleDeg) > MAX_SAFE_ANGLE_DEG) {
      coastAllMotors();
      safetyStopActive = true;
      robotArmed = false;
      stableStartMs = 0;
      resetControlMemory();

      const float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
      sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, true);

      Serial.print("# Safety stop. Tilt = ");
      Serial.println(tiltAngleDeg, 2);
      pollBleCommandAfterControlLoop();
      return;
    }

    if (!robotArmed) {
      coastAllMotors();
      resetControlMemory();

      const float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
      bool angleReady =
        fabs(tiltAngleDeg - targetAngleDeg) < START_ANGLE_WINDOW_DEG;
      bool gyroReady =
        fabs(gyroRate) < START_GYRO_WINDOW_DPS;

      if (angleReady && gyroReady) {
        if (stableStartMs == 0) {
          stableStartMs = millis();
        }

        if (millis() - stableStartMs >= ARM_STABLE_TIME_MS) {
          robotArmed = true;
          stableStartMs = 0;
          resetControlMemory();
          Serial.println("# ARMED: balance-position controller enabled.");
        }
      } else {
        stableStartMs = 0;
      }

      sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, false);
      pollBleCommandAfterControlLoop();
      return;
    }

    updatePositionLoop();
    updateRampSequence();

    float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
    int rawCommand = balancePidCommand(targetAngleDeg, tiltAngleDeg, gyroRate, dt);

    int leftRawCommand = 0;
    int rightRawCommand = 0;
    applyTurningLoopP(rawCommand, dt, &leftRawCommand, &rightRawCommand);

    int leftCommand = addDeadband(leftRawCommand, leftDeadband);
    int rightCommand = addDeadband(rightRawCommand, rightDeadband);
    rightCommand = applyRightBackwardPwmCompensation(rightCommand);

    driveSigned(LEFT_IN1, LEFT_IN2, leftCommand);
    driveSigned(RIGHT_IN1, RIGHT_IN2, rightCommand);

    sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, rawCommand, leftCommand, rightCommand, false);
    pollBleCommandAfterControlLoop();
  } else if (millis() - lastTiltSampleMs > IMU_SAMPLE_TIMEOUT_MS) {
    coastAllMotors();
    robotArmed = false;
    stableStartMs = 0;
    resetControlMemory();

    if (!imuTimeoutStopActive) {
      Serial.println("# IMU sample timeout. Motors stopped.");
      imuTimeoutStopActive = true;
    }
  }

  if (CONTROL_LOOP_IDLE_DELAY_MS > 0) {
    delay(CONTROL_LOOP_IDLE_DELAY_MS);
  }
}

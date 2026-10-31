#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

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
const uint16_t BLE_CONTROL_LOOP_POLL_INTERVAL = 20;

const float BLE_POSITION_TARGET_COUNTS = 3820.0;

bool bleReady = false;
bool bleCentralConnected = false;
uint16_t controlLoopsSinceBlePoll = 0;

// ---------------- Motor pins from 391ProjectBalance ----------------
const uint8_t LEFT_IN1 = 9;
const uint8_t LEFT_IN2 = 10;
const uint8_t RIGHT_IN1 = 4;
const uint8_t RIGHT_IN2 = 6;

const unsigned long SERIAL_BAUD = 115200;
const unsigned long IMU_I2C_CLOCK_HZ = 1000000;
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// ---------------- Encoder pins from 391Lab2 ----------------
const uint8_t LEFT_ENCODER_A = 11;
const uint8_t LEFT_ENCODER_B = 12;
const uint8_t RIGHT_ENCODER_A = 7;
const uint8_t RIGHT_ENCODER_B = 8;

const long LEFT_ENCODER_CPR = 1920;
const long RIGHT_ENCODER_CPR = 1920;

const int LEFT_ENCODER_SIGN = -1;
const int RIGHT_ENCODER_SIGN = 1;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint8_t leftEncoderState = 0;
volatile uint8_t rightEncoderState = 0;

// ---------------- Motor deadband from 391ProjectBalance ----------------
int leftDeadband = 42;
int rightDeadband = 42;
const int RIGHT_BACKWARD_PWM_COMPENSATION = 0;

// ---------------- IMU/filter settings from 391ProjectBalance ----------------
const int GYRO_CALIBRATION_SAMPLES = 500;
const float GYRO_CALIBRATION_ACCEL_MAG_TOLERANCE_G = 0.12;
const float GYRO_CALIBRATION_MAX_STILL_DPS = 5.0;
const float FILTER_K = 0.98;

// ---------------- Upright inner-loop PID ----------------
const float BASE_TARGET_ANGLE_DEG = -0.5;//-0.87;//-0.96
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
const float MAX_POSITION_ANGLE_OFFSET_DEG = 3;
const float POSITION_INTEGRAL_LIMIT = 2000.0;
const float POSITION_TARGET_RAMP_SPEED_CM_PER_SEC = 12.0;
const float WHEEL_DIAMETER_CM = 8.0;
const float ENCODER_COUNTS_PER_CM =
  LEFT_ENCODER_CPR / (PI * WHEEL_DIAMETER_CM);
const float POSITION_TARGET_RAMP_COUNTS_PER_SEC =
  POSITION_TARGET_RAMP_SPEED_CM_PER_SEC * ENCODER_COUNTS_PER_CM;
// Flip this sign if position error drives the robot farther from the target.
const int POSITION_LOOP_DIRECTION_SIGN = 1;

float targetPositionCounts = 0.0;
float commandedPositionTargetCounts = 0.0;
float positionKp = 0.0016;//0.0015
float positionKi = 0;
float positionKd = 0.0007;//0.0007

// During 45-degree center-pivot turns, the normal position loop can be too weak.
// These multipliers keep the encoder-average center closer to the start point.
const float YAW_TURN_POSITION_KP_MULT = 1.15;  // Keep position hold only slightly stronger during turns; too much can knock the robot over.
const float YAW_TURN_POSITION_KD_MULT = 1.00;
const float YAW_TURN_MAX_POSITION_ANGLE_OFFSET_DEG = 3.2;

// ---------------- Yaw heading hold ----------------
const bool YAW_HEADING_HOLD_ENABLED = true;
const float YAW_HEADING_KP = 0.25;
const float YAW_HEADING_DEADBAND_DEG = 1.0;
const float MAX_YAW_CORRECTION_PWM = 5.0;
const int YAW_CONTROL_MIN_RAW_PWM = 3;
const int YAW_CONTROL_DIRECTION_SIGN = 1;

// ---------------- Yaw 45-degree turn controller ----------------
// BLE L/A = left 45 deg, R/D = right 45 deg.
// If the robot turns the wrong way for both L and R, flip YAW_TURN_DIRECTION_SIGN.
// If L/R are simply swapped but the turn still stops correctly, flip BLE_LEFT_TURN_SIGN.
// Base 45-degree targets.
// The auto-90 sequence below can use different first/second-step targets
// to compensate for the real robot stopping a little short on the second turn.
const float BLE_LEFT_TURN_TARGET_DEG = 45.0;
const float BLE_RIGHT_TURN_TARGET_DEG = 45.0;
const float BLE_LEFT_FIRST_TURN_TARGET_DEG = 45.0;
const float BLE_LEFT_SECOND_TURN_TARGET_DEG = 45.0;
const float BLE_RIGHT_FIRST_TURN_TARGET_DEG = 46.0;   // Right turn was a little short, so give a small angle compensation.
const float BLE_RIGHT_SECOND_TURN_TARGET_DEG = 48.0;  // Second right turn was slower/shorter, so compensate slightly more.
const int BLE_LEFT_TURN_SIGN = 1;
const bool YAW_TURN_ENABLED = true;
const float YAW_TURN_KP = 0.82;        // Slightly faster, but still safer than the aggressive versions.
const float YAW_TURN_KD = 0.40;        // Lighter braking so each 45 deg turn finishes faster.
const float MAX_YAW_TURN_PWM = 40.0;   // A little more turn power to help meet the 2 second limit.
const float MIN_YAW_TURN_PWM = 0.0;    // Do not force minimum PWM near the target.
const float YAW_TURN_DONE_ERROR_DEG = 3.0;
const float YAW_TURN_DONE_RATE_DPS = 32.0;
const unsigned long YAW_TURN_SETTLE_MS = 25;
const int YAW_TURN_DIRECTION_SIGN = 1;
const float YAW_TURN_SLOW_ZONE_DEG = 7.0;    // Brake a bit later so it keeps speed for longer.
const float YAW_TURN_STOP_ERROR_DEG = 3.0;   // Stop a little earlier to avoid fighting/over-braking near the target.
const float YAW_TURN_FAST_RATE_DPS = 48.0;   // Allow more yaw speed before emergency braking.
const float YAW_TURN_BRAKE_KD = 0.34;        // Softer brake inside the slow zone.

// Left/right turn parameters are intentionally separated here.
// Left was shaking, so it gets slightly less power and more damping.
// Right was short/slow, so it gets more power and lighter braking.
const float LEFT_TURN_KP_SCALE = 0.72;
const float LEFT_TURN_KD_SCALE = 1.58;
const float LEFT_MAX_YAW_TURN_PWM = 30.0;
const float LEFT_TURN_FAST_RATE_DPS = 30.0;
const float LEFT_TURN_BRAKE_KD_SCALE = 1.60;

const float RIGHT_TURN_KP_SCALE = 0.86;
const float RIGHT_TURN_KD_SCALE = 1.18;
const float RIGHT_MAX_YAW_TURN_PWM = 35.0;
const float RIGHT_TURN_FAST_RATE_DPS = 42.0;
const float RIGHT_TURN_BRAKE_KD_SCALE = 1.18;

// Second-step protection is now direction-specific.
// Left second turn stays damped to reduce shaking.
const float LEFT_SECOND_TURN_KP_SCALE = 0.66;
const float LEFT_SECOND_TURN_KD_SCALE = 1.75;
const float LEFT_SECOND_MAX_YAW_TURN_PWM = 28.0;
const float LEFT_SECOND_TURN_FAST_RATE_DPS = 26.0;
const float LEFT_SECOND_TURN_BRAKE_KD_SCALE = 1.80;
const float LEFT_SECOND_TURN_DONE_RATE_DPS = 18.0;
const unsigned long LEFT_SECOND_TURN_SETTLE_MS = 90;
const float LEFT_SECOND_TURN_STOP_BRAKE_MAX_PWM = 12.0;
const unsigned long LEFT_SECOND_TURN_POST_RECOVERY_MS = 900;

// Right second turn is faster so it can finish within 2 seconds.
const float RIGHT_SECOND_TURN_KP_SCALE = 0.90;
const float RIGHT_SECOND_TURN_KD_SCALE = 1.10;
const float RIGHT_SECOND_MAX_YAW_TURN_PWM = 38.0;
const float RIGHT_SECOND_TURN_FAST_RATE_DPS = 50.0;
const float RIGHT_SECOND_TURN_BRAKE_KD_SCALE = 1.05;
const float RIGHT_SECOND_TURN_DONE_RATE_DPS = 30.0;
const unsigned long RIGHT_SECOND_TURN_SETTLE_MS = 35;
const float RIGHT_SECOND_TURN_STOP_BRAKE_MAX_PWM = 7.0;
const unsigned long RIGHT_SECOND_TURN_POST_RECOVERY_MS = 650;

const float TURN_DONE_RATE_DPS = 24.0;
const unsigned long TURN_SETTLE_MS = 45;
const float TURN_STOP_BRAKE_MAX_PWM = 8.0;

// After a yaw turn finishes, briefly disable heading-hold and yaw-rate transient damping.
// This avoids a sudden correction exactly when the turn controller releases.
const unsigned long LEFT_TURN_POST_RECOVERY_MS = 750;
const unsigned long RIGHT_TURN_POST_RECOVERY_MS = 550;


// ---------------- Rubric 90-degree timing sequences ----------------
// Press L/A once: the robot automatically performs two 45 deg left turns.
// Press R/D once: the robot automatically performs two 45 deg right turns.
// Each sequence pauses 2.5 s between the two 45 deg turns,
// then holds still for 6 s after the full 90 deg turn.
const bool AUTO_LEFT_90_SEQUENCE_ENABLED = true;
const bool AUTO_RIGHT_90_SEQUENCE_ENABLED = true;
const unsigned long TURN_INTER_STEP_PAUSE_MS = 2800;  // Still inside the 2-3 s rubric window, gives the robot more time to settle.
const unsigned long TURN_FINAL_PAUSE_MS = 6000;
const unsigned long TURN_MIN_OPPOSITE_COMMAND_WAIT_MS = 5000;

// ---------------- Center-pivot turn option ----------------
// For L/R turns, use the center of the robot as the pivot.
// The two wheels receive opposite yaw commands so their travel distances are as equal as possible.
const bool YAW_TURN_LEFT_WHEEL_PIVOT_MODE = false;
const float YAW_TURN_PIVOT_MOVING_WHEEL_GAIN = 2.0;  // Not used when center-pivot mode is selected.

// Center lock during center-pivot turns.
// The code does not know the physical center directly.
// It defines the robot center position as the average of the two encoder counts:
// center = (leftCount + rightCount) / 2.
// For a true center pivot, this center value should stay almost unchanged.
const bool YAW_TURN_CENTER_LOCK_ENABLED = false;  // Disabled for stability; center drift is not prioritized in this version.

// If one side is mechanically stronger, scale only the extra yaw-turn part.
// Your robot shows the right wheel turns more than the left, so start with the right side slightly reduced.
const float YAW_TURN_LEFT_EXTRA_PWM_SCALE = 1.00;
const float YAW_TURN_RIGHT_EXTRA_PWM_SCALE = 1.00;

// Direct encoder-based center correction during yaw turns.
// If the center drifts forward/backward, this adds a small same-direction correction to both wheels.
// Stronger direct center lock. If center drift gets worse, flip the direction sign.
const int YAW_TURN_CENTER_LOCK_DIRECTION_SIGN = 1;
const float YAW_TURN_CENTER_LOCK_KP = 0.014;
const float YAW_TURN_CENTER_LOCK_KI = 0.0000;
const float YAW_TURN_CENTER_LOCK_KD = 0.0018;
const float YAW_TURN_CENTER_LOCK_INTEGRAL_LIMIT = 2500.0;
const float MAX_YAW_TURN_CENTER_LOCK_PWM = 10.0;

// Wheel-sync loop during center-pivot turns.
// This is the main correction for keeping the robot center fixed:
// left and right wheel travel should be equal magnitude and opposite sign.
// If the right wheel travels more, this loop slows the right side and pushes the left side harder.
const bool YAW_TURN_WHEEL_SYNC_ENABLED = false;  // Disabled for stability; avoids fighting the balance loop during left turns.
const float YAW_TURN_WHEEL_SYNC_KP = 0.018;
const float YAW_TURN_WHEEL_SYNC_KD = 0.0006;
const float MAX_YAW_TURN_WHEEL_SYNC_PWM = 6.0;
// If wheel-sync makes the left/right mismatch worse, flip this from 1 to -1.
const int YAW_TURN_WHEEL_SYNC_DIRECTION_SIGN = 1;

// ---------------- Yaw-rate transient damping ----------------
const bool YAW_RATE_TRANSIENT_DAMPING_ENABLED = true;
const float YAW_RATE_TRANSIENT_KP = 0.25;
const float YAW_RATE_TRANSIENT_DEADBAND_DPS = 0.9;
const float MAX_YAW_RATE_TRANSIENT_CORRECTION_PWM = 8.0;
const int YAW_RATE_TRANSIENT_MIN_RAW_PWM = 3;
const int YAW_RATE_TRANSIENT_DIRECTION_SIGN = 1;

const unsigned long IMU_SAMPLE_TIMEOUT_MS = 100;
const unsigned long CONTROL_LOOP_IDLE_DELAY_MS = 0;
const unsigned long CONTROL_LOOP_TARGET_PERIOD_US = 5556; // 180 Hz, copied from the more stable BLE base
const unsigned long DEBUG_PRINT_INTERVAL_MS = 50;
const unsigned long CONTROL_FREQUENCY_WINDOW_MS = 1000;
const bool DEBUG_PRINT_ENABLED = false;

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

unsigned long controlFrequencyWindowStartMs = 0;
unsigned long controlFrequencySampleCount = 0;
float measuredControlFrequencyHz = 0.0;
unsigned long nextControlLoopStartUs = 0;

float yawRateDps = 0.0;
float yawAngleDeg = 0.0;
float yawTargetHeadingDeg = 0.0;
float yawHeadingErrorDeg = 0.0;
float yawCorrectionPwm = 0.0;

bool yawTurnActive = false;
unsigned long yawTurnSettledStartMs = 0;
unsigned long yawTurnRecoveryUntilMs = 0;
float yawTurnCorrectionPwm = 0.0;
long yawTurnStartLeftCount = 0;
long yawTurnStartRightCount = 0;
float yawTurnStartCenterCounts = 0.0;
float yawTurnLastCenterCounts = 0.0;
unsigned long yawTurnLastCenterSampleMs = 0;
float yawTurnCenterErrorCounts = 0.0;
float yawTurnCenterVelocityCountsPerSec = 0.0;
float yawTurnCenterIntegralCountsSec = 0.0;
float yawTurnCenterLockCorrectionPwm = 0.0;
int yawTurnCommandSign = 1;
float yawTurnWheelSyncErrorCounts = 0.0;
float yawTurnWheelSyncVelocityCountsPerSec = 0.0;
float yawTurnWheelSyncCorrectionPwm = 0.0;


enum LeftTurnSequenceState {
  LEFT_SEQ_IDLE,
  LEFT_SEQ_FIRST_TURN,
  LEFT_SEQ_PAUSE_BETWEEN,
  LEFT_SEQ_SECOND_TURN,
  LEFT_SEQ_FINAL_PAUSE
};

enum RightTurnSequenceState {
  RIGHT_SEQ_IDLE,
  RIGHT_SEQ_FIRST_TURN,
  RIGHT_SEQ_PAUSE_BETWEEN,
  RIGHT_SEQ_SECOND_TURN,
  RIGHT_SEQ_FINAL_PAUSE
};

LeftTurnSequenceState leftTurnSequenceState = LEFT_SEQ_IDLE;
unsigned long leftTurnSequencePauseStartMs = 0;
RightTurnSequenceState rightTurnSequenceState = RIGHT_SEQ_IDLE;
unsigned long rightTurnSequencePauseStartMs = 0;

float yawRateTransientErrorDps = 0.0;
float yawRateTransientCorrectionPwm = 0.0;

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

float positionControlCountsForCurrentMode(long leftCount, long rightCount) {
  if (YAW_TURN_LEFT_WHEEL_PIVOT_MODE && yawTurnActive) {
    return (float)leftCount;  // Hold the left wheel as the pivot during yaw turns.
  }

  return averageEncoderCounts(leftCount, rightCount);
}

float signedRpmFromCounts(long countChange, long cpr, float elapsedSeconds) {
  if (elapsedSeconds <= 0.0 || cpr == 0) return 0.0;
  return ((float)countChange * 60.0) / ((float)cpr * elapsedSeconds);
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

  measuredPositionCounts = positionControlCountsForCurrentMode(leftCount, rightCount);
  lastMeasuredPositionCounts = measuredPositionCounts;
  measuredPositionVelocityCountsPerSec = 0.0;
  targetPositionCounts = 0.0;
  commandedPositionTargetCounts = 0.0;
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

  measuredPositionCounts = positionControlCountsForCurrentMode(leftCount, rightCount);
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

void setPositionTargetCounts(float targetCounts) {
  if (fabs(commandedPositionTargetCounts - targetCounts) < 0.5) {
    return;
  }

  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  targetPositionCounts = positionControlCountsForCurrentMode(leftCount, rightCount);
  commandedPositionTargetCounts = targetCounts;
  resetPositionLoopStateForCurrentTarget();
}

void movePositionTargetRelativeCounts(float deltaCounts) {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentPositionCounts =
    positionControlCountsForCurrentMode(leftCount, rightCount);

  setPositionTargetCounts(currentPositionCounts + deltaCounts);
}

void holdCurrentPositionTarget() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentPositionCounts =
    positionControlCountsForCurrentMode(leftCount, rightCount);

  targetPositionCounts = currentPositionCounts;
  commandedPositionTargetCounts = currentPositionCounts;
  resetPositionLoopStateForCurrentTarget();
}

void holdLeftWheelPositionTarget() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  float currentLeftCounts = (float)leftCount;
  targetPositionCounts = currentLeftCounts;
  commandedPositionTargetCounts = currentLeftCounts;
  measuredPositionCounts = currentLeftCounts;
  lastMeasuredPositionCounts = currentLeftCounts;
  measuredPositionVelocityCountsPerSec = 0.0;
  positionErrorCounts = 0.0;
  positionIntegralError = 0.0;
  positionPAngleDeg = 0.0;
  positionIAngleDeg = 0.0;
  positionDAngleDeg = 0.0;
  positionAngleOffsetDeg = 0.0;
  lastPositionSampleMs = millis();
}

void resetHeadingControlMemory() {
  yawRateDps = 0.0;
  yawAngleDeg = 0.0;
  yawTargetHeadingDeg = 0.0;
  yawHeadingErrorDeg = 0.0;
  yawCorrectionPwm = 0.0;
  yawTurnActive = false;
  yawTurnSettledStartMs = 0;
  yawTurnRecoveryUntilMs = 0;
  yawTurnCorrectionPwm = 0.0;
  yawTurnStartLeftCount = 0;
  yawTurnStartRightCount = 0;
  yawTurnStartCenterCounts = 0.0;
  yawTurnLastCenterCounts = 0.0;
  yawTurnLastCenterSampleMs = 0;
  yawTurnCenterErrorCounts = 0.0;
  yawTurnCenterVelocityCountsPerSec = 0.0;
  yawTurnCenterIntegralCountsSec = 0.0;
  yawTurnCenterLockCorrectionPwm = 0.0;
  yawTurnCommandSign = 1;
  yawTurnWheelSyncErrorCounts = 0.0;
  yawTurnWheelSyncVelocityCountsPerSec = 0.0;
  yawTurnWheelSyncCorrectionPwm = 0.0;
  leftTurnSequenceState = LEFT_SEQ_IDLE;
  leftTurnSequencePauseStartMs = 0;
  rightTurnSequenceState = RIGHT_SEQ_IDLE;
  rightTurnSequencePauseStartMs = 0;
}

void resetYawRateTransientMemory() {
  yawRateTransientErrorDps = 0.0;
  yawRateTransientCorrectionPwm = 0.0;
}

void resetControlMemory() {
  resetPositionLoopMemory();
  resetHeadingControlMemory();
  resetYawRateTransientMemory();
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
  float maxTargetStep =
    POSITION_TARGET_RAMP_COUNTS_PER_SEC * elapsedSeconds;
  float targetDelta =
    commandedPositionTargetCounts - targetPositionCounts;

  if (targetDelta > maxTargetStep) {
    targetPositionCounts += maxTargetStep;
  }
  else if (targetDelta < -maxTargetStep) {
    targetPositionCounts -= maxTargetStep;
  }
  else {
    targetPositionCounts = commandedPositionTargetCounts;
  }

  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  measuredPositionCounts = positionControlCountsForCurrentMode(leftCount, rightCount);
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

  float activePositionKp = positionKp;
  float activePositionKd = positionKd;
  float activeMaxPositionAngleOffsetDeg = MAX_POSITION_ANGLE_OFFSET_DEG;

  if (yawTurnActive && !YAW_TURN_LEFT_WHEEL_PIVOT_MODE) {
    activePositionKp = positionKp * YAW_TURN_POSITION_KP_MULT;
    activePositionKd = positionKd * YAW_TURN_POSITION_KD_MULT;
    activeMaxPositionAngleOffsetDeg = YAW_TURN_MAX_POSITION_ANGLE_OFFSET_DEG;
  }

  positionPAngleDeg = POSITION_LOOP_DIRECTION_SIGN * activePositionKp * positionErrorCounts;
  positionIAngleDeg = POSITION_LOOP_DIRECTION_SIGN * positionKi * positionIntegralError;
  positionDAngleDeg = POSITION_LOOP_DIRECTION_SIGN * -activePositionKd * measuredPositionVelocityCountsPerSec;

  float rawAngleOffset = positionPAngleDeg + positionIAngleDeg + positionDAngleDeg;

  positionAngleOffsetDeg = clampFloat(rawAngleOffset,
                                      -activeMaxPositionAngleOffsetDeg,
                                      activeMaxPositionAngleOffsetDeg);

  if (rawAngleOffset != positionAngleOffsetDeg && positionKi != 0.0) {
    positionIntegralError = previousIntegralError;
  }
}

// ---------------- Yaw heading / 45-degree turn control ----------------
void updateYawEstimate(float dt) {
  yawRateDps = gz - yawGyroOffset;
  yawAngleDeg = wrapAngleDeg(yawAngleDeg + yawRateDps * dt);
  yawHeadingErrorDeg = wrapAngleDeg(yawTargetHeadingDeg - yawAngleDeg);
}

bool yawTurnRecoveryActive() {
  return (long)(yawTurnRecoveryUntilMs - millis()) > 0;
}

void resetYawTurnCenterEncoderBalanceReference() {
  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  yawTurnStartLeftCount = leftCount;
  yawTurnStartRightCount = rightCount;
  yawTurnStartCenterCounts = averageEncoderCounts(leftCount, rightCount);
  yawTurnLastCenterCounts = yawTurnStartCenterCounts;
  yawTurnLastCenterSampleMs = millis();
  yawTurnCenterErrorCounts = 0.0;
  yawTurnCenterVelocityCountsPerSec = 0.0;
  yawTurnCenterIntegralCountsSec = 0.0;
  yawTurnCenterLockCorrectionPwm = 0.0;
  yawTurnWheelSyncErrorCounts = 0.0;
  yawTurnWheelSyncVelocityCountsPerSec = 0.0;
  yawTurnWheelSyncCorrectionPwm = 0.0;
}

void updateYawTurnCenterEncoderBalance() {
  yawTurnCenterErrorCounts = 0.0;
  yawTurnCenterVelocityCountsPerSec = 0.0;
  yawTurnCenterLockCorrectionPwm = 0.0;
  yawTurnWheelSyncVelocityCountsPerSec = 0.0;
  yawTurnWheelSyncCorrectionPwm = 0.0;

  if (YAW_TURN_LEFT_WHEEL_PIVOT_MODE) {
    return;
  }

  long leftCount = 0;
  long rightCount = 0;
  readEncoderCounts(&leftCount, &rightCount);

  unsigned long now = millis();
  float elapsedSeconds = (now - yawTurnLastCenterSampleMs) / 1000.0;

  float currentCenterCounts = averageEncoderCounts(leftCount, rightCount);
  if (elapsedSeconds > 0.001 && elapsedSeconds < 0.2) {
    yawTurnCenterVelocityCountsPerSec =
      (currentCenterCounts - yawTurnLastCenterCounts) / elapsedSeconds;
  }

  yawTurnLastCenterCounts = currentCenterCounts;
  yawTurnLastCenterSampleMs = now;

  // Center position error: for a true center pivot, encoder average should not move.
  yawTurnCenterErrorCounts = currentCenterCounts - yawTurnStartCenterCounts;

  if (elapsedSeconds > 0.001 && elapsedSeconds < 0.2) {
    yawTurnCenterIntegralCountsSec += yawTurnCenterErrorCounts * elapsedSeconds;
    yawTurnCenterIntegralCountsSec = clampFloat(
      yawTurnCenterIntegralCountsSec,
      -YAW_TURN_CENTER_LOCK_INTEGRAL_LIMIT,
      YAW_TURN_CENTER_LOCK_INTEGRAL_LIMIT
    );
  }

  if (YAW_TURN_CENTER_LOCK_ENABLED) {
    float rawCenterLockCorrection =
      YAW_TURN_CENTER_LOCK_DIRECTION_SIGN *
      (YAW_TURN_CENTER_LOCK_KP * yawTurnCenterErrorCounts +
       YAW_TURN_CENTER_LOCK_KI * yawTurnCenterIntegralCountsSec +
       YAW_TURN_CENTER_LOCK_KD * yawTurnCenterVelocityCountsPerSec);

    yawTurnCenterLockCorrectionPwm = clampFloat(
      rawCenterLockCorrection,
      -MAX_YAW_TURN_CENTER_LOCK_PWM,
      MAX_YAW_TURN_CENTER_LOCK_PWM
    );
  }

  // Wheel-sync error: compare how far each wheel has travelled in the turning direction.
  // For a center pivot:
  //   left turn:  leftDelta should be negative, rightDelta should be positive
  //   right turn: leftDelta should be positive, rightDelta should be negative
  // After multiplying by yawTurnCommandSign, both travel values should be positive and equal.
  if (YAW_TURN_WHEEL_SYNC_ENABLED) {
    float leftDeltaCounts = (float)(leftCount - yawTurnStartLeftCount);
    float rightDeltaCounts = (float)(rightCount - yawTurnStartRightCount);

    float leftTurnTravelCounts = -yawTurnCommandSign * leftDeltaCounts;
    float rightTurnTravelCounts = yawTurnCommandSign * rightDeltaCounts;

    float previousSyncErrorCounts = yawTurnWheelSyncErrorCounts;
    yawTurnWheelSyncErrorCounts = rightTurnTravelCounts - leftTurnTravelCounts;

    if (elapsedSeconds > 0.001 && elapsedSeconds < 0.2) {
      yawTurnWheelSyncVelocityCountsPerSec =
        (yawTurnWheelSyncErrorCounts - previousSyncErrorCounts) / elapsedSeconds;
    }

    float rawWheelSyncCorrection =
      YAW_TURN_WHEEL_SYNC_DIRECTION_SIGN *
      (YAW_TURN_WHEEL_SYNC_KP * yawTurnWheelSyncErrorCounts +
       YAW_TURN_WHEEL_SYNC_KD * yawTurnWheelSyncVelocityCountsPerSec);

    yawTurnWheelSyncCorrectionPwm = clampFloat(
      rawWheelSyncCorrection,
      -MAX_YAW_TURN_WHEEL_SYNC_PWM,
      MAX_YAW_TURN_WHEEL_SYNC_PWM
    );
  }
}
void cancelYawTurnAndHoldCurrentHeading() {
  bool wasTurning = yawTurnActive;
  yawTurnActive = false;
  yawTurnSettledStartMs = 0;
  yawTurnCorrectionPwm = 0.0;
  yawTurnCenterLockCorrectionPwm = 0.0;
  yawTurnCenterErrorCounts = 0.0;
  yawTurnCenterVelocityCountsPerSec = 0.0;
  yawTurnCenterIntegralCountsSec = 0.0;
  yawTurnWheelSyncErrorCounts = 0.0;
  yawTurnWheelSyncVelocityCountsPerSec = 0.0;
  yawTurnWheelSyncCorrectionPwm = 0.0;
  yawTargetHeadingDeg = yawAngleDeg;
  yawHeadingErrorDeg = 0.0;

  if (wasTurning) {
    holdCurrentPositionTarget();
  }
}

void startYawTurnRelativeDeg(float deltaDeg) {
  if (!YAW_TURN_ENABLED) {
    return;
  }

  // Center pivot: keep the robot center position fixed while the two wheels rotate opposite ways.
  holdCurrentPositionTarget();
  yawTurnCommandSign = (deltaDeg >= 0.0) ? 1 : -1;
  resetYawTurnCenterEncoderBalanceReference();

  yawTargetHeadingDeg = wrapAngleDeg(yawAngleDeg + deltaDeg);
  yawHeadingErrorDeg = wrapAngleDeg(yawTargetHeadingDeg - yawAngleDeg);
  yawTurnActive = true;
  yawTurnRecoveryUntilMs = 0;
  yawTurnSettledStartMs = 0;
  yawTurnCorrectionPwm = 0.0;
}

void cancelLeftTurnSequence() {
  leftTurnSequenceState = LEFT_SEQ_IDLE;
  leftTurnSequencePauseStartMs = 0;
}

void cancelRightTurnSequence() {
  rightTurnSequenceState = RIGHT_SEQ_IDLE;
  rightTurnSequencePauseStartMs = 0;
}

bool leftTurnFinalPauseStillRequired() {
  if (leftTurnSequenceState != LEFT_SEQ_FINAL_PAUSE) {
    return false;
  }

  return millis() - leftTurnSequencePauseStartMs < TURN_MIN_OPPOSITE_COMMAND_WAIT_MS;
}

bool rightTurnFinalPauseStillRequired() {
  if (rightTurnSequenceState != RIGHT_SEQ_FINAL_PAUSE) {
    return false;
  }

  return millis() - rightTurnSequencePauseStartMs < TURN_MIN_OPPOSITE_COMMAND_WAIT_MS;
}

void startLeftTurn90Sequence() {
  if (!AUTO_LEFT_90_SEQUENCE_ENABLED) {
    startYawTurnRelativeDeg(BLE_LEFT_TURN_SIGN * BLE_LEFT_FIRST_TURN_TARGET_DEG);
    return;
  }

  cancelLeftTurnSequence();
  cancelRightTurnSequence();
  holdCurrentPositionTarget();
  leftTurnSequenceState = LEFT_SEQ_FIRST_TURN;
  leftTurnSequencePauseStartMs = 0;
  startYawTurnRelativeDeg(BLE_LEFT_TURN_SIGN * BLE_LEFT_FIRST_TURN_TARGET_DEG);
}

void startRightTurn90Sequence() {
  if (!AUTO_RIGHT_90_SEQUENCE_ENABLED) {
    startYawTurnRelativeDeg(-BLE_LEFT_TURN_SIGN * BLE_RIGHT_FIRST_TURN_TARGET_DEG);
    return;
  }

  cancelRightTurnSequence();
  cancelLeftTurnSequence();
  holdCurrentPositionTarget();
  rightTurnSequenceState = RIGHT_SEQ_FIRST_TURN;
  rightTurnSequencePauseStartMs = 0;
  startYawTurnRelativeDeg(-BLE_LEFT_TURN_SIGN * BLE_RIGHT_FIRST_TURN_TARGET_DEG);
}

void updateLeftTurn90Sequence() {
  if (!AUTO_LEFT_90_SEQUENCE_ENABLED || leftTurnSequenceState == LEFT_SEQ_IDLE) {
    return;
  }

  unsigned long now = millis();

  if (leftTurnSequenceState == LEFT_SEQ_FIRST_TURN) {
    if (!yawTurnActive) {
      holdCurrentPositionTarget();
      leftTurnSequenceState = LEFT_SEQ_PAUSE_BETWEEN;
      leftTurnSequencePauseStartMs = now;
    }
  }
  else if (leftTurnSequenceState == LEFT_SEQ_PAUSE_BETWEEN) {
    if (now - leftTurnSequencePauseStartMs >= TURN_INTER_STEP_PAUSE_MS) {
      leftTurnSequenceState = LEFT_SEQ_SECOND_TURN;
      leftTurnSequencePauseStartMs = 0;
      startYawTurnRelativeDeg(BLE_LEFT_TURN_SIGN * BLE_LEFT_SECOND_TURN_TARGET_DEG);
    }
  }
  else if (leftTurnSequenceState == LEFT_SEQ_SECOND_TURN) {
    if (!yawTurnActive) {
      holdCurrentPositionTarget();
      leftTurnSequenceState = LEFT_SEQ_FINAL_PAUSE;
      leftTurnSequencePauseStartMs = now;
    }
  }
  else if (leftTurnSequenceState == LEFT_SEQ_FINAL_PAUSE) {
    if (now - leftTurnSequencePauseStartMs >= TURN_FINAL_PAUSE_MS) {
      cancelLeftTurnSequence();
      holdCurrentPositionTarget();
    }
  }
}

void updateRightTurn90Sequence() {
  if (!AUTO_RIGHT_90_SEQUENCE_ENABLED || rightTurnSequenceState == RIGHT_SEQ_IDLE) {
    return;
  }

  unsigned long now = millis();

  if (rightTurnSequenceState == RIGHT_SEQ_FIRST_TURN) {
    if (!yawTurnActive) {
      holdCurrentPositionTarget();
      rightTurnSequenceState = RIGHT_SEQ_PAUSE_BETWEEN;
      rightTurnSequencePauseStartMs = now;
    }
  }
  else if (rightTurnSequenceState == RIGHT_SEQ_PAUSE_BETWEEN) {
    if (now - rightTurnSequencePauseStartMs >= TURN_INTER_STEP_PAUSE_MS) {
      rightTurnSequenceState = RIGHT_SEQ_SECOND_TURN;
      rightTurnSequencePauseStartMs = 0;
      startYawTurnRelativeDeg(-BLE_LEFT_TURN_SIGN * BLE_RIGHT_SECOND_TURN_TARGET_DEG);
    }
  }
  else if (rightTurnSequenceState == RIGHT_SEQ_SECOND_TURN) {
    if (!yawTurnActive) {
      holdCurrentPositionTarget();
      rightTurnSequenceState = RIGHT_SEQ_FINAL_PAUSE;
      rightTurnSequencePauseStartMs = now;
    }
  }
  else if (rightTurnSequenceState == RIGHT_SEQ_FINAL_PAUSE) {
    if (now - rightTurnSequencePauseStartMs >= TURN_FINAL_PAUSE_MS) {
      cancelRightTurnSequence();
      holdCurrentPositionTarget();
    }
  }
}

void applyYawTurnController(int *leftRawCommand, int *rightRawCommand) {
  yawTurnCorrectionPwm = 0.0;

  if (!yawTurnActive || !YAW_TURN_ENABLED) {
    return;
  }

  float absError = fabs(yawHeadingErrorDeg);
  float absRate = fabs(yawRateDps);

  bool isLeftTurnDirection = (yawTurnCommandSign > 0);
  bool isSecondTurn = (leftTurnSequenceState == LEFT_SEQ_SECOND_TURN ||
                       rightTurnSequenceState == RIGHT_SEQ_SECOND_TURN);

  float activeTurnKp = YAW_TURN_KP * (isLeftTurnDirection ? LEFT_TURN_KP_SCALE : RIGHT_TURN_KP_SCALE);
  float activeTurnKd = YAW_TURN_KD * (isLeftTurnDirection ? LEFT_TURN_KD_SCALE : RIGHT_TURN_KD_SCALE);
  float activeMaxYawTurnPwm = isLeftTurnDirection ? LEFT_MAX_YAW_TURN_PWM : RIGHT_MAX_YAW_TURN_PWM;
  float activeFastRateDps = isLeftTurnDirection ? LEFT_TURN_FAST_RATE_DPS : RIGHT_TURN_FAST_RATE_DPS;
  float activeBrakeKd = YAW_TURN_BRAKE_KD * (isLeftTurnDirection ? LEFT_TURN_BRAKE_KD_SCALE : RIGHT_TURN_BRAKE_KD_SCALE);
  float activeDoneRateDps = TURN_DONE_RATE_DPS;
  unsigned long activeSettleMs = TURN_SETTLE_MS;
  float activeStopBrakeMaxPwm = TURN_STOP_BRAKE_MAX_PWM;
  unsigned long activeRecoveryMs = isLeftTurnDirection ? LEFT_TURN_POST_RECOVERY_MS : RIGHT_TURN_POST_RECOVERY_MS;

  if (isSecondTurn) {
    if (isLeftTurnDirection) {
      activeTurnKp = YAW_TURN_KP * LEFT_SECOND_TURN_KP_SCALE;
      activeTurnKd = YAW_TURN_KD * LEFT_SECOND_TURN_KD_SCALE;
      activeMaxYawTurnPwm = LEFT_SECOND_MAX_YAW_TURN_PWM;
      activeFastRateDps = LEFT_SECOND_TURN_FAST_RATE_DPS;
      activeBrakeKd = YAW_TURN_BRAKE_KD * LEFT_SECOND_TURN_BRAKE_KD_SCALE;
      activeDoneRateDps = LEFT_SECOND_TURN_DONE_RATE_DPS;
      activeSettleMs = LEFT_SECOND_TURN_SETTLE_MS;
      activeStopBrakeMaxPwm = LEFT_SECOND_TURN_STOP_BRAKE_MAX_PWM;
      activeRecoveryMs = LEFT_SECOND_TURN_POST_RECOVERY_MS;
    } else {
      activeTurnKp = YAW_TURN_KP * RIGHT_SECOND_TURN_KP_SCALE;
      activeTurnKd = YAW_TURN_KD * RIGHT_SECOND_TURN_KD_SCALE;
      activeMaxYawTurnPwm = RIGHT_SECOND_MAX_YAW_TURN_PWM;
      activeFastRateDps = RIGHT_SECOND_TURN_FAST_RATE_DPS;
      activeBrakeKd = YAW_TURN_BRAKE_KD * RIGHT_SECOND_TURN_BRAKE_KD_SCALE;
      activeDoneRateDps = RIGHT_SECOND_TURN_DONE_RATE_DPS;
      activeSettleMs = RIGHT_SECOND_TURN_SETTLE_MS;
      activeStopBrakeMaxPwm = RIGHT_SECOND_TURN_STOP_BRAKE_MAX_PWM;
      activeRecoveryMs = RIGHT_SECOND_TURN_POST_RECOVERY_MS;
    }
  }

  // Finish only after the yaw rate has dropped enough.
  // This is extra important for the second right turn, which was falling at release.
  if (absError <= YAW_TURN_STOP_ERROR_DEG && absRate <= activeDoneRateDps) {
    if (yawTurnSettledStartMs == 0) {
      yawTurnSettledStartMs = millis();
    }

    if (millis() - yawTurnSettledStartMs >= activeSettleMs) {
      yawTurnActive = false;
      yawTurnRecoveryUntilMs = millis() + activeRecoveryMs;
      yawTurnCorrectionPwm = 0.0;
      yawTurnCenterLockCorrectionPwm = 0.0;
      yawTurnCenterErrorCounts = 0.0;
      yawTurnCenterVelocityCountsPerSec = 0.0;
      yawTurnCenterIntegralCountsSec = 0.0;
      yawTurnWheelSyncErrorCounts = 0.0;
      yawTurnWheelSyncVelocityCountsPerSec = 0.0;
      yawTurnWheelSyncCorrectionPwm = 0.0;
      yawTargetHeadingDeg = yawAngleDeg;  // Hold the actual finished angle, not fight back.
      yawHeadingErrorDeg = 0.0;
      holdCurrentPositionTarget();        // Keep the finished center position as the new hold target.

      return;
    }
  } else {
    yawTurnSettledStartMs = 0;
  }

  float rawCorrection =
    YAW_TURN_DIRECTION_SIGN *
    (activeTurnKp * yawHeadingErrorDeg - activeTurnKd * yawRateDps);

  // When close to the target, reduce maximum turn power so it does not shoot past 45 deg.
  float maxAllowedTurnPwm = activeMaxYawTurnPwm;
  if (absError < YAW_TURN_SLOW_ZONE_DEG) {
    maxAllowedTurnPwm = 4.0 + (activeMaxYawTurnPwm - 4.0) * absError / YAW_TURN_SLOW_ZONE_DEG;
  }

  // If it is already spinning quickly near the target, prioritize braking over adding more turn.
  float rateTowardTarget = (yawHeadingErrorDeg >= 0.0) ? yawRateDps : -yawRateDps;
  if (absError < YAW_TURN_SLOW_ZONE_DEG && rateTowardTarget > activeFastRateDps) {
    rawCorrection = -YAW_TURN_DIRECTION_SIGN * activeBrakeKd * yawRateDps;
  }

  yawTurnCorrectionPwm = clampFloat(
    rawCorrection,
    -maxAllowedTurnPwm,
    maxAllowedTurnPwm
  );

  // Important: no forced minimum PWM near the target.
  // addDeadband() turns even tiny raw commands into a large motor command, causing oscillation.
  // For right turns, keep a small controlled brake until yaw rate is low enough to release.
  if (absError <= YAW_TURN_STOP_ERROR_DEG) {
    if (absRate > activeDoneRateDps) {
      float stopBrake = -YAW_TURN_DIRECTION_SIGN * activeBrakeKd * yawRateDps;
      yawTurnCorrectionPwm = clampFloat(stopBrake, -activeStopBrakeMaxPwm, activeStopBrakeMaxPwm);
    } else {
      yawTurnCorrectionPwm = 0.0;
    }
  }

  if (YAW_TURN_LEFT_WHEEL_PIVOT_MODE) {
    // Left wheel acts as the pivot: do not add extra yaw-turn PWM to it.
    // Right wheel gets about 2x correction so the yaw torque stays similar to center-pivot mode.
    *rightRawCommand =
      (int)clampFloat(
        *rightRawCommand + YAW_TURN_PIVOT_MOVING_WHEEL_GAIN * yawTurnCorrectionPwm,
        -255.0,
        255.0
      );
  } else {
    // Center pivot: left and right wheels get opposite yaw commands.
    // The physical center is approximated by the encoder average.
    // A common correction is then applied to both wheels to keep that average fixed.
    updateYawTurnCenterEncoderBalance();

    float leftTurnPwm = YAW_TURN_LEFT_EXTRA_PWM_SCALE * yawTurnCorrectionPwm;
    float rightTurnPwm = YAW_TURN_RIGHT_EXTRA_PWM_SCALE * yawTurnCorrectionPwm;

    // Center lock correction has priority: it pushes both wheels back toward the starting center position.
    // This is what makes the robot rotate about its own center instead of drifting forward/backward.
    float centerLockCommonPwm = -yawTurnCenterLockCorrectionPwm;

    // Wheel-sync correction is the main fix for the problem where the right wheel travels much more.
    // If right travel > left travel, yawTurnWheelSyncCorrectionPwm is positive.
    // For a left turn, this subtracts PWM from both sides: left becomes more backward, right becomes less forward.
    // For a right turn, this adds PWM to both sides: left becomes more forward, right becomes less backward.
    float wheelSyncCommonPwm = -yawTurnCommandSign * yawTurnWheelSyncCorrectionPwm;

    *leftRawCommand =
      (int)clampFloat(
        *leftRawCommand - leftTurnPwm + centerLockCommonPwm + wheelSyncCommonPwm,
        -255.0,
        255.0
      );

    *rightRawCommand =
      (int)clampFloat(
        *rightRawCommand + rightTurnPwm + centerLockCommonPwm + wheelSyncCommonPwm,
        -255.0,
        255.0
      );
  }
}

// ---------------- Yaw-rate transient damping ----------------
void applyYawRateTransientDamping(int *leftRawCommand, int *rightRawCommand) {
  yawRateTransientErrorDps = -yawRateDps;
  yawRateTransientCorrectionPwm = 0.0;

  if (!YAW_RATE_TRANSIENT_DAMPING_ENABLED) {
    return;
  }

  int leftMagnitude = abs(*leftRawCommand);
  int rightMagnitude = abs(*rightRawCommand);
  if (leftMagnitude < YAW_RATE_TRANSIENT_MIN_RAW_PWM &&
      rightMagnitude < YAW_RATE_TRANSIENT_MIN_RAW_PWM) {
    return;
  }

  if (*leftRawCommand != 0 && *rightRawCommand != 0 &&
      ((*leftRawCommand > 0) != (*rightRawCommand > 0))) {
    return;
  }

  if (fabs(yawRateTransientErrorDps) <= YAW_RATE_TRANSIENT_DEADBAND_DPS) {
    return;
  }

  float rawCorrection =
    YAW_RATE_TRANSIENT_DIRECTION_SIGN
    * YAW_RATE_TRANSIENT_KP
    * yawRateTransientErrorDps;

  float maxAllowedCorrection = MAX_YAW_RATE_TRANSIENT_CORRECTION_PWM;
  float smallerDriveMagnitude = min(leftMagnitude, rightMagnitude);
  float sameDirectionLimit =
    smallerDriveMagnitude > 1.0 ? smallerDriveMagnitude - 1.0 : 0.0;
  if (sameDirectionLimit < maxAllowedCorrection) {
    maxAllowedCorrection = sameDirectionLimit;
  }

  yawRateTransientCorrectionPwm =
    clampFloat(
      rawCorrection,
      -maxAllowedCorrection,
      maxAllowedCorrection
    );

  *leftRawCommand =
    (int)clampFloat(
      *leftRawCommand - yawRateTransientCorrectionPwm,
      -255.0,
      255.0
    );

  *rightRawCommand =
    (int)clampFloat(
      *rightRawCommand + yawRateTransientCorrectionPwm,
      -255.0,
      255.0
    );
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

void applyYawHeadingHold(int rawCommand, float dt, int *leftRawCommand, int *rightRawCommand) {
  (void)dt;
  yawCorrectionPwm = 0.0;

  const float driveMagnitude = abs(rawCommand);
  if (driveMagnitude >= YAW_CONTROL_MIN_RAW_PWM &&
      fabs(yawHeadingErrorDeg) > YAW_HEADING_DEADBAND_DEG) {
    yawCorrectionPwm = YAW_CONTROL_DIRECTION_SIGN * YAW_HEADING_KP * yawHeadingErrorDeg;

    float maxAllowedCorrection = MAX_YAW_CORRECTION_PWM;
    const float sameDirectionLimit = driveMagnitude > 1.0 ? driveMagnitude - 1.0 : 0.0;
    if (sameDirectionLimit < maxAllowedCorrection) {
      maxAllowedCorrection = sameDirectionLimit;
    }

    yawCorrectionPwm = clampFloat(yawCorrectionPwm,
                                  -maxAllowedCorrection,
                                  maxAllowedCorrection);
  }

  *leftRawCommand = (int)clampFloat(rawCommand - yawCorrectionPwm, -255.0, 255.0);
  *rightRawCommand = (int)clampFloat(rawCommand + yawCorrectionPwm, -255.0, 255.0);
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
  Serial.print(" yawRate=");
  Serial.print(yawRateDps, 2);
  Serial.print(" yawRateErr=");
  Serial.print(yawRateTransientErrorDps, 2);
  Serial.print(" yawPWM=");
  Serial.print(yawRateTransientCorrectionPwm, 2);
  Serial.print(" yawAngle=");
  Serial.print(yawAngleDeg, 2);
  Serial.print(" yawTarget=");
  Serial.print(yawTargetHeadingDeg, 2);
  Serial.print(" yawErr=");
  Serial.print(yawHeadingErrorDeg, 2);
  Serial.print(" turnActive=");
  Serial.print(yawTurnActive ? 1 : 0);
  Serial.print(" turnPWM=");
  Serial.print(yawTurnCorrectionPwm, 2);
  Serial.print(" turnRecovery=");
  Serial.print(yawTurnRecoveryActive() ? 1 : 0);
  Serial.print(" centerErr=");
  Serial.print(yawTurnCenterErrorCounts, 1);
  Serial.print(" centerVel=");
  Serial.print(yawTurnCenterVelocityCountsPerSec, 1);
  Serial.print(" centerPWM=");
  Serial.print(yawTurnCenterLockCorrectionPwm, 2);
  Serial.print(" syncErr=");
  Serial.print(yawTurnWheelSyncErrorCounts, 1);
  Serial.print(" syncPWM=");
  Serial.println(yawTurnWheelSyncCorrectionPwm, 2);
}

void sendTelemetryIfDue(float targetAngleDeg, float tiltAngleDeg, float gyroRate, int rawCommand,
                        int leftCommand, int rightCommand, float dt, bool force) {
  static unsigned long lastTelemetryPrint = 0;

  (void)dt;

  unsigned long now = millis();
  if (lastTelemetryPrint == 0) {
    lastTelemetryPrint = now;
  }

  unsigned long elapsedMs = now - lastTelemetryPrint;
  if (!force && elapsedMs <= DEBUG_PRINT_INTERVAL_MS) {
    return;
  }

  if (DEBUG_PRINT_ENABLED) {
    printDebugLine(
      tiltAngleDeg,
      gyroRate,
      targetAngleDeg,
      rawCommand,
      leftCommand,
      rightCommand,
      measuredControlFrequencyHz
    );
  }

  lastTelemetryPrint = now;
}

void updateControlFrequencyMonitor() {
  unsigned long now = millis();

  if (controlFrequencyWindowStartMs == 0) {
    controlFrequencyWindowStartMs = now;
    controlFrequencySampleCount = 0;
  }

  controlFrequencySampleCount++;

  unsigned long elapsedMs =
    now - controlFrequencyWindowStartMs;

  if (elapsedMs < CONTROL_FREQUENCY_WINDOW_MS) {
    return;
  }

  measuredControlFrequencyHz =
    controlFrequencySampleCount * 1000.0 / elapsedMs;

  controlFrequencySampleCount = 0;
  controlFrequencyWindowStartMs = now;
}

// ---------------- BLE command layer ----------------
void setupBle() {
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
}

void writeBleAck(const char* text) {
  if (!bleReady) {
    return;
  }

  customCharacteristic.writeValue(text);
}

void handleBleCommandChar(char command) {
  if (command >= 'a' && command <= 'z') {
    command = command - 'a' + 'A';
  }

  if (command == 'F' || command == 'W') {
    cancelLeftTurnSequence();
    cancelRightTurnSequence();
    movePositionTargetRelativeCounts(BLE_POSITION_TARGET_COUNTS);
    writeBleAck("Forward");
  }
  else if (command == 'B') {
    cancelLeftTurnSequence();
    cancelRightTurnSequence();
    movePositionTargetRelativeCounts(-BLE_POSITION_TARGET_COUNTS);
    writeBleAck("Backward");
  }
  else if (command == 'L' || command == 'A') {
    if (rightTurnFinalPauseStillRequired()) {
      holdCurrentPositionTarget();
      writeBleAck("WaitPause");
    } else {
      startLeftTurn90Sequence();
      writeBleAck(AUTO_LEFT_90_SEQUENCE_ENABLED ? "Left90Auto" : "Left45");
    }
  }
  else if (command == 'R' || command == 'D') {
    if (leftTurnFinalPauseStillRequired()) {
      holdCurrentPositionTarget();
      writeBleAck("WaitPause");
    } else {
      startRightTurn90Sequence();
      writeBleAck(AUTO_RIGHT_90_SEQUENCE_ENABLED ? "Right90Auto" : "Right45");
    }
  }
  else if (command == 'S' || command == 'X' || command == '0') {
    cancelLeftTurnSequence();
    cancelRightTurnSequence();
    holdCurrentPositionTarget();
    cancelYawTurnAndHoldCurrentHeading();
    writeBleAck("Hold");
  }
  else {
    writeBleAck("Bad");
  }
}

void pollBleCommandAfterControlLoop() {
  if (!bleReady) {
    return;
  }

  controlLoopsSinceBlePoll++;
  if (controlLoopsSinceBlePoll < BLE_CONTROL_LOOP_POLL_INTERVAL) {
    return;
  }

  controlLoopsSinceBlePoll = 0;

  BLE.poll();

  bool connected = BLE.connected();
  if (connected != bleCentralConnected) {
    bleCentralConnected = connected;
    digitalWrite(LED_BUILTIN, connected ? HIGH : LOW);

    if (!connected) {
      cancelLeftTurnSequence();
      cancelRightTurnSequence();
      holdCurrentPositionTarget();
      cancelYawTurnAndHoldCurrentHeading();
    }
  }

  if (!customCharacteristic.written()) {
    return;
  }

  int length = customCharacteristic.valueLength();
  const unsigned char* data = customCharacteristic.value();
  if (length <= 0 || data == nullptr) {
    return;
  }

  handleBleCommandChar((char)data[0]);
}

// ---------------- Arduino setup / loop ----------------
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

  // This initializes the BMI270 accel/gyro only; the BMM150 magnetometer is not used.
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

  Serial.println("# Start balance-position balancing.");
  Serial.println("# Position loop enabled. Speed loop disabled.");
  Serial.println("# Yaw heading hold enabled. Yaw 45-degree turn controller enabled. Yaw-rate transient damping enabled.");
  Serial.print("# Position target ramp cm/s: ");
  Serial.println(POSITION_TARGET_RAMP_SPEED_CM_PER_SEC, 1);
  Serial.print("# Control loop target Hz: ");
  Serial.println(1000000.0 / CONTROL_LOOP_TARGET_PERIOD_US, 1);
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
    updateControlFrequencyMonitor();

    if (safetyStopActive) {
      coastAllMotors();
      resetControlMemory();

      if (abs(tiltAngleDeg) >= SAFETY_RESTART_ANGLE_DEG) {
        const float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
        sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, dt, false);

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
      sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, dt, true);

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

      sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, 0, 0, 0, dt, false);
      pollBleCommandAfterControlLoop();
      return;
    }

    updatePositionLoop();

    float targetAngleDeg = BASE_TARGET_ANGLE_DEG + positionAngleOffsetDeg;
    int rawCommand = balancePidCommand(targetAngleDeg, tiltAngleDeg, gyroRate, dt);

    updateYawEstimate(dt);

    int leftRawCommand = 0;
    int rightRawCommand = 0;
    if (YAW_HEADING_HOLD_ENABLED && !yawTurnActive && !yawTurnRecoveryActive()) {
      applyYawHeadingHold(rawCommand, dt, &leftRawCommand, &rightRawCommand);
    } else {
      leftRawCommand = rawCommand;
      rightRawCommand = rawCommand;
      yawCorrectionPwm = 0.0;
    }

    applyYawTurnController(&leftRawCommand, &rightRawCommand);
    updateLeftTurn90Sequence();
    updateRightTurn90Sequence();

    if (!yawTurnActive && !yawTurnRecoveryActive()) {
      applyYawRateTransientDamping(&leftRawCommand, &rightRawCommand);
    }

    int leftCommand = addDeadband(leftRawCommand, leftDeadband);
    int rightCommand = addDeadband(rightRawCommand, rightDeadband);
    rightCommand = applyRightBackwardPwmCompensation(rightCommand);

    driveSigned(LEFT_IN1, LEFT_IN2, leftCommand);
    driveSigned(RIGHT_IN1, RIGHT_IN2, rightCommand);

    sendTelemetryIfDue(targetAngleDeg, tiltAngleDeg, gyroRate, rawCommand, leftCommand, rightCommand, dt, false);
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

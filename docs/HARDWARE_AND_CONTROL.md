# Hardware and control notes

[← Project overview](../README.md)

## Connections

Pin assignments for both sketches:

| Signal | Arduino pin |
| :--- | :--- |
| Left DRV8871 IN1 | D9 |
| Left DRV8871 IN2 | D10 |
| Right DRV8871 IN1 | D4 |
| Right DRV8871 IN2 | D6 |
| Left encoder A | D11 |
| Left encoder B | D12 |
| Right encoder A | D7 |
| Right encoder B | D8 |

The report's Figure 5 reverses the left/right driver pin labels relative to the sketches. Resolve physical motor orientation against these constants before uploading; do not rely on that figure alone.

The robot uses a 12 V power system, a shared ground, and 27 kΩ / 10 kΩ encoder voltage dividers (approximately 3.24 V from 12 V). The Nano's logic is 3.3 V; raw 12 V encoder signals must not connect directly to its inputs.

The IMU and BLE radio are onboard. The firmware uses `Wire1` for the BMI270 sensor, with a configured I²C clock of 1 MHz.

## Shared configuration

| Parameter | Value |
| :--- | :--- |
| Accelerometer / gyro output data rate | 400 Hz |
| Accelerometer range | ±4 g |
| Gyroscope range | ±2000°/s |
| Control scheduler period | 5556 µs, approximately 180 Hz |
| Position loop period | 50 ms, 20 Hz |
| Complementary filter coefficient | 0.98 |
| Gyro calibration sample count | 500 consecutive stable samples |
| Upright target trim | −0.5° |
| Balance PID gains | `Kp = 8.5`, `Ki = 65.0`, `Kd = 0.5` |
| Position gains | `Kp = 0.0016`, `Ki = 0`, `Kd = 0.0007` |
| Normal position lean-angle limit | ±3° |
| Position target ramp rate | 12 cm/s |
| Wheel diameter / encoder resolution | 8 cm / 1920 counts per revolution |
| Tilt cutoff / recovery threshold | 35° / less than 4° |
| IMU sample timeout | 100 ms |
| Serial baud rate | 115200 |
| BLE polling | Once per 20 control cycles, approximately 9 Hz at the target loop rate |

Encoder resolution and wheel diameter give `1920 / (π × 8) ≈ 76.4` counts/cm.

## Control structure

The complementary filter combines accelerometer tilt with integrated, bias-corrected gyro rate. The position loop uses the average signed left/right encoder counts to calculate a target lean offset. The faster balance PID tracks the trimmed upright angle plus that offset and produces a common motor command. Yaw control adds differential left/right output before motor deadband compensation.

The position integral gain is zero, so the position controller operates as **PD**. The balance derivative term uses gyro rate. Yaw is estimated by gyro integration and can drift over time.

### Baseline

- P-only yaw controller: `turningKp = 0.4`, output limited to ±35 PWM counts.
- Motor deadbands: **41 left / 42 right**.
- Preset ascent / 30-second hold / return sequence.
- No disconnect-triggered hold logic.

### Turning variant

- Direction-specific yaw PD and braking settings, heading hold, and yaw-rate transient damping.
- Automatic two-step turn sequences, with asymmetric right-turn setpoints.
- Motor deadbands: **42 left / 42 right**.
- Normal position offset limit ±3°, increased to ±3.2° during turns.
- Encoder center-lock and wheel-sync code are present but **disabled** by their feature constants.
- Disconnect cancels turning and requests position/heading hold.

## Startup and stop behavior

1. Motor outputs are initialized with the motors stopped.
2. IMU initialization and automatic gyro-bias calibration run. Calibration requires stillness; it does not require the chassis to be perfectly vertical.
3. To arm, hold the robot within 1° of its target angle with gyro rate below 25°/s for 300 ms.
4. Once armed, position, balance, and yaw control drive the motors.
5. Tilt beyond 35° stops motor output and resets control memory. After tilt returns below 4°, the system clears the stop and must satisfy arming conditions again.
6. More than 100 ms without a valid IMU sample stops the motors and resets arming/control state.

BLE initialization failure does not prevent local balancing. A BLE hold command continues active balancing; it is not a motor-power disconnect.

## Dependencies

- [Arduino_BMI270_BMM150](https://github.com/arduino-libraries/Arduino_BMI270_BMM150)
- [ArduinoBLE](https://github.com/arduino-libraries/ArduinoBLE)
- Arduino Mbed OS Nano Boards

The custom `FastImuClass` uses `BoschSensorClass` and Bosch driver structures from the IMU library.

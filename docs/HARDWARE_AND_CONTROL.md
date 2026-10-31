# Hardware and control notes

[← Project overview](../README.md)

## Connections

This table follows the pin constants in **both supplied sketches**.

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

The report describes a 12 V power system, a shared ground, and 27 kΩ / 10 kΩ encoder voltage dividers (approximately 3.24 V from 12 V). The Nano's logic is 3.3 V; raw 12 V encoder signals must not connect directly to its inputs. The report's power and wiring figures are project documentation, not a substitute for confirming the assembled circuit.

The IMU and BLE radio are onboard. The firmware uses `Wire1` for the BMI270 sensor, with a configured I²C clock of 1 MHz.

## Shared configuration

| Parameter | Supplied value |
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

The target loop rate is a scheduling configuration, not a fresh timing measurement. Encoder resolution and wheel diameter are used to compute `1920 / (π × 8) ≈ 76.4` counts/cm.

## Control structure

The complementary filter combines accelerometer tilt with integrated, bias-corrected gyro rate. The position loop uses the average signed left/right encoder counts to calculate a target lean offset. The faster balance PID tracks the trimmed upright angle plus that offset and produces a common motor command. Yaw control adds differential left/right output before motor deadband compensation.

The position integral gain is zero, so the supplied position controller operates as **PD**, despite PID naming in parts of the source. The balance derivative term uses gyro rate rather than a finite difference of angle error. Yaw is estimated by gyro integration; the sketches do not use the BMM150 magnetometer for absolute heading, so yaw can drift.

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

This variant should not be described as the report's P-only turning controller. Its additional tuning is preserved as a separate sketch; no independent results for those settings were supplied.

## Startup and stop behavior

1. Motor outputs are initialized with the motors stopped.
2. IMU initialization and automatic gyro-bias calibration run. Calibration requires stillness; it does not require the chassis to be perfectly vertical.
3. To arm, hold the robot within 1° of its target angle with gyro rate below 25°/s for 300 ms.
4. Once armed, position, balance, and yaw control drive the motors.
5. Tilt beyond 35° stops motor output and resets control memory. After tilt returns below 4°, the system clears the stop and must satisfy arming conditions again.
6. More than 100 ms without a valid IMU sample stops the motors and resets arming/control state.

BLE initialization failure does not prevent local balancing. A BLE hold command continues active balancing; it is not a motor-power disconnect.

## Report and firmware differences

The final report and supplied source do not describe precisely the same configuration. The operational documentation uses the source constants.

| Topic | Report | Supplied firmware |
| :--- | :--- | :--- |
| Balance-loop rate | Approximately 190 Hz | Target approximately 180 Hz |
| Tilt cutoff | 40° | 35° |
| Position angle offset limit | ±4° | Normally ±3°; turning variant allows ±3.2° during turns |
| Left / right driver pins | Figure 5 shows D4/D6 left, D9/D10 right | D9/D10 left, D4/D6 right |
| Turning controller | P-only, `Kp = 0.4` | Matches baseline; turning variant contains additional controllers |

## Build provenance

The two `.ino` files are byte-for-byte copies of the submitted sources. Original board-core/library versions were not provided. Both derive a custom `FastImuClass` from `BoschSensorClass` and use Bosch driver structures directly, which makes compatibility dependent on the selected `Arduino_BMI270_BMM150` implementation.

Install dependencies from the [official Arduino IMU library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150) and [ArduinoBLE](https://github.com/arduino-libraries/ArduinoBLE), then verify the selected sketch against the actual board. A successful compilation and a hardware retest have not been established for this repository import.

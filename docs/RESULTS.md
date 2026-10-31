# Final demo results and limitations

[← Project overview](../README.md)

## Demo performance

Results from the [final demo report](ELEC391_Final_Demo_Report.pdf), Section 4.3 / Table 10.

| Task | Result | Status |
| :--- | :--- | :--- |
| Forward 50 cm | Stops within ±3 cm | Met |
| Backward 50 cm | Stops within ±3 cm | Met |
| Individual 45° turn | Less than 2 s | Met |
| Left 90° turn | Within ±3° | Met |
| Right 90° turn | Approximately 10° heading error | Did not meet ±3° target |
| 5° ramp | Ascent, stop, descent while balancing | Met |
| 10° and 15° ramps | Ascent and stop; descent unreliable | Partially met |

Stationary balancing showed approximately ±3 cm of position oscillation.

## What improved during the project

| Observed problem | Engineering response |
| :--- | :--- |
| Drift and inconsistent short-duration balance | More rigid IMU attachment, automatic gyro calibration, then balance PID tuning |
| Weak or uneven low-speed motor response | Measure deadband on the assembled platform; compensate left/right motors separately |
| Position overshoot or poor stopping | Tune the encoder PD loop after stabilizing the inner loop |
| Changing balance after mechanical movement | Reinforce battery placement and keep the center of mass repeatable |
| BLE commands interfering with control | Periodic polling after control cycles and ramped position targets |

## Remaining work

- Improve right-turn calibration and account for motor, friction, and wheel-slip differences.
- Add downhill speed limiting or braking compensation for 10° and 15° ramps.
- Replace temporary IMU and battery attachments with more repeatable mounts.
- Record battery voltage and compensate motor output as the supply changes.
- Collect angle, encoder, yaw, PWM, and loop-frequency logs for repeatable tests.

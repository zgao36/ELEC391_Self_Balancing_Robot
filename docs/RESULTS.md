# Final demo results and limitations

[← Project overview](../README.md)

## Evidence

Results below are transcribed from the [final demo report](ELEC391_Final_Demo_Report.pdf), primarily Section 4.3 / Table 10 and Section 6.1. They describe the team's demo observations, not newly collected benchmark data. Trial counts, raw time series, and statistical confidence intervals were not supplied with the project files.

| Task | Reported result | Assessment in report |
| :--- | :--- | :--- |
| Forward 50 cm | Stops within ±3 cm | Met |
| Backward 50 cm | Stops within ±3 cm | Met |
| Individual 45° turn | Less than 2 s | Met |
| Left 90° turn | Within ±3° | Met |
| Right 90° turn | Approximately 10° heading error | Did not meet ±3° target |
| 5° ramp | Ascent, stop, descent while balancing | Met |
| 10° and 15° ramps | Ascent and stop; descent unreliable | Partially met |

The report's overview also describes approximately ±3 cm of stationary position oscillation. This is a separate observation from the ±3 cm stopping error of the 50 cm travel tests.

## Reading the report consistently

The overview says a 90° turn achieved heading error within 5°. The detailed turning tests and conclusions distinguish left turns (within ±3°) from right turns (approximately 10° error). This repository presents those detailed, direction-specific results.

Likewise, stability or ascent on a steeper ramp is not evidence of a complete ascent/stop/descent task. Only the 5° ramp is reported as completing the full sequence reliably.

The report describes the baseline P-only yaw controller. `TURNING.ino` contains additional tuning, but no separate test dataset was supplied for that variant. Its setpoints and recovery timers should not be read as measured performance.

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

These are improvement directions discussed in the report; they are not completed features.

## Public report copy

The public PDF removes student IDs from the cover while retaining the original technical text and figures. Report/source inconsistencies are documented in [hardware and control notes](HARDWARE_AND_CONTROL.md#report-and-firmware-differences).

<div align="center">

# Two-Wheel Self-Balancing Robot

**Balance. Move. Turn. Stay connected.**

A BLE-controlled inverted-pendulum robot built for **UBC ELEC 391 · Summer 2026 · Group 11**.

![Platform](https://img.shields.io/badge/Platform-Arduino_Nano_33_BLE_Sense-00878F?style=flat-square)
![Firmware](https://img.shields.io/badge/Firmware-Arduino_C%2B%2B-00599C?style=flat-square)
![Control](https://img.shields.io/badge/Control-PID_%2B_PD-325CC8?style=flat-square)
![Wireless](https://img.shields.io/badge/Wireless-Bluetooth_LE-5B4AE8?style=flat-square)

[Quick start](#quick-start) · [BLE commands](docs/BLE_COMMANDS.md) · [Hardware & control](docs/HARDWARE_AND_CONTROL.md) · [Demo results](docs/RESULTS.md) · [Final report](docs/ELEC391_Final_Demo_Report.pdf)

<img src="docs/assets/robot.jpg" alt="Top view of the assembled self-balancing robot, including its Arduino controller, motor drivers and chassis" width="680" />

*The assembled robot, photographed for the final demo report.*

</div>

## Overview

This two-wheel robot balances by continuously moving its wheels beneath its center of mass. An onboard BMI270 IMU estimates body tilt, wheel encoders measure travel, and cascaded controllers turn position commands into a target lean angle and motor PWM. A phone or computer sends movement commands over Bluetooth Low Energy.

The project combines embedded control with practical hardware work: securing the IMU, calibrating gyro bias, measuring motor deadbands, reinforcing the battery mount, and tuning the balance loop before position and turning control.

## Final demo highlights

| Task | Reported result |
| :--- | :--- |
| Forward and backward travel | 50 cm in each direction; stopping error within **±3 cm** |
| Individual 45° turns | **Less than 2 seconds** per turn |
| Left 90° turn | Final heading within **±3°** |
| Right 90° turn | Approximately **10° error**; accuracy target not met |
| 5° ramp | Completed ascent, stop, and descent while balancing |
| 10° and 15° ramps | Climbed and stopped; descent remained unreliable |

These are the team's results in the final report, Section 4.3 / Table 10 and Section 6.1, rather than new tests of this repository. See [results and limitations](docs/RESULTS.md) for context.

## How it works

<p align="center">
  <img src="docs/assets/control-diagram.png" alt="Original control architecture diagram from the final report" width="760" />
</p>

- **Tilt estimation:** 400 Hz accelerometer/gyroscope configuration, 500-sample automatic bias calibration, and a complementary filter with `K = 0.98`.
- **Balance:** inner PID loop, scheduled at approximately **180 Hz** in the supplied firmware.
- **Position:** a **20 Hz** encoder-based PD loop sets a bounded lean-angle offset; position targets ramp at 12 cm/s.
- **Turning:** differential motor commands provide yaw control. The baseline uses a P controller; the turning variant adds direction-specific PD control and automatic two-step turns.
- **Wireless commands:** BLE is polled after every 20 control cycles so command handling stays outside the inner control calculation.

The source files are preserved unchanged. The report describes some earlier or different settings; [hardware and control notes](docs/HARDWARE_AND_CONTROL.md#report-and-firmware-differences) explain the differences.

## Hardware

| Component | Purpose |
| :--- | :--- |
| Arduino Nano 33 BLE Sense with BMI270 IMU | Control, inertial sensing, and built-in BLE |
| Two DC gear motors with quadrature encoders | Actuation and wheel-position feedback |
| Two DRV8871 motor drivers | Bidirectional PWM motor drive |
| 12 V supply / battery pack | Robot power |
| 8 cm wheels and two-level chassis | Two-wheel inverted-pendulum platform |
| Encoder voltage dividers | Interface encoder signals to 3.3 V logic |

The supplied `Arduino_BMI270_BMM150` dependency targets the **Nano 33 BLE Sense Rev2** sensor combination. Confirm the board revision before building; an original Sense board with a different IMU is not interchangeable with this sketch. See the [official board documentation](https://docs.arduino.cc/hardware/nano-33-ble-sense-rev2/) and [Arduino IMU library](https://github.com/arduino-libraries/Arduino_BMI270_BMM150).

## Firmware layout

```text
ELEC391_Self_Balancing_Robot/
├── firmware/
│   ├── final_requirement_balance_robot/
│   │   └── final_requirement_balance_robot.ino
│   └── TURNING/
│       └── TURNING.ino
├── docs/
│   ├── assets/                         # Original report photo and diagram
│   ├── BLE_COMMANDS.md
│   ├── HARDWARE_AND_CONTROL.md
│   ├── RESULTS.md
│   └── ELEC391_Final_Demo_Report.pdf    # Public copy with student IDs removed
└── README.md
```

| Sketch | Use it for | Main behavior |
| :--- | :--- | :--- |
| [`final_requirement_balance_robot`](firmware/final_requirement_balance_robot/final_requirement_balance_robot.ino) | Baseline balance, travel, and ramp tasks | Balance PID, position PD, P-only yaw, preset ramp sequence |
| [`TURNING`](firmware/TURNING/TURNING.ino) | Direction-specific turning experiments | Yaw PD, heading hold, rate damping, automatic two-step left/right turns |

Each folder is a separate Arduino sketch with its own `setup()` and `loop()`. Open and upload **one sketch at a time**.

## Quick start

1. Clone this repository:

   ```bash
   git clone https://github.com/zgao36/ELEC391_Self_Balancing_Robot.git
   ```

2. In Arduino IDE, install the **Arduino Mbed OS Nano Boards** package and select the board entry matching your Nano 33 BLE Sense hardware.
3. Install **Arduino_BMI270_BMM150** and **ArduinoBLE** through Library Manager.
4. Open `firmware/final_requirement_balance_robot/final_requirement_balance_robot.ino` for the baseline, or `firmware/TURNING/TURNING.ino` for the turning variant.
5. Check motor and encoder wiring against the [firmware pin table](docs/HARDWARE_AND_CONTROL.md#connections), then verify and upload the selected sketch.
6. Open Serial Monitor at **115200 baud**. Keep the robot stationary during automatic gyro calibration. After calibration, support it close to the upright target until the controller arms.
7. Connect a BLE client to **`BLE-DEVICE111`** and write a command to the custom characteristic described in [BLE commands](docs/BLE_COMMANDS.md).

The original project did not record board-core or library versions. The custom `FastImuClass` accesses Bosch driver interfaces, so library compatibility matters. This import has not been compiled or tested on hardware; the demo results above come from the report.

**Operating behavior:** `S` requests position/heading hold while balancing; it does not cut motor power. The baseline keeps its existing target if BLE disconnects. Both sketches include a 35° tilt cutoff and a 100 ms IMU timeout. Review the [startup and stop behavior](docs/HARDWARE_AND_CONTROL.md#startup-and-stop-behavior) before running.

## Team

| Team member | GitHub |
| :--- | :--- |
| Peter Gao | [@zgao36](https://github.com/zgao36) |
| Tony Zhao | — |
| Echo Zhou | — |
| Rita Chang | — |

Team names follow the final report. GitHub profile links and commit attribution for the other members can be added once their account details are available.

## Engineering lessons

- **Make the measurements trustworthy first.** A rigid IMU mount and repeatable calibration reduced drift before gain tuning.
- **Treat the motors separately.** Measured deadbands of 41 and 42 PWM counts improved low-speed response, but did not remove turning asymmetry.
- **Tune from the inside out.** Stabilize balance, then position, then turning and ramp behavior.
- **Separate climbing from descent.** Successful uphill motion did not establish reliable downhill speed control on steeper ramps.

## Project references

- [Final demo report](docs/ELEC391_Final_Demo_Report.pdf), UBC ELEC 391, Group 11, June 2026. Student IDs are removed from this public copy; the project text and figures are retained.
- [ELEC 291 Autonomous Coin-Picking Robot](https://github.com/zgao36/ELEC_291_Autonomous_Coin_Picking_Robot)
- [Peter Gao's portfolio](https://zgao36.github.io/#projects)

# BLE command reference

[← Project overview](../README.md)

Use a BLE client to write text/ASCII commands to the Arduino's built-in BLE radio.

## Connection

| Setting | Value |
| :--- | :--- |
| Advertised name | `BLE-DEVICE111` |
| Service UUID | `00000000-5EC4-4083-81CD-A10B8D5CF6EC` |
| Characteristic UUID | `00000001-5EC4-4083-81CD-A10B8D5CF6EC` |
| Properties | Read, Write, Notify |
| Maximum value length | 20 bytes |

Connect to the characteristic and write an ASCII command such as `F`. Commands are case-insensitive. Enable notifications for acknowledgements from the turning variant. The initial value is `Ready`.

## Baseline sketch

Source: [`final_requirement_balance_robot.ino`](../firmware/final_requirement_balance_robot/final_requirement_balance_robot.ino).

| Write | Action |
| :--- | :--- |
| `F` or `W` | Set a target approximately 50 cm forward from the current position |
| `B` | Set a target approximately 50 cm backward from the current position |
| `L` | Shift the yaw target by −45° |
| `R` | Shift the yaw target by +45° |
| `S`, `X`, or `0` | Cancel the ramp sequence and hold current position and heading |
| `P`, `M`, or `RAMP` | Start the preset ramp sequence |

The travel target is 3820 encoder counts, approximately 50 cm. Position targets ramp at 12 cm/s.

`RAMP` is recognized before the single-letter `R` command. The ramp sequence is:

1. Save the starting encoder position.
2. Request `start + 4000` counts, approximately 52.4 cm.
3. Once the target is reached within 230 counts (approximately 3 cm), hold for 30 seconds.
4. Return to the saved start position.

The sequence uses encoder position without slope detection. A travel or turning command cancels it. BLE disconnection leaves the baseline's active target unchanged.

## Turning variant

Source: [`TURNING.ino`](../firmware/TURNING/TURNING.ino).

| Write | Action | Acknowledgement |
| :--- | :--- | :--- |
| `F` or `W` | Target approximately 50 cm forward | `Forward` |
| `B` | Target approximately 50 cm backward | `Backward` |
| `L` or `A` | Start an automatic two-step left turn | `Left90Auto` |
| `R` or `D` | Start an automatic two-step right turn | `Right90Auto` |
| `S`, `X`, or `0` | Cancel turns and hold position/heading | `Hold` |
| Other input | No recognized command | `Bad` |

The parser uses only the first character of the written value. **Do not send `RAMP` to this variant: its first character is interpreted as a right-turn command.** There is no ramp-sequence command in this sketch.

The left sequence targets 45° + 45°. The right sequence targets 46° + 48° to compensate for undershoot. The inter-step pause is **2800 ms** and the final pause is **6000 ms**. Some opposite-direction commands during the final pause return `WaitPause`.

On BLE disconnection, this variant cancels turn sequences and requests position/heading hold.

## Hold and motor shutdown

`S` / `X` / `0` keep balancing active; they are not power-off commands. The sketches stop motor output on excessive tilt or an IMU timeout, and can arm again after the required recovery conditions. Use the physical power connection when motor power must be removed.

# HARDWARE — what this robot physically is

Single source of truth for boards, wiring, ports, and physical quirks.
Update this file whenever wiring or hardware changes.

## Overview

2WD differential-drive base:

| Part | Detail |
|------|--------|
| Compute | NVIDIA Jetson (this machine, Ubuntu / Linux tegra) |
| MCU (current) | **Teensy 4.1**, micro-ROS over USB serial |
| MCU (migration branch) | Nucleo-H753ZI (`claude/teensy-nucleo-migration-b9g26c`), FDCAN — see below |
| Motors | 2× CubeMars **AK10-9**, MIT mode over CAN @ 1 Mbps |
| Lidar | RPLIDAR on `/dev/ttyUSB0`, full 360° minus battery-box mask |
| IMU | MPU6050 (I2C on the MCU) |

## Serial / ports

- `/dev/ttyACM0` — MCU micro-ROS serial, **baud 921600** (set in
  `firmware/src/main.cpp`). Exactly one micro-ROS agent may hold it.
- `/dev/ttyUSB0` — RPLIDAR.
- Port **8000** — live map web page served by `robot.launch.py`
  (`http://<robot-ip>:8000`, or `ssh -L 8000:localhost:8000`). Used to set
  the AMCL initial pose and click nav goals.

## CAN bus (Teensy 4.1, current)

- CAN2: Teensy pin 0 = CRX2, pin 1 = CTX2 → 3.3 V transceiver
  (SN65HVD230 / TJA1051T/3) → motors, 1 Mbps.
- Transceiver mode pin MUST be tied low/enabled: SN65HVD230 **RS ≤ 1 kΩ to
  GND**; TJA1051 **S pin LOW**. Floating = silent mode = receive-only —
  this exact fault has eaten days.
- Motor CAN IDs: left = controller `0x68` (cmd `0x868`),
  right = `0x69` (cmd `0x869`).
- Link health is exposed live on `/motor_current`
  (see TROUBLESHOOTING.md → "CAN link health").

## CAN bus (Nucleo-H753ZI, migration branch)

Correct wiring per UM2407 Table 20 — straight-through, never crossed:

- transceiver RXD → **PD0** = CN9 pin 25 (D67)
- transceiver TXD → **PD1** = CN9 pin 27 (D66)
- CN9 odd-row end order: PF0(23), PD0(25), PD1(27), PG0(29)

Verified good on this board: HSE bypass 8 MHz clock config, FDCAN NBTP=0x500
(exact 1 Mbps), PD0/PD1 in AF9 push-pull. Firmware never recovers from
bus-off (INIT stays latched) — **always reset the board after fixing CAN
hardware**. Live-debug register recipes (openocd + ST-LINK, FDCAN TEST
register, listen-only pokes) are in TROUBLESHOOTING.md → "Nucleo FDCAN".

## Motors / firmware behavior

- Boot: firmware sends the MIT "enter motor mode" handshake to both motors
  **unconditionally**, braked immediately. A small boot twitch is expected.
  (Probe-and-skip was tried and is unsafe: AK10-9s stream status even when
  NOT in MIT mode.)
- Motor battery power-cycled while the MCU stays up → motors resume
  streaming on their own but may not be in MIT mode; **reboot the MCU**.
- Overcurrent latch: > `OVERCURRENT_AMPS` (8 A) for `OVERCURRENT_MS` (1 s)
  brakes both motors until a zero cmd_vel releases it (teleop space bar).
- Calibrated constants in `firmware/include/config.h` (hardware
  measurements — recalibrate, don't look up):
  - `ERPM_TO_WHEEL_RADPS = 0.01237` (tape test 2026-07-20, TESTING.md §4)
  - `WHEEL_DIAMETER`, `LR_WHEELS_DISTANCE` — measured on the robot
  - `LEFT_MOTOR_DIR = +1`, `RIGHT_MOTOR_DIR = -1`

## Physical blind zones & footprint facts

- The **battery box sits behind the robot center** and is masked out of
  `/scan` by `linorobot2_bringup/config/box_laser_filter.yaml`
  (base_footprint frame: x −1.0..−0.10 m, y ±0.5 m). Everything inside the
  box is a permanent lidar blind spot — this is why the nav recovery BT
  uses a short 0.15 m BackUp instead of the stock 0.30 m.
- Collision monitor footprint: circumscribed radius ≈ 0.39 m. Any scan
  return inside it (robot-mounted part not yet masked, or a real wall)
  zeroes ALL cmd_vel — see TROUBLESHOOTING.md.

## Flashing

- Teensy: `cd firmware && pio run -e teensy41 -t upload`. First attempt
  often fails (`error writing to Teensy`) — retry; confirm `[SUCCESS]` or
  the Teensy may be stranded in its bootloader (no `/dev/ttyACM0`).
- Flashing makes the USB device vanish, which **kills the micro-ROS agent
  and it does not respawn** — restart `robot.launch.py` after a flash.
- Nucleo: ST-LINK; `reset run` via openocd reboots firmware and the agent
  reconnects in ~2 s.

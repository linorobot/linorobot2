# Host simulation — boot must not move the robot, stalls must stop it

Compiles the **real** `include/ak10_mit.h` + `include/config.h` on your PC
(no Teensy, no PlatformIO) with stubbed `Arduino.h`/`FlexCAN_T4.h`, models the
two AK10-9s on the other end of the CAN bus, and replays the motor-related
boot/loop logic from `src/main.cpp` through four scenarios:

- **A. Teensy gets power while the motor battery is already on** (the case
  that used to jerk the robot on every boot/flash/USB replug) → asserts the
  MIT "enter motor mode" handshake is **never** sent, so no jerk.
- **B. Whole-robot cold power-on** → exactly one handshake per motor (the
  motor's own one-time enable transient; not removable from the Teensy side),
  ending in a brake hold.
- **C. Motor battery switched on after the Teensy** → the recovery check
  enables the motors within ~3 s and never re-sends the handshake once they
  stream feedback.
- **D. Teensy reboots mid-drive** → a brake frame goes out within 50 ms,
  before the CAN settle delay.
- **E. Wheel stall mid-drive** → sustained current above `OVERCURRENT_AMPS`
  for `OVERCURRENT_MS` latches a stop (drive commands refused, only brakes go
  out) until a zero cmd_vel releases it; a shorter spike must not latch.

Run it:

```bash
cd firmware/host_sim
g++ -std=c++14 -Wall -Wextra -Istub -I../include sim_boot.cpp -o sim_boot && ./sim_boot
```

Expected output ends with `ALL SCENARIOS PASS`.

Note: the `setup()`/`loop()` fragments in `sim_boot.cpp` are copied from
`src/main.cpp` — if you change the boot sequence there, mirror it here.

# TROUBLESHOOTING — symptom-first playbook

Every failure mode below was hit on real hardware and diagnosed the hard
way. Check here before re-deriving a diagnosis. Wiring/board facts live in
HARDWARE.md; per-change pass criteria live in TESTING.md.

## "Robot won't move" — check in THIS order

1. **Stacked / dead micro-ROS agents** (cheapest, most common):
   ```bash
   fuser -v /dev/ttyACM0        # must show exactly ONE holder
   ps aux | grep micro_ros_agent | grep -v grep
   ```
   Killing the `ros2 run micro_ros_agent ...` wrapper does NOT kill the
   agent binary — repeated test runs stack agents that each steal a share
   of serial RX bytes, mimicking a broken board (sessions taking ~20 s,
   dropping every ~2 s, topics flickering, cmd_vel ignored/delayed).
   Kill strays with `pkill -f micro_ros_agent` (exit 144 = it matched your
   own shell, not a failure). The agent also does NOT respawn if it dies
   (e.g. USB vanished during a flash): `/odom/unfiltered` silent while
   `/dev/ttyACM0` exists → restart the launch.

2. **CAN link health** — `/motor_current` carries diagnostics:
   ```bash
   ros2 topic echo /motor_current --once --field data
   # [0] left A  [1] right A  [2] frames rx since boot
   # [3] last raw CAN id      [4] TX err counter  [5] RX err counter
   ```
   Healthy: `[4]`=`[5]`=0, `[2]` climbing. `[4]` bouncing high with
   `[5]`=0 = TX wiring fault: commands die between MCU and bus while
   odometry still streams (receive path fine). With motor battery OFF
   check: TX wire to transceiver TXD, transceiver mode pin (must be
   low/enabled — floating = silent = RX-only!), VCC/GND, CANH/CANL +
   termination. CAN auto-retry masks a sick link — nonzero `[4]` deserves
   attention even when driving "works".

3. **Motor battery was power-cycled** while the MCU stayed up → motors
   stream feedback but aren't in MIT mode; they ignore every command.
   Reboot the MCU (reset button or re-flash).

4. **Overcurrent latch engaged**: watch `[0]`/`[1]` on `/motor_current`;
   after ~1 s above 8 A both motors brake and ignore cmd_vel until a zero
   command (teleop space) releases the latch.

5. **Autonomous only (teleop works)** → next section.

## Nav2 sends goals but `/cmd_vel` is silent

Root-caused 2026-07-19. Verify the chain stage by stage — record
`/cmd_vel_nav`, `/cmd_vel_smoothed`, `/cmd_vel`, `/odom`, `/rosout` while
sending a goal; per-stage message counts pinpoint the zeroing layer.

- **Collision monitor is the usual suspect.** FootprintApproach treats ANY
  scan return inside the ~0.39 m circumscribed footprint as collision at
  t=0 and scales every twist (forward, reverse, spin) to zero — it also
  explains failed Spin/BackUp recoveries. After `stop_pub_timeout` (2 s)
  it stops publishing entirely: **total silence on `/cmd_vel` is its
  permanent-stop signature.** `/collision_monitor_state` publishes only on
  state CHANGE, so echo may show nothing.
  Fix depends on the offending points: robot-mounted part → extend the
  mask in `box_laser_filter.yaml`; real obstacle → move the robot.
- **Startup deadlock (fixed, don't reintroduce):** controller accel clamp
  ≈ max speed + smoother deadband > first commanded step → robot never
  moves → odom stays 0 → clamp never opens. Keep controller accel limits
  well above the 0.05 m/s cap and deadband at 0.
- **Progress checker:** required_movement must be ≪ (max speed × timeout),
  or "Failed to make progress" loops are guaranteed. Current: 0.1 m / 15 s.
- **AMCL pose silently invalid:** hand-moving/lifting the robot produces no
  odom change — map walls shift ~0.5 m and planning goes insane. Re-set the
  pose via the map page after ANY manual move; `pose_keeper.py` restores
  the last pose only on startup.
- Stock BT BackUp (0.30 m) reverses into the battery-box blind zone —
  that's why `navigate_to_pose_doorway_recovery.xml` exists (0.15 m,
  recovery order: clear costmaps → short backup → wait → spin).

## ros2 CLI pitfalls on this Jetson

- `ros2 topic echo/hz` piped to a file buffers — killed by `timeout` it
  leaves an EMPTY file. Prefix with `PYTHONUNBUFFERED=1`.
- DDS discovery under load takes 2–3 s: `ros2 topic pub` injection tests
  must publish **≥ 10 s** or the subscriber never matches.
- `ros2 topic echo /scan` truncates `ranges` — use `--full-length`.
- The Jetson CPU is the budget: rf2o alone burns ~50% of a core; planner
  and controller rates in `navigation.yaml` were lowered deliberately.

## Nucleo FDCAN bring-up (migration branch)

Wiring truth in HARDWARE.md. Debug techniques proven on this board
(PlatformIO-bundled openocd + ST-LINK, works while firmware runs):

```bash
~/.platformio/packages/tool-openocd/bin/openocd -s ~/.platformio/packages/tool-openocd/openocd/scripts \
  -f interface/stlink.cfg -f target/stm32h7x.cfg -c init -c "mdw <addr>" -c shutdown
```

- FDCAN1 base `0x4000A000`: CCCR 0x18, ECR 0x40, PSR 0x44, RXF0S 0xA4,
  TXFQS 0xC4, TXBRP 0xCC, NBTP 0x1C, TEST 0x10. GPIOD base `0x58020C00`
  (MODER/PUPDR/IDR/BSRR at 0x00/0x0C/0x10/0x18).
- **Listen-only poke** (watch bus without bus-off risk): CCCR INIT+CCE →
  set MON (bit 5) → clear INIT.
- **Force-dominant TX-pin test**: CCCR=0x3 → 0x83 → TEST TX[6:5]=0b10
  drives the real TX pin through the AF path — verifies FDCAN→pin without
  firmware. TEST.RX (bit 7) is NOT trustworthy while INIT=1.
- **Internal loopback (TEST.LBCK) ignores the physical RX pin** — it tests
  peripheral + message RAM only; never use it to "prove" the analog loop.
- **Floating-pin detection**: enable internal pull-down via PUPDR; a driven
  push-pull line holds high, a floating pin collapses. When loopback-testing
  adjacent pins, park a pull-up on the observed pin — a floating no-pull
  input follows its neighbor capacitively and fakes a pass.
- **Error decoding**: Bit0 error (LEC=5) + TEC near 248/bus-off can ONLY be
  the local TX path (other nodes can't turn dominant into recessive).
  No-ACK gives LEC=3 and caps at TEC 128 (error passive), never bus-off.
- **Bit-rate downshift** (NBTP NBRP=7 → 125 kbps): if Bit0 persists at
  125 k, rule out slew/loop-delay entirely.
- Firmware never exits bus-off — reset the board after any CAN hardware fix.
- Serial symptoms that mimic all of this: stacked agents (section 1).

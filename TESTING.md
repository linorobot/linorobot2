# TESTING — hands-on verification of the consolidated changes

Step-by-step checks for every change merged from the `claude/*` branches and
added on top of them. Each section says **what changed**, **what pass looks
like**, and the exact commands.

Every robot terminal needs ROS sourced first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/linorobot2_ws/install/setup.bash
```

After pulling this branch, rebuild the workspace once so launch/config changes
are installed (firmware changes additionally need a flash, covered below):

```bash
cd ~/linorobot2_ws && colcon build --symlink-install && source install/setup.bash
```

---

## 1. Boot behavior (firmware)

**What changed (revised 2026-07-09 after hardware testing):** the Teensy
sends the MIT "enter motor mode" handshake to both motors **unconditionally**
at boot, braked immediately. The original probe-and-skip design (skip the
handshake if the motor already streams feedback) turned out to be unsafe on
real hardware: the AK10-9s stream their status frame even when NOT in MIT
mode, so the probe skipped the handshake after a motor-battery power-cycle
and the robot silently ignored every drive command. A small boot twitch is
expected and accepted.

**Revised 2026-07-20 (robot moved at boot):** the old sequence enabled both
motors, waited 50 ms, then sent a single brake — so for 50 ms each motor ran
whatever MIT command it had latched, unopposed, and the robot visibly rolled
during boot. Now each motor's brake frame goes out in the **same millisecond**
as its enable handshake, and the brake is re-sent every `BOOT_BRAKE_PERIOD_MS`
(10 ms) for `BOOT_BRAKE_HOLD_MS` (300 ms) while the enable transient damps
out (constants in `firmware/include/config.h`). The host sim
(`firmware/host_sim/`) now fails if any enable is not braked in the same
millisecond or the hold window shrinks.

**Flash the firmware** (from the Jetson, Teensy on USB):

```bash
cd ~/Desktop/linorobot2/firmware
pio run -e teensy41 -t upload
```

> **Flash quirk:** the first upload attempt frequently fails with
> `error writing to Teensy` — just run the command again; the retry succeeds.
> ALWAYS confirm the output ends `[SUCCESS]`: a failed upload can leave the
> Teensy stranded in its bootloader (no /dev/ttyACM0, no micro-ROS, robot
> dead until a successful flash).

**Pass criteria:**

1. *Any Teensy boot with the motor battery on:* at most a barely perceptible
   per-wheel twitch (the enable transient, braked in the same millisecond) —
   the robot must NOT roll or visibly move during boot. Then it holds braked.
   Feedback flows:

   ```bash
   ros2 launch linorobot2_bringup robot.launch.py   # terminal 1
   ros2 topic echo /odom/unfiltered --once           # terminal 2: prints a message
   ```

2. *Motor battery power-cycled while the Teensy stays up:* **also reboot the
   Teensy afterwards** (reset button or re-flash). The motors resume streaming
   on their own, which the firmware cannot distinguish from "still in MIT
   mode" — without the reboot they may ignore all drive commands.

3. *Reboot mid-drive:* drive with teleop, then reset the Teensy. The robot
   must brake immediately (within tens of ms), not coast.

4. *Drive check:* teleop `i` moves the robot. If it doesn't, see section 13
   (CAN link health) before suspecting software.

## 2. Host simulation (no hardware needed)

**What changed:** `firmware/host_sim/` compiles the real driver headers on any
PC and replays boot, stall, and wheel-trim scenarios (A–F).

```bash
cd firmware/host_sim
g++ -std=c++14 -Wall -Wextra -Istub -I../include sim_boot.cpp -o sim_boot && ./sim_boot
```

**Pass:** output ends with `ALL SCENARIOS PASS`. Run this after ANY change to
`ak10_mit.h`, `config.h`, or the boot/stall logic in `main.cpp`.

## 3. EKF odometry fusion

**What changed:** robot_localization now fuses the Teensy's 50 Hz wheel
odometry with the IMU and owns `odom->base_footprint`; rf2o is a diagnostic
cross-check on `/odom_rf2o` only. Requires the firmware from section 1 (the
`ERPM_TO_WHEEL_RADPS = 0.01237` calibration).

```bash
ros2 launch linorobot2_bringup robot.launch.py        # terminal 1
```

**Pass criteria** (terminal 2):

```bash
# Exactly one publisher of odom->base_footprint, and it's the EKF:
ros2 run tf2_tools view_frames    # inspect frames.pdf: odom->base_footprint broadcaster = ekf_filter_node

# Fused odometry at ~50 Hz:
ros2 topic hz /odom

# rf2o still alive but NOT publishing TF:
ros2 topic hz /odom_rf2o
```

Then drive a taped straight line (~2 m) and a 360° spin with teleop while
recording (see section 6):

```bash
python3 speed_recorder.py
```

**Pass:** `wheel_vx` tracks `cmd_vx` during the straight; `imu_wz` and
`rf2o_wz` agree with `wheel_wz` during the spin (within ~10%). Large steady
disagreement between wheel and rf2o linear speed means the ERPM calibration is
off — do section 4.

Fallback check: `ros2 launch linorobot2_bringup robot.launch.py use_ekf:=false`
must restore rf2o-owned TF (view_frames shows rf2o as the broadcaster).

## 4. ERPM calibration tape test

**What changed:** `ERPM_TO_WHEEL_RADPS` history: 0.00677 → 0.00842 → 0.01138 →
**0.01237** (2026-07-20, two tape-test passes: 1.11 m reported over ~1.50 m
actual with 0.00842, then 1.38 m over 1.50 m with 0.01138). Confirm on your
floor:

1. Tape a start line, measure exactly 2.00 m, tape a finish line.
2. Reset odometry by restarting the launch, align the robot on the start line.
3. Drive forward at the 0.05 m/s default over the finish line, stop, and read
   the integrated position:

   ```bash
   ros2 topic echo /odom --field pose.pose.position.x
   ```

**Pass:** reported x within ±0.05 m of 2.00. If it reads `2.00 * k`, divide
`ERPM_TO_WHEEL_RADPS` in `firmware/include/config.h` by `k`, re-flash
(section 1), repeat.

## 5. Obstacle avoidance (autonomous navigation) — ✅ PASSED 2026-07-19

**What changed:** the local controller is RotationShim + **RegulatedPurePursuit**
again. MPPI was tried and reverted: at the 0.05 m/s cap its optimizer output
vx=0 permanently while starving the Jetson CPU (see git history and
navigation.yaml comments). Avoidance = 1 Hz global replan around costmap
obstacles + RPP collision braking + collision monitor (0.5 s envelope).
Inflation radius 0.4 m; the stock 0.30 m BackUp recovery is replaced by a
short 0.15 m retreat (rear lidar blind zone — only re-enters just-traversed
space), used to back off and re-approach when stuck at doorways;
pose_keeper auto-restores AMCL's pose on startup.

```bash
ros2 launch linorobot2_bringup robot.launch.py slam:=false    # terminal 1
ros2 launch linorobot2_navigation navigation.launch.py        # terminal 2
```

Set the initial pose via the map page (`http://<robot-ip>:8000` → set robot
pose), then sanity-check the controller actually loaded:

```bash
ros2 param get /controller_server FollowPath.primary_controller
# expect: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

Send a goal a few meters across open floor (click the map page, paste the
generated `ros2 action send_goal /navigate_to_pose ...` command), then place a
box in the robot's path mid-drive.

**Pass** (verified 2026-07-19): the robot brakes if the box is close, the 1 Hz
global replan routes around it, and the robot resumes without help; a too-close
box triggers the collision monitor stop, and removing it lets the robot resume
on its own.

## 6. speed_recorder.py

**What changed:** new tool logging commanded vs measured motion to CSV at
20 Hz.

```bash
# with robot.launch.py running; drive with teleop in another terminal
python3 speed_recorder.py --dir ~/speed_logs
# Enter = start, Enter = stop (writes speed_log_<timestamp>.csv), q+Enter = quit
```

**Pass:** the CSV has columns `t_s, dt_s, rf2o_vx, rf2o_wz, wheel_vx,
wheel_wz, imu_wz, cmd_vx, cmd_wz`; cells go EMPTY (not stale-repeated) if you
kill a source mid-recording (e.g. Ctrl-C the launch briefly).

## 7. Box laser filter (battery mask replaces the 180° crop)

**What changed:** `/scan` now keeps the full 360° minus a box over the battery
(`linorobot2_bringup/config/box_laser_filter.yaml`, box in `base_footprint`
frame: x −1.0..−0.10 m, y ±0.5 m). The old crop is still available with
`laser_filter_config:=<path>/angle_laser_filter.yaml`.

```bash
ros2 launch linorobot2_bringup robot.launch.py rviz:=true   # or view /scan remotely
```

**Pass criteria:**

- In rviz the scan shows points BEHIND the robot (walls to the rear are
  visible) but nothing where the battery sits — no phantom blob trailing the
  robot on the map.
- Quick numeric check that the rear isn't all dropped anymore:

  ```bash
  ros2 topic echo /scan --once --field ranges | tr ',' '\n' | grep -cv 'inf\|nan'
  # count of valid points; should be well above the ~half you got with the old crop
  ```

- Drive a lap and confirm rf2o stays healthy (no "Pose is not updated" spam in
  the launch terminal) and SLAM maps rear walls it previously couldn't see.

**Tune the box:** measure the battery's actual footprint relative to the robot
center and shrink `min_x/max_x/min_y/max_y` — everything inside the box is a
permanent blind spot.

Also raised `max_laser_range` 10 → 20 m in `slam.yaml`: remap a long corridor
and check the far end registers before you've driven half of it.

## 8. Overcurrent / stall cutoff (firmware)

**What changed:** if either motor draws more than `OVERCURRENT_AMPS` (8 A
default) continuously for `OVERCURRENT_MS` (1 s), the firmware latches a brake
and ignores cmd_vel until a zero command releases it.

With the robot's drive wheels off the ground or on a slippery mat:

```bash
ros2 topic echo /motor_current    # terminal 2: watch [left, right] amps
```

1. Drive slowly with teleop and firmly hold one wheel (glove/rag — low speed
   cap makes this safe, but mind pinch points). Current on that side climbs.
2. **Pass:** after ~1 s above threshold, BOTH motors brake and stay braked
   even while you keep commanding motion with teleop.
3. Release the wheel, hit space (teleop hard stop → zero cmd_vel).
   **Pass:** the latch releases; normal driving resumes on the next keypress.
4. Calibrate: log `/motor_current` during a normal drive; if your worst normal
   draw approaches 8 A, raise `OVERCURRENT_AMPS` in `firmware/include/config.h`
   to ~2× normal peak and re-flash.

## 9. Periodic micro-ROS time resync (firmware)

**What changed:** the Teensy re-syncs its epoch with the agent every 60 s
(`TIME_SYNC_PERIOD_MS`) instead of once per connection.

Leave the stack running for 30+ minutes, then:

```bash
ros2 topic delay /odom/unfiltered
```

**Pass:** the reported delay stays small and bounded (tens of ms) instead of
growing steadily with uptime.

## 10. map_viewer packaged with linorobot2_bringup + graceful skip

**What changed:** `map_viewer.py` now lives in the repo at
`linorobot2_bringup/scripts/map_viewer.py` and is installed to the package
share directory; `robot.launch.py`'s `map_viewer_path` defaults to the
installed copy instead of the old machine-local `~/Desktop/map_viewer.py`.
If the script is missing (stale install, bad `map_viewer_path` override),
the launch logs `[map_viewer] ... not found -- skipping` and everything else
still comes up (previously the whole launch died).

```bash
# Packaged default serves the page:
cd ~/linorobot2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch linorobot2_bringup robot.launch.py
# then: curl -s -o /dev/null -w '%{http_code}\n' http://localhost:8000  -> 200

# Graceful skip still works:
ros2 launch linorobot2_bringup robot.launch.py map_viewer_path:=/nonexistent.py
```

**Pass:** with the default, port 8000 answers (HTTP 200); with the bogus
override, the skip message appears and `ros2 topic hz /scan` still works.

## 11. graphify hook guard (Claude tooling)

**What changed:** `.claude/settings.json` hooks exit 0 when graphify isn't
installed; `CLAUDE.md` tells Claude to skip graphify when absent.

**Pass:** start a Claude Code session on any machine WITHOUT graphify (e.g. a
web session) and run any command/file-read — no hook error appears. On the
Jetson, the hooks still invoke `~/.local/bin/graphify` as before.

## 12. Firmware CI

**What changed:** `.github/workflows/firmware.yml` builds the host sim
(scenarios A–E, `-Werror`) and the full Teensy firmware via PlatformIO on
every push/PR touching `firmware/`.

```bash
git commit --allow-empty -m "ci: exercise firmware workflow" && git push
```

**Pass:** both `host-sim` and `pio-build` jobs green in the repo's Actions tab
(first `pio-build` run is slow while the micro-ROS lib compiles; cached after).

## 13. CAN link health (debugging "robot won't move")

Hard-won on 2026-07-09: the robot went completely deaf to commands while all
software looked healthy. Root cause was an **intermittent CAN transmit
fault** — receive kept working perfectly (odometry streamed) while transmit
frames died on the wire, so nothing commanded the motors and boot handshakes
were lost. `/motor_current` now carries link diagnostics:

```bash
ros2 topic echo /motor_current --once --field data
# [0] left amps   [1] right amps   [2] CAN frames received since boot
# [3] last raw CAN id              [4] TX error counter  [5] RX error counter
```

**Healthy:** `[4]` and `[5]` sit at 0 and `[2]` climbs steadily.
**TX wiring fault:** `[4]` bouncing high (tens to ~250) with `[5]` = 0 —
commands are dying between the Teensy and the bus. Check, with the motor
battery OFF: the Teensy pin 1 (CTX2) → transceiver TXD wire, the
transceiver's Rs/S mode pin (must be tied low — floating = silent mode =
receive-only!), its VCC/GND, and CANH/CANL + termination. CAN auto-retry can
mask a sick link for a while (driving "works"), so a nonzero `[4]` deserves
attention even when the robot moves.

## 14. Exactly ONE micro-ROS agent

`robot.launch.py` starts the agent. **Never start a second one** (manually or
via a second launch): two agents on `/dev/ttyACM0` corrupt each other's
sessions — symptoms are the Teensy connecting/disconnecting every ~2 s,
topics flickering in and out, and cmd_vel doing nothing or acting delayed.
If the robot behaves strangely, check first:

```bash
ps aux | grep micro_ros_agent | grep -v grep    # must show exactly one
```

Also note the agent does NOT respawn if it dies (e.g. the Teensy's USB device
vanishing during a flash kills it) — if `/odom/unfiltered` is silent but
`/dev/ttyACM0` exists, restart the launch.

## 15. Straight-line drive (per-wheel speed trim)

**What changed (2026-07-20):** the robot always veered when driving straight.
`/odom/unfiltered angular.z` captured during a straight teleop run showed a
sustained yaw bias (+0.005..+0.027 rad/s in 0.0018 rad/s = 1-ERPM steps), i.e.
the wheels genuinely spin at different speeds: each AK10 runs effectively
open-loop from the robot's side (internal KD damping + a torque feedforward
shared by both wheels), so a left/right friction mismatch becomes a
wheel-speed mismatch of up to ~20% at the 0.05 m/s crawl. The firmware now
closes the loop: `moveBase()` integrates each wheel's commanded-vs-measured
speed error (`WHEEL_TRIM_KI`) and offsets that wheel's command until measured
speed converges, clamped at `WHEEL_TRIM_MAX` and reset whenever the command is
(near) zero so a stale trim can never lurch the robot from rest. Constants in
`firmware/include/config.h`; host-sim scenario F covers ramp, clamp, no-touch
on the on-speed wheel, and reset-at-zero.

**Flash** (section 1), then verify:

1. Wheels-up sanity: command a straight drive and confirm both wheels converge
   to the same speed within ~2 s (watch `/odom/unfiltered angular.z` → should
   decay toward 0).
2. Floor test: drive straight for 3+ m with teleop.

   ```bash
   ros2 topic echo /odom/unfiltered --field twist.twist.angular.z
   ```

**Pass:** after the first ~2 s of a straight run, angular.z hovers around zero
(±0.005 rad/s average, single 1-count samples of ±0.0018+ are fine) in BOTH
drive directions, and the robot visibly tracks straight — lateral drift under
~10 cm over 3 m. If it still veers with angular.z ≈ 0, the residual is
mechanical (wheel diameter mismatch / slip) — measure the wheels, don't retune
the trim.

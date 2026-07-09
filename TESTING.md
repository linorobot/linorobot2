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

## 1. Boot-twitch fix (firmware)

**What changed:** the Teensy no longer blindly sends the MIT "enter motor
mode" handshake on boot (which twitched both shafts). It probes for feedback
first and only handshakes motors that are silent; a recovery check re-arms a
motor that powers up late.

**Flash the firmware** (from the Jetson, Teensy on USB):

```bash
cd ~/Desktop/linorobot2/firmware
pio run -e teensy41 -t upload
```

**Pass criteria:**

1. *Teensy-only reboot, motors powered:* with the motor battery on, press the
   Teensy's reset button (or re-flash). The wheels must NOT twitch. Verify
   feedback still flows afterwards:

   ```bash
   ros2 launch linorobot2_bringup robot.launch.py   # terminal 1
   ros2 topic echo /odom/unfiltered --once           # terminal 2: prints a message
   ```

2. *Motor powered after Teensy:* boot the Teensy with the motor battery OFF,
   then switch the battery on. Within ~3 s the recovery handshake arms the
   motors (one small unavoidable enable transient per motor — this is the
   motor's own enable behavior, only on a real power-on). `ros2 topic echo
   /motor_current` starts showing nonzero-capable readings once armed.

3. *Reboot mid-drive:* drive with teleop, then reset the Teensy. The robot
   must brake immediately (within tens of ms), not coast.

## 2. Host simulation (no hardware needed)

**What changed:** `firmware/host_sim/` compiles the real driver headers on any
PC and replays boot + stall scenarios (A–E).

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
`ERPM_TO_WHEEL_RADPS = 0.00842` calibration).

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

**What changed:** `ERPM_TO_WHEEL_RADPS` was corrected from 0.00677 to 0.00842
(~20% error). Confirm on your floor:

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

## 5. MPPI obstacle steering (autonomous navigation)

**What changed:** the local controller is now RotationShim + **MPPI** (was
RegulatedPurePursuit), tuned for the 0.05 m/s cap; global costmap updates at
2 Hz; MPPI accel limits now match the velocity_smoother envelope.

```bash
ros2 launch linorobot2_bringup robot.launch.py slam:=false    # terminal 1
ros2 launch linorobot2_navigation navigation.launch.py        # terminal 2
```

Set the initial pose via the map page (`http://<robot-ip>:8000` → set robot
pose), then sanity-check the controller actually loaded:

```bash
ros2 param get /controller_server FollowPath.primary_controller
# expect: nav2_mppi_controller::MPPIController
```

Send a goal a few meters across open floor (click the map page, paste the
generated `ros2 action send_goal /navigate_to_pose ...` command), then place a
box in the robot's path mid-drive.

**Pass:** the trajectory bends AROUND the box within ~0.5 s (it does not just
stop); a too-close box triggers the collision monitor stop, and removing it
lets the robot resume on its own. Motion at the speed cap is smooth — no
lurch-pause-lurch cycling (that was the old accel-limit mismatch).

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

## 10. map_viewer graceful skip

**What changed:** if `/home/jetson1/Desktop/map_viewer.py` is missing, the
launch logs `[map_viewer] ... not found -- skipping` and everything else still
comes up (previously the whole launch died).

```bash
ros2 launch linorobot2_bringup robot.launch.py map_viewer_path:=/nonexistent.py
```

**Pass:** the skip message appears, and `ros2 topic hz /scan` still works.

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

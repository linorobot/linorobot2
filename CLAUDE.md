# linorobot2 — Wesley's robot (project instructions)

This is a heavily customized fork of linorobot2 (ROS 2 **Jazzy**) running a
real 2WD differential-drive robot. Sessions on this machine run **on the
robot's Jetson itself** — commands here can physically move the robot.

Primary work: hardware bring-up (firmware, CAN, motors), odometry/SLAM
quality, and Nav2 autonomous-driving tuning. The upstream simulation/docs
content (`docs/`, `linorobot2_gazebo/`) is mostly untouched reference.

## Reference documents

- **HARDWARE.md** — the physical robot: boards, wiring, ports, CAN IDs,
  blind zones. Read before touching firmware or anything wiring-adjacent.
- **TROUBLESHOOTING.md** — symptom-first playbook ("robot won't move",
  flaky micro-ROS, Nav2 silent cmd_vel). Check it BEFORE re-deriving a
  diagnosis; most failure modes here have been hit and documented already.
- **TESTING.md** — per-change verification procedures with pass criteria.
  Every behavior change must add or update a section here.
- **firmware/README.md** — firmware topics, config.h tuning knobs.

## Environment & build

- Source lives here (`~/Desktop/linorobot2`), symlinked into the colcon
  workspace at `~/linorobot2_ws/src/linorobot2`. Every terminal:
  ```bash
  source /opt/ros/jazzy/setup.bash && source ~/linorobot2_ws/install/setup.bash
  ```
- After changing launch files, configs, or scripts:
  ```bash
  cd ~/linorobot2_ws && colcon build --symlink-install && source install/setup.bash
  ```
- Firmware (PlatformIO, from `firmware/`): `pio run -e teensy41 -t upload`.
  First upload often fails with `error writing to Teensy` — retry once and
  ALWAYS confirm `[SUCCESS]` (a failed flash strands the Teensy in its
  bootloader and kills the robot until a good flash).
- Firmware logic changes (`ak10_mit.h`, `config.h`, boot/stall paths in
  `main.cpp`) must pass the host sim before flashing:
  ```bash
  cd firmware/host_sim && g++ -std=c++14 -Wall -Wextra -Istub -I../include sim_boot.cpp -o sim_boot && ./sim_boot
  ```
- CI (`.github/workflows/firmware.yml`) builds host sim + Teensy firmware on
  any push touching `firmware/`.

## Safety rules (physical robot)

- **Never publish to `/cmd_vel`\* topics or send nav goals unprompted.**
  Ask, or make sure the user said the robot is safe to move (wheels up,
  clear floor). For injection tests publish ≥10 s (DDS discovery on the
  loaded Jetson takes 2–3 s; short bursts never match).
- **Exactly ONE micro-ROS agent** on `/dev/ttyACM0`. `robot.launch.py`
  starts it — never start a second. Check with `fuser -v /dev/ttyACM0`.
- Motor battery power-cycled while the Teensy stays up → **reboot the
  Teensy too**, or motors silently ignore drive commands.
- If the robot is moved by hand while nav is running, AMCL's pose is
  silently invalid — re-set the pose (map page `http://<robot-ip>:8000`).

## Key files (the ones actually being iterated on)

- `linorobot2_navigation/config/navigation.yaml` — Nav2 tuning. Controller
  history matters: RegulatedPurePursuit is current; MPPI was tried and
  reverted (see file comments + git log) — don't reintroduce it casually.
- `linorobot2_navigation/config/navigate_to_pose_doorway_recovery.xml` —
  custom BT (short 0.15 m BackUp for the rear blind zone). XML gotcha: no
  `--` inside XML comments.
- `linorobot2_navigation/scripts/pose_keeper.py` — restores AMCL pose on
  startup.
- `linorobot2_bringup/config/box_laser_filter.yaml` — battery-box scan mask.
- `linorobot2_bringup/launch/robot.launch.py` — main bringup (agent, lidar,
  EKF, map viewer on :8000).
- `firmware/include/config.h` — wheel geometry, motor dirs, ERPM odometry
  scale, overcurrent limits. `firmware/src/main.cpp` — boot handshake,
  stall latch, CAN diagnostics.
- Root helper scripts: `teleop_keyboard.py` (space = hard stop),
  `speed_recorder.py` (commanded-vs-measured CSV), `lidar_alignment_check.py`.

## Conventions

- Work lands on `claude/*` branches; PRs target `jazzy`. The Jetson has
  limited CPU — treat planner/controller rates and extra nodes (e.g. rf2o)
  as costs, not free.
- Every behavior change gets a TESTING.md section (what changed, pass
  criteria, exact commands). Hard-won debugging lessons go into
  TROUBLESHOOTING.md or HARDWARE.md, not just the commit message.
- Firmware constants that were calibrated on hardware (e.g.
  `ERPM_TO_WHEEL_RADPS = 0.01237`) are measurements, not guesses — don't
  "correct" them from datasheets.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

The `graphify` CLI is only installed on the robot's Jetson (`/home/jetson1/.local/bin/graphify`). **First check it is available** (`command -v graphify`); if it is not (e.g. remote/web sessions), skip these rules and browse the source directly — do not report the missing command as an error.

Rules (only when `graphify` is installed):
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

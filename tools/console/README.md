# Linorobot2 Console

> For a complete step-by-step walkthrough, see [QUICKSTART.md](QUICKSTART.md).

A local browser UI for the ROS2 / robot-computer side of linorobot2: install
the package and sensor drivers, run bringup/teleop/SLAM/navigation, calibrate
the magnetometer, and watch a live LiDAR scan -- without hand-typing the
underlying `ros2 launch`/`colcon`/`rosdep` commands every time.

It complements `robot_config_engine` in `linorobot2_hardware` (which handles
the MCU/firmware side: generating a config header, building, flashing). The
two tools are fully independent at runtime -- Console never calls
config-engine's API or vice versa -- but Console can *import* a
config-engine-generated `config/custom/<name>_config.h` once, as a plain file
read, to pre-fill its own forms (base type, agent transport/baud, whether an
IMU/magnetometer is present).

## Running

```
python3 tools/console/web/server.py [port]   # default port 8090
```

Then open `http://<robot-computer>:<port>/` in a browser. No pip
dependencies -- just the Python 3 standard library, same as config-engine.

## What it does

- **Install** -- clones `linorobot2` itself (no `install.bash`, see below),
  skips the Gazebo package, then `rosdep install` + `colcon build`. Laser and
  depth camera drivers are installed the same way: Console has its own native
  command list per sensor (clone URL, build steps, udev rule), so nothing
  shells out to or sources `install.bash` anywhere, including for sensors.
  `install.bash` was used only as a reference for which commands/URLs are
  correct per sensor.
- **Bringup & Auto-Bringup** -- Tracked in its own dedicated `bringup` runner slot
  (independent of the `main` and `agent` slots). Console features **Automatic Bringup**:
  whenever Teleop, SLAM, Navigation, or Magnetometer calibration is triggered, Console
  automatically checks if Bringup is already active. If not, it launches Bringup in the
  background (via modular `launch_bringup.py`), waits for nodes and micro-ROS agent to
  initialize, and seamlessly launches the requested action in the `main` slot.
- **Multi-Distro ROS 2 Support** -- Full operational support for **Jazzy**, **Lyrical**,
  and **Rolling**. A live distribution selector in the header dynamically
  adapts environment variables (`$ROS_DISTRO`), APT package names, Git branch fallbacks
  (`$ROS_DISTRO` -> `main` -> `jazzy`), and micro-ROS agent container tags.
- **Per-Distro Nav2 Parameter Studio & Python Launcher** -- Console provides an in-browser
  YAML parameter editor with dedicated configuration templates for each distribution:
  `nav2_jazzy.yaml`, `nav2_lyrical.yaml`, `nav2_rolling.yaml` (using modern `behavior_server`
  and `smoother_server`). It uses our own
  modular `launch_nav2.py` Python launcher to provide fully configurable `params_file`, `map`,
  and `sim` arguments without being locked into hardcoded upstream launch parameters.
- **Teleop / SLAM / Navigation / magnetometer calibration** -- One-click actions with
  automatic Bringup dependency resolution. Stop actions independently without terminating
  the underlying Bringup driver.
- **Laser driver** (Sensors tab) -- launches the laser model standalone,
  independent of Bringup and the agent, since the LiDAR talks to the robot
  computer directly. For the `ldlidar_stl_ros2`-family models (LD06/LD19/
  STL27L) this runs the node directly with `--ros-args -p ...` so serial
  port, baudrate, and transport are all genuinely configurable -- including a
  `socat`-based UDP-to-pty bridge for linorobot2_hardware firmware's own
  "LiDAR over WiFi UDP" feature (a raw byte relay with no other ROS2-side
  consumer). Other models go through `linorobot2_bringup`'s own
  `lasers.launch.py`, which already hardcodes persistent `/dev/ydlidar`/
  `/dev/rplidar` udev symlinks for YDLIDAR/RPLIDAR.
- **LiDAR viewer** -- a live polar plot of `/scan` rendered on an HTML
  canvas, fed by a small SSE endpoint that parses `ros2 topic echo /scan`.
  Deliberately its own thing rather than the LiDAR driver's bundled
  `view_*`/`*_view` launch file, which just opens a local RViz window on the
  robot computer's own display -- useless if you're viewing from a laptop.
- **Visualize (RViz via browser)** -- a native (no Docker) equivalent of the
  `kasmvnc` service in linorobot2's own `docker/docker-compose.yaml`: Xvfb +
  x11vnc + noVNC (`websockify`) serve RViz as a plain browser page, viewable
  from the robot computer or any other machine on the network. Meaningfully
  heavier than the rest of Console (RViz is a full Qt/OpenGL app) -- on a
  small SBC (4GB-RAM class), prefer the plain `/scan` canvas viewer unless you
  actually need the full map/costmap/TF view.

## Micro-ROS agent handling

Console never uses `install.bash`'s `install_microros`/`setup_microros_agent`
(the `micro_ros_setup` package route). Instead it reuses config-engine's own
already-working approach:

1. Check whether `micro_ros_agent` is already on the path (system package, or
   a workspace built earlier by config-engine or Console itself at
   `~/uros_ws` -- the two tools share that location so the build only ever
   happens once).
2. Try `apt install ros-$ROS_DISTRO-micro-ros-agent`.
3. Only if both fail, clone `micro-ROS/micro-ROS-Agent` +
   `micro-ROS/micro_ros_msgs` into `~/uros_ws/src` (branch = `$ROS_DISTRO`,
   falling back to `rolling`) and `colcon build`.

Launching the agent afterward is tracked as its own long-lived process, kept
separate from the main action runner, so the agent can keep running while
Bringup/Teleop/SLAM use the main slot for their own commands.

## Workspace directory

Configurable in the Settings tab, defaulting to `~/linorobot2_ws` (matching
`install.bash`'s own default). Every generated command is built against this
path -- changing it just changes where things get cloned/built/launched.

## Not yet implemented: Docker/Podman as an install backend

linorobot2 ships its own `docker/docker-compose.yaml` (build/bringup/slam/
navigate/save-map/rviz/rviz-nav/kasmvnc services). A future addition could
let Install offer Docker/Podman as an alternative to Console's native
clone-and-build path -- e.g. running `docker compose up bringup` (or the
`podman-compose` equivalent) instead of a native `ros2 launch`. Deliberately
out of scope for this pass; noted here so it isn't lost.

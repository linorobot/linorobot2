# Docker

Docker lets you run linorobot2 without a native ROS2 installation. It is particularly useful for simulation on machines not running a supported
Ubuntu version, for cloud-based simulation, and for remote visualization over a browser without needing a display attached.

All commands below are run from the **repository root** unless a `cd` step explicitly changes directory.

---

## Build

### 1. Configure docker/.env

Edit `docker/.env` and set `BASE_IMAGE` to match your target use case:

```
BASE_IMAGE=hardware      # Physical Robot
BASE_IMAGE=gazebo        # Simulated Robot
BASE_IMAGE=gazebo-cuda   # Gazebo simulation with CUDA support (recommended if you have an NVIDIA GPU)
```

If you are using `gazebo-cuda`, set the GPU to use for rendering:

```
GPU_ID=0  # GPU index to use for rendering
```

The GPU index may not always start at `0`. To find the correct value, run:

```bash
ls /dev/dri/card*
```

The number at the end of each path (e.g. `card0`, `card1`) corresponds to a GPU index. Set `GPU_ID` to the index of the GPU you want to use.

If you are working with Physical Robot, also set the following:

```
ROBOT_BASE=<robot_type>      # 2wd, 4wd, or mecanum
LASER_SENSOR=<laser_sensor>  # e.g. ld06, a1, ydlidar  (leave blank if not used)
DEPTH_SENSOR=<depth_sensor>  # e.g. realsense, oakd     (leave blank if not used)
```

### 2. Build the image

Run the build script, which automatically passes your host user's UID and GID so that files created inside the container are owned by your user:

```bash
cd docker
./build
```

Alternatively, run the build manually:

```bash
cd docker
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build
```

> Passing `HOST_UID` and `HOST_GID` is important when using the `dev` service, which mounts the host repository directory into the container. Without these, files created inside the container (e.g. build artifacts) will be owned by root on the host.

After building the image and you're working on a Physical Robot, see the [Hardware](#hardware) section for post-build setup (udev rules, device mapping, and running the robot).

---

## Web Visualization

All tmuxinator configs in `docker/profiles` (`dev`, `hardware`, `sim`) route GUI output (including Gazebo, RViz, and any other ROS GUI tools) to a virtual display at `:200`. A [KasmVNC](https://github.com/kasmtech/KasmVNC) server captures that display and streams it to a browser at:

```
http://<host_ip>:3000
```

On machines with an NVIDIA GPU, the `gazebo-cuda` image uses **VirtualGL** to intercept OpenGL calls from Gazebo and redirect them to the GPU for hardware-accelerated rendering. Without this, a headless server falls back to software rendering, bypassing the GPU entirely.

### Why run headless with a browser interface?

This setup is particularly well-suited for **cloud simulation** on instances such as GCP or AWS, where no monitor is physically attached. The entire simulation stack (Gazebo, Nav2, sensor processing) runs on the remote machine. The browser is the only local interface needed, with no local ROS installation required. Compute can be scaled on demand simply by upgrading the instance, without being constrained by local hardware.

It is also more efficient for **remote visualization** than the conventional approach. In a typical ROS setup, tools like RViz running on a remote machine receive raw topic data (point clouds, laser scans, images) over the network, which can be extremely bandwidth-intensive. With VNC, only a compressed video stream of the rendered display is transmitted, significantly reducing network usage, especially with data-heavy sensors like 3D LiDARs or RGBD cameras.

---

## Development

The `dev` service mounts the entire `linorobot2` repository from the host into the container at `/home/ros/linorobot2_ws/src/linorobot2`. You can edit code on the host with your preferred editor and changes are immediately visible inside the container, with no image rebuild needed. Once changes are made, rebuild the workspace from inside the container with:

```bash
colcon build
```

### Option 1: ./dev (single terminal, GUI forwarded to host)

The `./dev` script starts the `dev` container if it is not already running, then opens an interactive bash shell inside it:

```bash
cd docker
./dev
```

In this mode, GUI applications (e.g. RViz, rqt) are forwarded to the host machine's screen via the `$DISPLAY` environment variable.

### Option 2: Tmuxinator (multi-pane, GUI at http://\<host_ip\>:3000)

[Tmuxinator](https://github.com/tmuxinator/tmuxinator) manages named tmux sessions from a YAML config. In this project it acts like a launch file for multiple Docker services: a single command starts all the required containers and arranges their output into named panes inside one terminal. Each pane runs a different service (Gazebo, Nav2, KasmVNC, an interactive shell) so you have everything visible and reachable without juggling multiple SSH sessions or terminal windows.

A Tmuxinator config at `docker/profiles/dev.yml` opens a tmux window with four panes, each exec'd into the `dev` container, alongside a KasmVNC server. In this mode, any GUI application launched from inside the container appears in the browser at `http://<host_ip>:3000` (see [Web Visualization](#web-visualization)).

**1. Install Tmuxinator**

Follow the installation instructions [here](https://github.com/tmuxinator/tmuxinator?tab=readme-ov-file#installation).

**2. Set the config path**

```bash
source docker/setup_tmux.bash
```

Verify the profiles are detected:

```bash
tmuxinator ls
```

Expected output:

```
tmuxinator projects:
dev       hardware  sim
```

**3. Start the session**

```bash
tmuxinator start dev
```

This starts the `dev` container and KasmVNC, then opens a tmux window with four bash panes ready for development.

Useful tmux key bindings:

- `Ctrl+B` then arrow keys: navigate between panes
- `Ctrl+B` then `D`: detach from the session (containers keep running)

**4. Stop the session**

```bash
tmuxinator stop dev
```

Or to stop and remove all containers:

```bash
cd docker
docker compose down
```

> Inside the container each pane behaves exactly like a native ROS 2 installation. You can run any ROS 2 command directly, for example:
> ```bash
> ros2 topic list
> ros2 topic echo /cmd_vel
> ros2 node list
> ros2 run teleop_twist_keyboard teleop_twist_keyboard
> ```
> No extra setup is needed. `/opt/ros/<ros_distro>/setup.bash` is sourced automatically when the shell starts. To use workspace packages, run `source install/setup.bash` inside `linorobot2_ws`.

---

## Hardware

> Before continuing, complete the [Build](#build) steps and ensure `ROBOT_BASE`, `LASER_SENSOR`, and `DEPTH_SENSOR` are set in `docker/.env`.

### 1. Install udev rules on the host

The Docker image already contains the sensor drivers (installed during build). To create the `/dev/<sensor>` symlinks on the **host** machine, run `install.bash` with `--udev-only`.

`--udev-only` is required: it forces the script to only copy udev rules and skip driver installation (which is not needed since drivers are already inside the container).

```bash
bash install.bash --laser <laser_sensor> --udev-only
# and/or
bash install.bash --depth <depth_sensor> --udev-only
```

### 2. Reload udev rules

After installing, apply the rules without rebooting:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Verify the device symlinks

**Microcontroller:**

Plug in the microcontroller (for example Raspberry Pi Pico), then confirm the device node exists on the host:

```bash
ls /dev/ttyACM0
```

**Sensors:**

Plug in the sensor, then confirm the udev symlink exists on the host:

```bash
ls /dev/<sensor_name>
```

Expected `/dev` paths after udev rules are installed:

| Device | Host `/dev` path |
|--------|-----------------|
| Raspberry Pi Pico microcontroller | `/dev/ttyACM0` |
| `ydlidar` | `/dev/ydlidar` |
| `ld06`, `ld19`, `stl27l` | `/dev/ldlidar` |
| `a1`, `a2`, `a3`, ... | `/dev/rplidar` |

### 4. Update the bringup service devices

Edit `docker/docker-compose.yaml` and update the `bringup` service's `devices` section to map the correct host devices into the container:

```yaml
  bringup:
    ...
    devices:
      - /dev/ttyACM0:/dev/ttyACM0 # Robot's microcontroller (e.g. Pico)
      - /dev/ldlidar:/dev/ldlidar # Laser sensor (adjust to match your sensor symlink)
```

### 5. Run the robot

**5.1. Set the tmuxinator config path:**

```bash
source docker/setup_tmux.bash
```

Verify the profiles are detected:

```bash
tmuxinator ls
```

Expected output:

```
tmuxinator projects:
dev       hardware  sim
```

**5.2. Start the robot:**

```bash
tmuxinator start hardware
```

Once running, visualization is available at: `http://<robot_ip>:3000`

To stop, press `Ctrl+B` then `D` to detach from the tmux session, then run:

```bash
tmuxinator stop hardware
```

---

## Simulated Robot

> Before continuing, complete the [Build](#build) steps with `BASE_IMAGE` set to `gazebo` or `gazebo-cuda`.

### 1. Install Tmuxinator

Follow the installation instructions for Tmuxinator [here](https://github.com/tmuxinator/tmuxinator?tab=readme-ov-file#installation).

### 2. Set the tmuxinator config path

```bash
source docker/setup_tmux.bash
```

Verify the profiles are detected:

```bash
tmuxinator ls
```

Expected output:

```
tmuxinator projects:
dev       hardware  sim
```

### 3. Run the Nav2 demo in Gazebo

```bash
tmuxinator start sim
```

Once running, visualization is available at: `http://<host_ip>:3000`

To stop the simulation, press `Ctrl+B` then `D` to detach from the tmux session, then run:

```bash
tmuxinator stop sim
```

---

## Custom Profiles

Tmuxinator profiles live in `docker/profiles/`. Each profile defines which Docker services to start and how to arrange tmux panes. You can create your own profile to run a different combination of services or add extra panes.

### Available docker-compose services

| Service | Description |
|---------|-------------|
| `kasmvnc` | KasmVNC server: streams the virtual display (`:200`) to a browser at `http://<host_ip>:3000`. Should be included in every profile that uses GUI tools. |
| `dev` | Development container: mounts the host `linorobot2` repo into the container so edits are reflected immediately without rebuilding the image. Runs `sleep infinity` so you can exec into it as needed. |
| `gazebo` | Launches Gazebo with the world specified by `$WORLD` (default: `playground`). Loads `<WORLD>.sdf` from `linorobot2_gazebo/worlds/`. Pass `world_path:=<path>` to use an absolute SDF path instead. Requires `BASE_IMAGE=gazebo` or `gazebo-cuda`. |
| `bringup` | Starts the robot hardware stack (micro-ROS agent, sensors). Requires `ROBOT_BASE`, `LASER_SENSOR`, and/or `DEPTH_SENSOR` set in `.env`, and the corresponding devices mapped. |
| `slam` | Runs SLAM (online mapping). Set `SIM=true` when used with `gazebo`. |
| `navigate` | Runs Nav2 navigation with a pre-built map. Set `SIM=true` when used with `gazebo`. |
| `save-map` | Saves the current SLAM map to `linorobot2_navigation/maps/map.png`. Run this as a one-shot command while `slam` is active. |
| `rviz-nav` | Opens RViz with the navigation config pre-loaded. |
| `rviz` | Opens a bare RViz instance. |

### Creating a new profile

Copy the closest existing profile and edit it:

```bash
cp docker/profiles/sim.yml docker/profiles/my-profile.yml
```

A minimal profile looks like this:

```yaml
name: my-profile
root: <%= ENV["TMUXINATOR_CONFIG"] %>

pre_window:
  - export DISPLAY=:200
  - export SIM=true        # set any env vars your services need

on_project_stop:
  - docker compose down

windows:
  - main:
      panes:
        - docker compose up kasmvnc   # always include this for GUI
        - docker compose up gazebo
        - docker compose up slam
        - docker compose up rviz-nav
```

Each entry under `panes` becomes one tmux pane running that command. Use `docker compose up <service>` to start a service in the foreground, or `docker compose exec <service> bash` to open an interactive shell into an already-running container.

Start your profile with:

```bash
tmuxinator start my-profile
```

## Troubleshooting

### RViz map and costmaps flicker when the GUI is forwarded to the host

**Symptom:** RViz runs, but the map and costmap displays flicker rapidly (about 10 Hz), alternating between drawn and blank. Laser scans and other displays may look normal.

**When it happens:** the Lyrical image (Ubuntu 26.04, RViz built on Qt 6) forwards its window to a host desktop running an **X11 session** (e.g. GNOME on Xorg on Ubuntu 22.04/24.04). This is the setup used by `./dev` and by running a GUI service with `DISPLAY` set to the host display. It has also been seen with Lyrical in a distrobox on the same kind of host. The same container does not flicker when shown under a Wayland session or through KasmVNC.

This is a display issue only: the map, costmaps and navigation are unaffected.

**Workarounds** (any one of these):

- **Use a Wayland session on the host.** At the login screen, choose "Ubuntu on Wayland" (gear icon), then run RViz as usual.
- **Use the browser view.** Route the GUI to KasmVNC (see [Web Visualization](#web-visualization)):
    ```bash
    docker compose up -d kasmvnc
    DISPLAY=:200 docker compose up rviz-nav
    ```
    Then open `http://localhost:3000`.
- **Run RViz from a native Jazzy install on the host** with `use_sim_time:=true`. Jazzy RViz can display topics from the Lyrical container.

!!! note
    RViz in the Lyrical image may log `GLSL link result : active samplers with a different type refer to the same texture image unit` for `indexed_8bit_image`. This error also appears where maps render correctly, and it is not the cause of the flicker.

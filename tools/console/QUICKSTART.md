# 🚀 Linorobot2 Console — Quick Start & Walkthrough Guide

Linorobot2 Console is a lightweight, zero-dependency browser studio for the robot computer side of Linorobot2. It handles installation, sensor drivers, automated bringup, teleop, SLAM, autonomous navigation with editable per-distro parameters, and browser visualization.

---

## 🌟 Key Features

1. **Auto-Bringup Architecture**:
   - Actions requiring the robot hardware/base stack (Teleop, SLAM, Navigation, Magnetometer Calibration) automatically check if Bringup is running.
   - If Bringup is down, Console automatically starts Bringup in a dedicated background slot (`bringup`), waits for ROS 2 nodes and micro-ROS agent to initialize, and seamlessly launches your requested action in the `main` slot.
   - Bringup and higher-level tasks run concurrently without process conflicts.
2. **Multi-Distro ROS 2 Support**:
   - Full support for **Jazzy Jalisco** (Ubuntu 24.04 LTS), **Lyrical Resolute** (Ubuntu 26.04), and **Rolling Ridley**. (Humble was dropped upstream.)
   - One-click distribution selector in the header dynamically adapts environment variables, APT package names, Git branch fallbacks (`$ROS_DISTRO` &rarr; `main` &rarr; `jazzy`), and container image tags.
3. **Per-Distro Nav2 Parameter Studio**:
   - In-browser YAML parameter editor with dedicated configuration templates for each distro:
     - `nav2_jazzy.yaml` / `nav2_lyrical.yaml` / `nav2_rolling.yaml`: Modern `behavior_server`, `smoother_server`, and dynamic BT navigators.
   - Save custom robot footprints, planner tolerances, and costmap inflation radiuses directly in the web UI.
4. **Dedicated Python Launchers**:
   - `launch_nav2.py`: Overcomes upstream hardcoded parameter limitations by declaring configurable `params_file`, `map`, `distro`, and `sim` arguments.
   - `launch_bringup.py`: Modular robot bringup launcher with custom transport and baudrate mapping.
5. **Headless Browser Visualization**:
   - Native browser RViz via Xvfb + x11vnc + noVNC (`:6080`).
   - One-click KasmVNC Virtual Desktop integration (`:3000`) for Gazebo 3D simulations.
   - Zero-overhead HTML5 Canvas polar LiDAR viewer (`/scan` SSE stream).

---

## 🛠️ Quick Start Walkthrough

### 1. Launching Console
Clone this fork (branch `console`) -- no build, no `rosdep`, no `install.bash`
or `setup.bash` to run first; the Console is plain Python 3 stdlib and does the
ROS 2 install for you from the Install tab.
```bash
git clone -b console https://github.com/hippo5329/linorobot2.git
cd linorobot2/tools/console/web
python3 server.py 8090
```
Open **`http://localhost:8090/`** (or `http://<robot-ip>:8090/` from any computer on your local network or Tailscale).

---

### 2. Header Status & Distribution Selection
At the top of every tab, the Console header displays real-time status:
- **ROS Distro**: Selectable dropdown (`jazzy`, `lyrical`, `rolling`). Changing the dropdown updates the active environment instantly.
- **Workspace**: Shows whether `install/setup.bash` is built.
- **Bringup Pill**: Dynamically shows `down`, `starting...`, or `running`.
- **Agent Pill**: Shows micro-ROS agent connection status.

---

### 3. Step-by-Step Workflow

#### Step A: Hardware Config Import & Installation (Install Tab)
1. Point to your firmware configuration header (e.g. `config/custom/my_robot_config.h` created by `robot_config_engine`) and click **Import**. Base drive type, baudrates, and IMU configurations are automatically populated.
2. Select your Install Mode:
   - **Native**: Compiles packages into `~/linorobot2_ws`.
   - **Docker / Podman**: Runs containers with zero host package modifications.
3. Click **Run base install** to clone and build `linorobot2` using the active distro branch fallback.

#### Step B: Teleop with Auto-Bringup (Teleop Tab)
1. Navigate to the **Teleop** tab.
2. Click **Start teleop**.
3. **Notice Auto-Bringup**:
   - Console detects Bringup is not running.
   - It starts `launch_bringup.py` in the background `bringup` slot.
   - The header **Bringup** pill switches to `starting...` and then `running` (green).
   - Once initialized, Teleop starts in the `main` slot.
4. Drive the robot using your USB or Bluetooth gamepad.

#### Step C: SLAM Mapping & Map Saving (SLAM & Nav Tab)
1. Switch to the **SLAM & Nav** tab.
2. Click **Start SLAM**. Because Bringup is already active in the background, SLAM starts immediately.
3. Drive your robot around the room to construct the occupancy grid map.
4. Under **Save map**, enter a map name (e.g. `lab_world`) and click **Save map**.
5. The map is saved into `linorobot2_navigation/maps/` and automatically refreshed in the Navigation map selector.

#### Step D: Tune Nav2 Parameters & Navigate (SLAM & Nav Tab)
1. Under **Navigation**, select your saved map from the dropdown.
2. Click **Edit Nav2 Parameters (YAML)** to expand the in-browser parameter editor.
3. Select your target ROS 2 distribution from the editor dropdown (e.g. `jazzy` or `lyrical`).
4. Modify any desired parameters (e.g., `inflation_radius: 0.55`, `robot_radius: 0.3`, `max_vel_x: 0.6`).
5. Click **Save Nav2 Configuration**. The parameters are saved to `console_nav2_<distro>.yaml`.
6. Click **Start navigation**. Console executes:
   ```bash
   ros2 launch launch_nav2.py map:=/path/to/lab_world.yaml params_file:=.../console_nav2_jazzy.yaml distro:=jazzy sim:=false
   ```

#### Step E: Visualizing in Browser
- **Live 2D LiDAR**: Switch to the **Sensors / LiDAR** tab and click **Start LiDAR stream** for a lightweight HTML5 polar plot.
- **3D RViz (Native)**: In **SLAM & Nav**, click **Start RViz via noVNC** and open the provided link to view costmaps and set 2D navigation goals in your browser.
- **Gazebo Simulation (Docker)**: In the **Install** tab, select Docker service `gazebo` and click **Open VNC Display (:3000) ↗** to interact with full 3D simulation at 100% Real-Time Factor.

---

## 📁 Repository File Structure

```
tools/console/
├── README.md                      # Architecture & reference documentation
├── QUICKSTART.md                  # This quick start & walkthrough guide
├── launch_bringup.py              # Modular ROS 2 Python bringup launcher
├── launch_nav2.py                 # Multi-distro ROS 2 Python Nav2 launcher
├── test_console.py                # Unit test suite (8/8 tests PASS)
├── config/                        # Reference distro Nav2 templates
│   ├── nav2_jazzy.yaml            # Jazzy Nav2 configuration (behavior_server)
│   ├── nav2_lyrical.yaml          # Lyrical Nav2 configuration
│   └── nav2_rolling.yaml          # Rolling Nav2 configuration
└── web/                           # Zero-dependency web UI studio
    ├── server.py                  # Python SSE multi-runner HTTP backend
    ├── index.html                 # Console frontend layout
    ├── app.js                     # Frontend orchestration & event logic
    ├── style.css                  # Dark-mode responsive styling
    ├── console_config.json        # User workspace & distro preferences
    ├── console_nav2_jazzy.yaml    # Active editable Jazzy parameters
    ├── console_nav2_lyrical.yaml  # Active editable Lyrical parameters
    └── console_nav2_rolling.yaml  # Active editable Rolling parameters
```

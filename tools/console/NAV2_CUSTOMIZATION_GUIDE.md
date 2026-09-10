# Nav2 Customization & Tuning Guide
*(Synthesized from the Official Nav2 Documentation: First-Time Robot Setup Guide, Nav2 Tuning Guide & Plugin Customization Guides)*

This guide details the complete customization and parameter tuning architecture for **Navigation 2 (Nav2)**, **SLAM Toolbox**, and **robot_localization (EKF)** on Linorobot2 mobile robots across the supported ROS 2 distributions (**Jazzy**, **Lyrical**, **Rolling**). *(Humble was dropped following the upstream linorobot decision.)*

---

## 1. Nav2 System Architecture

Nav2 relies on a managed lifecycle state machine (Unconfigured -> Inactive -> Active) and modular `pluginlib` components. On Jazzy / Lyrical / Rolling the stack is:

| Subsystem / Feature | Component |
| :--- | :--- |
| **Recovery Engine** | `behavior_server` (Spin, BackUp, DriveOnHeading, AssistedTeleop) |
| **Path Follower** | `RegulatedPurePursuitController` with `RotationShimController` |
| **Path Smoothing** | `smoother_server` (SimpleSmoother) |
| **Charging & Docking** | `docking_server` (SimpleChargingDock) |
| **Velocity Limiter** | `nav2_velocity_smoother::VelocitySmoother` |
| **Behavior Trees** | BehaviorTree.CPP v4 (`NavigateToPose`, `NavigateThroughPoses`) |

Linorobot2 maintains validated per-distro configurations in:
- `linorobot2_navigation/config/navigation_<distro>.yaml` (Differential 2WD/4WD)
- `linorobot2_navigation/config/navigation_<distro>_mecanum.yaml` (Holonomic Mecanum)

---

## 2. Drive Kinematics & Coordinate Frames (TF2)

Per the **Nav2 First-Time Robot Setup Guide**, the coordinate transformation tree must be strictly maintained:

$$\text{map} \xrightarrow{\text{AMCL / SLAM}} \text{odom} \xrightarrow{\text{robot\_localization}} \text{base\_footprint} \xrightarrow{\text{URDF}} \text{base\_link} \xrightarrow{\text{URDF}} \text{laser / camera}$$

### 2.1 Differential Drive (2WD / 4WD Skid Steer)
Non-holonomic platforms cannot translate sideways ($v_y = 0$).

1. **Velocity Smoother (`velocity_smoother`)**:
   ```yaml
   velocity_smoother:
     ros__parameters:
       smoothing_frequency: 20.0
       scale_velocities: False
       feedback: "OPEN_LOOP"
       max_velocity: [0.5, 0.0, 2.5]       # [vx, vy, vtheta] -- vy is 0.0!
       min_velocity: [-0.5, 0.0, -2.5]
       max_accel: [2.5, 0.0, 3.2]          # [ax, ay, atheta] -- ay is 0.0!
       max_decel: [-2.5, 0.0, -3.2]
   ```
2. **AMCL Motion Model**:
   ```yaml
   amcl:
     ros__parameters:
       robot_model_type: "nav2_amcl::DifferentialMotionModel"
   ```
3. **Controller Lateral Velocity Filter**:
   ```yaml
   controller_server:
     ros__parameters:
       min_y_velocity_threshold: 0.5       # Suppresses encoder noise in Y
   ```
4. **EKF Fusion (`linorobot2_base/config/ekf_2wd.yaml`)**:
   `odom0_config` **MUST** set $v_y$ (row 3, column 2) to `false`. If enabled on differential drive, in-place wheel slip gets integrated into false lateral position drift!

---

### 2.2 Holonomic Mecanum Drive
Mecanum robots move omnidirectionally with independent forward ($v_x$), lateral ($v_y$), and rotational ($\omega$) motion.

1. **Velocity Smoother (`velocity_smoother`)**:
   ```yaml
   velocity_smoother:
     ros__parameters:
       max_velocity: [0.5, 0.5, 2.5]       # vy enabled (0.5 m/s)
       min_velocity: [-0.5, -0.5, -2.5]
       max_accel: [2.5, 2.5, 3.2]          # ay enabled (2.5 m/s^2)
       max_decel: [-2.5, -2.5, -3.2]
   ```
2. **AMCL Motion Model**:
   ```yaml
   amcl:
     ros__parameters:
       robot_model_type: "nav2_amcl::OmniMotionModel"
   ```
3. **Controller Lateral Velocity Filter**:
   ```yaml
   controller_server:
     ros__parameters:
       min_y_velocity_threshold: 0.001     # Allows fine lateral strafing maneuvers
   ```
4. **EKF Fusion (`linorobot2_base/config/ekf_mecanum.yaml`)**:
   `odom0_config` sets $v_y$ to `true` to integrate genuine lateral strafe velocity from the 4 mecanum wheels.

---

## 3. Costmap Inflation Layer & The Potential Field Formula

Per the official **Nav2 Tuning Guide**, the inflation layer should not merely create an obstacle barrier, but rather establish a smooth, consistent **potential field** that guides global and local planners:

$$\text{Cost}(d) = \exp\left(-1.0 \times \text{cost\_scaling\_factor} \times (d - r_{inscribed})\right) \times (\text{INSCRIBED\_INFLATED\_OBSTACLE} - 1)$$

```
Cost ^
254  | [Lethal Obstacle: Physical Wall]
253  |----+ [Inscribed Radius: Robot collision boundary]
     |     \
     |      \  <-- Higher cost_scaling_factor (5.0 - 10.0) = steep dropoff (open doors)
     |       \ <-- Lower cost_scaling_factor  (2.0 - 3.5)  = gentle slope (center in aisles)
  0  +--------+-------------------> Distance (d)
              r_inscribed   inflation_radius
```

### 3.1 Resolving the 80cm Doorway Problem
- **Symptom**: The robot refuses to pass through standard 80cm interior doorways, hesitating, oscillating, or aborting with "no valid path".
- **Nav2 Tuning Guide Remedy**:
  When `inflation_radius: 0.70` and `cost_scaling_factor: 3.0` are used, inflation from both door posts overlaps across the center of an 80cm doorway. The planner sees costs $> 180$ across the entire opening and refuses to traverse it.
- **Solution**:
  1. Reduce `inflation_radius` to `0.52` m (just above the inscribed radius).
  2. Increase `cost_scaling_factor` to `5.5` – `6.0`.
  This creates a steep potential drop, opening a low-cost valley ($< 50$) through the exact center of the door.

### 3.2 Costmap Observation Sources — 2D LiDAR + Depth Camera Together

Nav2 costmap layers fuse multiple sensors through the `observation_sources`
list. Running a 2D LiDAR and a depth camera at the same time is supported and
standard: the LiDAR feeds a `LaserScan` source, the depth camera feeds a
`PointCloud2` source (`/camera/depth/color/points`), and both mark/clear the
same costmap.

The **console's** config templates (`tools/console/config/nav2_*.yaml`, and the active `console_nav2_*.yaml`) define both; the upstream `linorobot2_navigation` package configs are left as shipped. Every costmap layer in the console templates carries:

```yaml
observation_sources: scan pointcloud
scan:        { topic: /scan,                        data_type: "LaserScan" }
pointcloud:  { topic: /camera/depth/color/points,   data_type: "PointCloud2" }
```

The `collision_monitor` (a fast safety loop) is deliberately kept LiDAR-only.

**Gating.** An `observation_sources` entry whose topic never publishes just logs
a periodic "observation buffer has not been updated" warning — harmless but
noisy on a robot with no depth camera. The **console's own launcher**
(`tools/console/launch_nav2.py`, self-contained — it includes
`nav2_bringup/bringup_launch.py` directly) handles this via a `depth_costmap`
argument —

| `depth_costmap:=` | effect |
| :--- | :--- |
| `auto` (default) | on iff `LINOROBOT2_DEPTH_SENSOR` is set (same env var `linorobot2_bringup` uses) |
| `true` | force the pointcloud source on |
| `false` | force it off |

When off, it reads the resolved params file, strips `pointcloud` from every
`observation_sources` line (the inert `pointcloud:` block stays), writes a
`console_nav2_gated_*.yaml` temp copy, and passes *that* as `params_file` to nav2 — **your saved YAML is never modified**. Console passes
`depth_costmap:=true|false` from the Bringup depth-sensor selection. `patcher.py`
exposes the same transform for the in-browser editor:

```python
patcher.patch_costmap_sources(text, depth_enabled=False)  # -> observation_sources: scan
patcher.costmap_depth_active(text)                          # -> bool
```

---

## 4. Path Tracking & Oscillation Damping

### 4.1 Regulated Pure Pursuit Controller (Jazzy / Lyrical / Rolling)
The official Nav2 documentation recommends RPP for robust path tracking with velocity regulation:

- **Adaptive Lookahead**:
  Enable `use_velocity_scaled_lookahead_dist: true`. Set `min_lookahead_dist: 0.3` m and `max_lookahead_dist: 0.9` m with `lookahead_time: 1.5` s. The controller looks further ahead at high speeds for stability, and tightens up at low speeds for precision.
- **Curvature Regulation**:
  `use_regulated_linear_velocity_scaling: true` automatically slows the robot on sharp turns to prevent wheel slip and rollover.
- **Goal Hunting / In-Place Oscillation**:
  If the robot rapidly shakes or oscillates at the final goal pose:
  1. Reduce `rotate_to_heading_angular_vel` from 1.8 down to `1.2` rad/s.
  2. Reduce `max_angular_accel` from 3.2 down to `2.2` rad/s².
  3. Increase `general_goal_checker` `yaw_goal_tolerance` to `0.15` rad (~8.5°).

---

## 5. Console Launch Pipeline

The console drives navigation and SLAM through its **own** launcher,
`tools/console/launch_nav2.py` — a single `OpaqueFunction` that
resolves the per-distro console params (`console_nav2_<distro>.yaml` →
`config/nav2_<distro>[_mecanum].yaml`), applies the depth→costmap gate, and
then includes **`nav2_bringup/bringup_launch.py`** directly plus an RViz node.
It does not depend on `linorobot2_navigation`'s own launch files, and it
never modifies upstream code.

```bash
# one `slam:=` arg switches SLAM mapping vs AMCL navigation
ros2 launch tools/console/launch_nav2.py \
  distro:=jazzy base:=mecanum map:=/path/to/my_map.yaml            # AMCL nav

ros2 launch tools/console/launch_nav2.py \
  distro:=jazzy base:=mecanum slam:=true                           # SLAM mapping
```

For SLAM the console concatenates its nav2 params + `slam.yaml` into one temp
file (they're disjoint top-level mappings) since `bringup_launch.py` feeds a
single `params_file` to both slam_toolbox and the nav2 stack.

---

### 5.5 Diagnostic Troubleshooting Matrix: Drift, Overshoot, Destination Reachability & In-Place Rotation

| Symptom / Failure Mode | Physical & Algorithmic Root Cause | Prescribed Parameter & System Fix |
| :--- | :--- | :--- |
| **State Estimation Drift** *(Sideways wandering, in-place spin displacement)* | For 2WD/4WD robots, wheel slip during spins publishes noisy $v_y$ in `/odom/unfiltered`. If fused, EKF integrates this as permanent lateral displacement. Low update rate (<20 Hz) causes numerical integration errors. | 1. Set EKF `odom0_config` $v_y = \text{false}$ for differential drive.<br>2. Standardize EKF `frequency: 50.0` (matching micro-ROS).<br>3. Enforce `two_d_mode: true`.<br>4. In `controller_server`, set `min_y_velocity_threshold: 0.5`. |
| **Goal Overshoot** *(Blowing past destination, late braking, corner overshoot)* | Loose deceleration limits in `velocity_smoother` (e.g. $-1.0\,\text{m/s}^2$) leave robot with excessive kinetic energy. Approach velocity scaling disabled or lookahead distance too long near goal. | 1. Stiffen braking authority in `velocity_smoother`: `max_decel: [-2.8, 0.0, -3.5]`.<br>2. Enable approach scaling: `approach_velocity_scaling_dist: 0.75m`, `min_approach_linear_velocity: 0.05m/s`.<br>3. Shorten lookahead near goal: `lookahead_dist: 0.45m`. |
| **Unable to Reach Destination** *(Stops short, tolerance timeout, goal abort)* | Overly strict goal tolerances (<5cm) when mechanical encoder backlash or deadband is 3-4cm; progress checker triggers timeout; or goal pose is placed inside obstacle inflation halo ($cost > 200$). | 1. Expand goal tolerances: `xy_goal_tolerance: 0.08m` (8cm), `yaw_goal_tolerance: 0.12rad` (~7°).<br>2. Increase progress allowance: `movement_time_allowance: 15.0s`, `required_movement_radius: 0.15m`.<br>3. Steepen inflation falloff: `inflation_radius: 0.52m`, `cost_scaling_factor: 5.5`. |
| **In-Place Rotation Instability** *(Oscillation at goal, wide banana-arc turns, motor shudder)* | Abrupt angular acceleration (>3.0 rad/s²) breaks wheel traction. Missing rotation shim causes controller to trace wide curves instead of turning on the spot. Strict yaw tolerance causes continuous "hunting" oscillation. | 1. Smooth angular acceleration: `max_angular_accel: 2.0 rad/s²`.<br>2. Tune `RotationShimController`: `angular_dist_threshold: 0.785 rad` (45°), `rotate_to_heading_angular_vel: 1.5 rad/s`.<br>3. Enable `rotate_to_heading_once: true`.<br>4. Expand `yaw_goal_tolerance: 0.12 rad`. |

---

## 6. Automated Patcher Tool (`patcher.py`)

Linorobot2 includes a zero-dependency CLI patcher in `tools/console/patcher.py`:

```bash
# Apply a pre-configured expert preset:
python3 tools/console/patcher.py preset \
  --preset mecanum_omni \
  --nav2-file linorobot2_navigation/config/navigation_jazzy.yaml \
  --ekf-file linorobot2_base/config/ekf.yaml \
  --slam-file linorobot2_navigation/config/slam.yaml

# Patch specific velocity, acceleration, and inflation parameters:
python3 tools/console/patcher.py nav2 \
  -i linorobot2_navigation/config/navigation_jazzy.yaml \
  -b mecanum --max-vel-x 0.6 --inflation-radius 0.52 --cost-scaling-factor 5.5
```

---

## 7. AI Tuning Studio & Custom Robot Builder in Console

Linorobot2 Console (`http://localhost:8090/`) provides an interactive web-based studio:
- **AI Robotics Tuning Assistant**: Click prompt chips or describe symptoms (e.g. *"Doorway hesitation"* or *"Mecanum strafe"*) to receive physics diagnoses and 1-click parameter patches.
- **Curated Presets**: Quick application of `standard_diff`, `mecanum_omni`, `cautious_indoor`, `fast_open_space`, and `high_res_slam`.
- **AI Custom Robot Builder Studio**: Complete guided workflow from chassis design, wheel diameter, and motor RPM to firmware `custom_config.h`, URDF footprint, EKF 50Hz filter, and Nav2 profiles.

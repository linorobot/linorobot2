#!/usr/bin/env python3
"""Linorobot2 Configuration Patcher for Nav2, SLAM & EKF.

Surgically patches configuration files with zero external dependencies:
- Nav2: Kinematics (Diff vs Mecanum), Max/Min Speeds, Accelerations, Costmap Inflation, Controller params
- EKF: Odometry lateral velocity fusion (Diff vy=false vs Mecanum vy=true), IMU Yaw/Gyro fusion, Frequency
- SLAM: Map resolution, Laser max range, Keyframe travel distance/heading, Loop closure search range
- Presets: Pre-packaged expert tuning presets for different robot configurations and operational environments
"""

import argparse
import math
import os
import re
import sys


PRESETS = {
    "smooth_rotation": {
        "label": "Smooth In-Place Rotation & Anti-Oscillation",
        "desc": "Tuned RotationShimController (45° threshold, 1.5 rad/s spin) with smooth angular acceleration (2.0 rad/s²) and zero lateral slip",
        "base": "2wd",
        "max_vel_x": 0.5,
        "max_vel_y": 0.0,
        "max_vel_theta": 1.8,
        "max_accel_x": 2.2,
        "max_accel_y": 0.0,
        "max_accel_theta": 2.0,
        "rotate_to_heading_angular_vel": 1.5,
        "angular_dist_threshold": 0.785,
        "xy_goal_tolerance": 0.08,
        "yaw_goal_tolerance": 0.12,
        "inflation_radius": 0.65,
        "cost_scaling_factor": 3.5,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "anti_drift": {
        "label": "Anti-Drift State Estimation",
        "desc": "Disables lateral velocity slip fusion in EKF (vy=false) and locks 50Hz 2D planar fusion for 2WD/4WD",
        "base": "2wd",
        "max_vel_x": 0.5,
        "max_vel_y": 0.0,
        "max_vel_theta": 2.5,
        "max_accel_x": 2.5,
        "max_accel_y": 0.0,
        "max_accel_theta": 3.2,
        "inflation_radius": 0.7,
        "cost_scaling_factor": 3.0,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "anti_overshoot": {
        "label": "Anti-Overshoot / Active Braking",
        "desc": "Stiff deceleration (-2.8 m/s²), approach velocity scaling (0.75m), and regulated lookahead to stop goal blow-by",
        "base": "2wd",
        "max_vel_x": 0.45,
        "max_vel_y": 0.0,
        "max_vel_theta": 2.2,
        "max_accel_x": 2.0,
        "max_accel_y": 0.0,
        "max_accel_theta": 2.5,
        "max_decel_x": 2.8,
        "max_decel_theta": 3.5,
        "xy_goal_tolerance": 0.08,
        "yaw_goal_tolerance": 0.12,
        "approach_velocity_scaling_dist": 0.75,
        "lookahead_dist": 0.45,
        "inflation_radius": 0.65,
        "cost_scaling_factor": 3.5,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "destination_guarantee": {
        "label": "Robust Goal Arrival (Anti-Stuck)",
        "desc": "Realistic goal tolerances (0.08m, 0.12rad), generous progress allowance (15s), and steep inflation falloff to reach destination",
        "base": "2wd",
        "max_vel_x": 0.4,
        "max_vel_y": 0.0,
        "max_vel_theta": 2.0,
        "max_accel_x": 2.0,
        "max_accel_y": 0.0,
        "max_accel_theta": 2.5,
        "xy_goal_tolerance": 0.08,
        "yaw_goal_tolerance": 0.12,
        "movement_time_allowance": 15.0,
        "required_movement_radius": 0.15,
        "inflation_radius": 0.52,
        "cost_scaling_factor": 5.5,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "standard_diff": {
        "label": "Standard Differential (2WD / 4WD)",
        "desc": "Balanced default for indoor differential / skid-steer navigation",
        "base": "2wd",
        "max_vel_x": 0.5,
        "max_vel_y": 0.0,
        "max_vel_theta": 2.5,
        "max_accel_x": 2.5,
        "max_accel_y": 0.0,
        "max_accel_theta": 3.2,
        "inflation_radius": 0.7,
        "cost_scaling_factor": 3.0,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "mecanum_omni": {
        "label": "Mecanum (Omnidirectional Strafe)",
        "desc": "Holonomic omnidirectional drive with lateral velocity and Omni motion model",
        "base": "mecanum",
        "max_vel_x": 0.5,
        "max_vel_y": 0.5,
        "max_vel_theta": 2.5,
        "max_accel_x": 2.5,
        "max_accel_y": 2.5,
        "max_accel_theta": 3.2,
        "inflation_radius": 0.65,
        "cost_scaling_factor": 3.5,
        "ekf_frequency": 50.0,
        "fuse_vy": True,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 10.0,
    },
    "cautious_indoor": {
        "label": "Cautious / Tight Hallways",
        "desc": "Lower speeds and larger obstacle safety margins for narrow passages and crowded areas",
        "base": "2wd",
        "max_vel_x": 0.3,
        "max_vel_y": 0.0,
        "max_vel_theta": 1.8,
        "max_accel_x": 1.5,
        "max_accel_y": 0.0,
        "max_accel_theta": 2.0,
        "inflation_radius": 0.85,
        "cost_scaling_factor": 2.0,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.04,
        "slam_max_range": 8.0,
    },
    "fast_open_space": {
        "label": "Fast / Large Open Space",
        "desc": "Higher cruising speed and acceleration for open warehouses or arenas",
        "base": "2wd",
        "max_vel_x": 0.8,
        "max_vel_y": 0.0,
        "max_vel_theta": 3.0,
        "max_accel_x": 3.0,
        "max_accel_y": 0.0,
        "max_accel_theta": 4.0,
        "inflation_radius": 0.6,
        "cost_scaling_factor": 4.0,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.05,
        "slam_max_range": 12.0,
    },
    "high_res_slam": {
        "label": "High-Definition Mapping",
        "desc": "Sub-centimeter (0.025m) SLAM resolution with frequent keyframe updates for fine floor plans",
        "base": "2wd",
        "max_vel_x": 0.35,
        "max_vel_y": 0.0,
        "max_vel_theta": 2.0,
        "max_accel_x": 2.0,
        "max_accel_y": 0.0,
        "max_accel_theta": 2.5,
        "inflation_radius": 0.7,
        "cost_scaling_factor": 3.0,
        "ekf_frequency": 50.0,
        "fuse_vy": False,
        "fuse_imu_yaw": False,
        "slam_resolution": 0.025,
        "slam_max_range": 12.0,
    },
}


# Keys patch_nav2_text / patch_ekf_text accept as tunables. Used to turn a
# sparse dict (an AI patch, a PRESETS entry, a robot-builder spec, an API
# payload) into keyword args without forwarding unrelated keys or reintroducing
# hardcoded defaults -- a targeted fix then leaves every untouched key alone.
NAV2_TUNABLES = (
    "max_vel_x", "max_vel_y", "max_vel_theta", "max_accel_x", "max_accel_y",
    "max_accel_theta", "desired_linear_vel", "inflation_radius", "cost_scaling_factor",
    "max_decel_x", "max_decel_theta", "xy_goal_tolerance", "yaw_goal_tolerance",
    "lookahead_dist", "approach_velocity_scaling_dist", "movement_time_allowance",
    "required_movement_radius", "rotate_to_heading_angular_vel", "angular_dist_threshold",
    "raytrace_range", "obstacle_max_range",
)
EKF_TUNABLES = ("frequency", "two_d_mode", "fuse_vy", "fuse_imu_yaw", "fuse_imu_vyaw")


def nav2_kwargs(src):
    """Filter a dict down to patch_nav2_text kwargs, dropping absent (None) keys.

    Accepts either ``base_type`` or ``base`` for the kinematics selector.
    """
    kw = {}
    base = src.get("base_type") or src.get("base")
    if base:
        kw["base_type"] = base
    for key in NAV2_TUNABLES:
        if src.get(key) is not None:
            kw[key] = src[key]
    return kw


def ekf_kwargs(src, base=None):
    """Filter a dict down to patch_ekf_text kwargs, dropping absent (None) keys."""
    kw = {}
    resolved_base = base or src.get("base_type") or src.get("base")
    if resolved_base:
        kw["base_type"] = resolved_base
    for key in EKF_TUNABLES:
        if src.get(key) is not None:
            kw[key] = src[key]
    return kw


def _fmt(value):
    """Render a numeric parameter value the way the YAML files already write them."""
    return str(float(value))


def _neg(value):
    """Negated value for min/decel arrays, without an ugly ``-0.0``."""
    v = float(value)
    return 0.0 if v == 0 else -v


def _set_bool_list_index(text, key, index, value):
    """Set the Nth ``true``/``false`` token inside ``key: [ ... ]`` to ``value``.

    Whitespace, newlines, indentation and comma placement between elements are
    preserved (they live in the split separators), so a reflowed EKF mask still
    patches instead of silently no-op'ing. Returns ``(text, matched)``.
    """
    m = re.search(re.escape(key) + r'(\s*:\s*\[)([^\]]*)(\])', text)
    if not m:
        return text, False
    tokens = re.split(r'(\btrue\b|\bfalse\b)', m.group(2))
    bool_positions = [i for i, t in enumerate(tokens) if t in ('true', 'false')]
    if index >= len(bool_positions):
        return text, False
    tokens[bool_positions[index]] = 'true' if value else 'false'
    return (text[:m.start()] + key + m.group(1) + ''.join(tokens) + m.group(3)
            + text[m.end():]), True


def _set_scalar(text, key, value):
    """Replace `key: <value>` on non-comment YAML lines, preserving any trailing `# comment`.

    A leading `#` (commented-out template block) is never matched, so disabled
    `precise_goal_checker` / `FollowPath` alternatives are left untouched.
    """
    if value is None:
        return text
    rendered = _fmt(value)
    pat = re.compile(
        r'^(?P<pre>[ \t]*' + re.escape(key) + r':[ \t]*)'
        r'(?P<val>[^\n#]*?)'
        r'(?P<post>[ \t]*(?:#[^\n]*)?)$',
        re.MULTILINE,
    )
    return pat.sub(lambda m: m.group('pre') + rendered + (m.group('post') or ''), text)


def _read_vec3(text, key, fallback):
    """Read a `key: [x, y, theta]` list from the YAML text; return `fallback` if absent/unparseable."""
    m = re.search(r'^[ \t]*' + re.escape(key) + r':[ \t]*\[([^\]]*)\]', text, re.MULTILINE)
    if not m:
        return list(fallback)
    parts = [p.strip() for p in m.group(1).split(',')]
    try:
        vec = [float(p) for p in parts[:3]]
        return vec if len(vec) == 3 else list(fallback)
    except ValueError:
        return list(fallback)


def patch_nav2_text(text, base_type=None, max_vel_x=None, max_vel_y=None, max_vel_theta=None,
                    max_accel_x=None, max_accel_y=None, max_accel_theta=None, desired_linear_vel=None,
                    inflation_radius=None, cost_scaling_factor=None,
                    max_decel_x=None, max_decel_theta=None,
                    xy_goal_tolerance=None, yaw_goal_tolerance=None,
                    lookahead_dist=None, approach_velocity_scaling_dist=None,
                    movement_time_allowance=None, required_movement_radius=None,
                    rotate_to_heading_angular_vel=None, angular_dist_threshold=None,
                    raytrace_range=None, obstacle_max_range=None):
    """Patch Nav2 YAML text in place.

    Every parameter is opt-in: a key is only rewritten when its argument is
    explicitly supplied (not ``None``). This lets a targeted fix (e.g. costmap
    clearing) leave the robot's tuned speeds, AMCL model and goal tolerances
    alone. Components of a velocity/accel triple that aren't supplied are read
    back from the existing file rather than reset to a default.
    """
    is_mecanum = (str(base_type).strip().lower() == 'mecanum') if base_type is not None else None

    # --- Kinematics: only when a base type is explicitly requested -----------
    if base_type is not None:
        model = '"nav2_amcl::OmniMotionModel"' if is_mecanum else '"nav2_amcl::DifferentialMotionModel"'
        text = re.sub(r'(robot_model_type:\s*)[^\n]+', rf'\g<1>{model}', text)
        y_thresh = 0.001 if is_mecanum else 0.5
        text = re.sub(r'(min_y_velocity_threshold:\s*)[^\n]+', rf'\g<1>{y_thresh}', text)
        if is_mecanum:
            text = re.sub(r'(\bvy_samples:\s*)[^\n]+', r'\g<1>20', text)

    # --- Velocity limits (velocity_smoother + DWB) --------------------------
    vel_requested = any(v is not None for v in (max_vel_x, max_vel_y, max_vel_theta))
    if vel_requested:
        cur = _read_vec3(text, 'max_velocity', (0.5, 0.0, 2.5))
        vx = float(max_vel_x) if max_vel_x is not None else cur[0]
        vth = float(max_vel_theta) if max_vel_theta is not None else cur[2]
        if max_vel_y is not None:
            vy = float(max_vel_y)
        elif is_mecanum:
            # holonomic base with no explicit lateral limit -> mirror the forward limit
            vy = vx if max_vel_x is not None else cur[1]
        else:
            vy = cur[1]
        if is_mecanum is False:
            vy = 0.0
        text = re.sub(r'(max_velocity:\s*)\[[^\]]+\]', rf'\g<1>[{vx}, {vy}, {vth}]', text)
        text = re.sub(r'(min_velocity:\s*)\[[^\]]+\]',
                      rf'\g<1>[{_neg(vx)}, {_neg(vy)}, {_neg(vth)}]', text)
        max_speed_xy = round(math.sqrt(vx ** 2 + vy ** 2), 3) if is_mecanum else vx
        text = re.sub(r'(\bmax_vel_x:\s*)[^\n]+', rf'\g<1>{vx}', text)
        text = re.sub(r'(\bmax_vel_y:\s*)[^\n]+', rf'\g<1>{vy}', text)
        text = re.sub(r'(\bmax_vel_theta:\s*)[^\n]+', rf'\g<1>{vth}', text)
        text = re.sub(r'(\bmax_speed_xy:\s*)[^\n]+', rf'\g<1>{max_speed_xy}', text)
        if desired_linear_vel is None and max_vel_x is not None:
            text = _set_scalar(text, 'desired_linear_vel', round(vx * 0.8, 3))
        if rotate_to_heading_angular_vel is None and max_vel_theta is not None:
            text = re.sub(r'(rotate_to_heading_angular_vel:\s*)[^\n]+',
                          rf'\g<1>{round(vth * 0.72, 3)}', text)

    # --- Acceleration / deceleration limits -------------------------------
    acc_requested = any(v is not None for v in (max_accel_x, max_accel_y, max_accel_theta,
                                                max_decel_x, max_decel_theta))
    if acc_requested:
        cur = _read_vec3(text, 'max_accel', (2.5, 0.0, 3.2))
        ax = float(max_accel_x) if max_accel_x is not None else cur[0]
        ath = float(max_accel_theta) if max_accel_theta is not None else cur[2]
        if max_accel_y is not None:
            ay = float(max_accel_y)
        elif is_mecanum:
            ay = ax if max_accel_x is not None else cur[1]
        else:
            ay = cur[1]
        if is_mecanum is False:
            ay = 0.0
        dec_x = abs(float(max_decel_x)) if max_decel_x is not None else ax
        dec_th = abs(float(max_decel_theta)) if max_decel_theta is not None else ath
        text = re.sub(r'(max_accel:\s*)\[[^\]]+\]', rf'\g<1>[{ax}, {ay}, {ath}]', text)
        text = re.sub(r'(max_decel:\s*)\[[^\]]+\]',
                      rf'\g<1>[{_neg(dec_x)}, {_neg(ay)}, {_neg(dec_th)}]', text)
        text = re.sub(r'(\bacc_lim_x:\s*)[^\n]+', rf'\g<1>{ax}', text)
        text = re.sub(r'(\bacc_lim_y:\s*)[^\n]+', rf'\g<1>{ay}', text)
        text = re.sub(r'(\bacc_lim_theta:\s*)[^\n]+', rf'\g<1>{ath}', text)
        text = re.sub(r'(\bdecel_lim_x:\s*)[^\n]+', rf'\g<1>{_neg(dec_x)}', text)
        text = re.sub(r'(\bdecel_lim_y:\s*)[^\n]+', rf'\g<1>{_neg(ay)}', text)
        text = re.sub(r'(\bdecel_lim_theta:\s*)[^\n]+', rf'\g<1>{_neg(dec_th)}', text)
        if max_accel_theta is not None:
            text = re.sub(r'(max_angular_accel:\s*)[^\n]+', rf'\g<1>{ath}', text)

    # --- Explicit scalar overrides (each opt-in) --------------------------
    if desired_linear_vel is not None:
        text = _set_scalar(text, 'desired_linear_vel', desired_linear_vel)
    if rotate_to_heading_angular_vel is not None:
        text = re.sub(r'(rotate_to_heading_angular_vel:\s*)[^\n]+',
                      rf'\g<1>{_fmt(rotate_to_heading_angular_vel)}', text)
    text = _set_scalar(text, 'angular_dist_threshold', angular_dist_threshold)
    text = _set_scalar(text, 'xy_goal_tolerance', xy_goal_tolerance)
    text = _set_scalar(text, 'yaw_goal_tolerance', yaw_goal_tolerance)
    text = _set_scalar(text, 'lookahead_dist', lookahead_dist)
    text = _set_scalar(text, 'approach_velocity_scaling_dist', approach_velocity_scaling_dist)
    text = _set_scalar(text, 'movement_time_allowance', movement_time_allowance)
    text = _set_scalar(text, 'required_movement_radius', required_movement_radius)
    text = _set_scalar(text, 'inflation_radius', inflation_radius)
    text = _set_scalar(text, 'cost_scaling_factor', cost_scaling_factor)

    # Costmap raytrace and obstacle clearing (Upstream Issue #37)
    if raytrace_range is not None:
        text = re.sub(r'(\braytrace_range:\s*)[^\n]+', rf'\g<1>{float(raytrace_range)}', text)
    if obstacle_max_range is not None:
        text = re.sub(r'(\bobstacle_max_range:\s*)[^\n]+', rf'\g<1>{float(obstacle_max_range)}', text)

    return text


def patch_ekf_text(text, base_type=None, frequency=None, two_d_mode=None,
                   fuse_vy=None, fuse_imu_yaw=None, fuse_imu_vyaw=None):
    """Patch EKF YAML text for base kinematics and sensor fusion options.

    Opt-in like :func:`patch_nav2_text`: ``frequency`` / ``two_d_mode`` are only
    rewritten when supplied, and the ``odom0`` / ``imu0`` masks are only touched
    when a relevant fusion flag (or ``base_type``) is given.
    """
    is_mecanum = (str(base_type).strip().lower() == 'mecanum') if base_type is not None else None
    if fuse_vy is None and base_type is not None:
        fuse_vy = bool(is_mecanum)

    # Update frequency
    if frequency is not None:
        text = re.sub(r'(frequency:\s*)[^\n]+', rf'\g<1>{float(frequency)}', text)

    # Update two_d_mode
    if two_d_mode is not None:
        two_d_val = 'true' if two_d_mode else 'false'
        text = re.sub(r'(two_d_mode:\s*)[^\n]+', rf'\g<1>{two_d_val}', text)

    # robot_localization fusion masks are 15-element boolean lists in the order
    #   x y z  roll pitch yaw  vx vy vz  vroll vpitch vyaw  ax ay az
    # Edit by index so any indentation / line wrapping / row order survives.
    if fuse_vy is not None:
        text, _ = _set_bool_list_index(text, 'odom0_config', 7, fuse_vy)   # vy
    if fuse_imu_yaw is not None:
        text, _ = _set_bool_list_index(text, 'imu0_config', 5, fuse_imu_yaw)   # yaw
    if fuse_imu_vyaw is not None:
        text, _ = _set_bool_list_index(text, 'imu0_config', 11, fuse_imu_vyaw)  # vyaw

    return text


def patch_slam_text(text, resolution=None, max_laser_range=None,
                    minimum_travel_distance=None, minimum_travel_heading=None,
                    loop_search_maximum_distance=None, map_update_interval=None):
    """Patch SLAM Toolbox YAML configuration."""
    if resolution is not None:
        text = re.sub(r'(resolution:\s*)[^\n]+', rf'\g<1>{float(resolution)}', text)
    if max_laser_range is not None:
        text = re.sub(r'(max_laser_range:\s*)[^\n]+', rf'\g<1>{float(max_laser_range)}', text)
        text = re.sub(r'(scan_buffer_maximum_scan_distance:\s*)[^\n]+', rf'\g<1>{float(max_laser_range)}', text)
    if minimum_travel_distance is not None:
        text = re.sub(r'(minimum_travel_distance:\s*)[^\n]+', rf'\g<1>{float(minimum_travel_distance)}', text)
    if minimum_travel_heading is not None:
        text = re.sub(r'(minimum_travel_heading:\s*)[^\n]+', rf'\g<1>{float(minimum_travel_heading)}', text)
    if loop_search_maximum_distance is not None:
        text = re.sub(r'(loop_search_maximum_distance:\s*)[^\n]+', rf'\g<1>{float(loop_search_maximum_distance)}', text)
    if map_update_interval is not None:
        text = re.sub(r'(map_update_interval:\s*)[^\n]+', rf'\g<1>{float(map_update_interval)}', text)
    return text


# Costmap layers carry `observation_sources: scan pointcloud` in the jazzy+
# templates. The depth `pointcloud:` block is always defined
# but is inert unless listed here -- so toggling this one token is how the
# console gates the depth camera in/out of the costmap on the selected robot.
_OBS_SCAN_ONLY = re.compile(r'(?m)^([ \t]*observation_sources:[ \t]*)scan[ \t]*$')
_OBS_SCAN_PC = re.compile(r'(?m)^([ \t]*observation_sources:[ \t]*)scan[ \t]+pointcloud[ \t]*$')


def patch_costmap_sources(text, depth_enabled):
    """Add/remove `pointcloud` from every scalar `observation_sources: scan` line.

    The `["scan"]` list form (collision_monitor -- a fast safety loop kept
    lidar-only by design) is never touched.
    """
    if depth_enabled:
        return _OBS_SCAN_ONLY.sub(r'\1scan pointcloud', text)
    return _OBS_SCAN_PC.sub(r'\1scan', text)


def costmap_depth_active(text):
    """True if any costmap layer currently consumes the depth pointcloud."""
    return bool(_OBS_SCAN_PC.search(text))


def patch_nav2_file(input_path, output_path=None, **kwargs):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f'File not found: {input_path}')
    with open(input_path, 'r') as f:
        content = f.read()
    patched = patch_nav2_text(content, **kwargs)
    out_target = output_path if output_path else input_path
    with open(out_target, 'w') as f:
        f.write(patched)
    return out_target


def patch_ekf_file(input_path, output_path=None, **kwargs):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f'File not found: {input_path}')
    with open(input_path, 'r') as f:
        content = f.read()
    patched = patch_ekf_text(content, **kwargs)
    out_target = output_path if output_path else input_path
    with open(out_target, 'w') as f:
        f.write(patched)
    return out_target


def patch_slam_file(input_path, output_path=None, **kwargs):
    if not os.path.exists(input_path):
        raise FileNotFoundError(f'File not found: {input_path}')
    with open(input_path, 'r') as f:
        content = f.read()
    patched = patch_slam_text(content, **kwargs)
    out_target = output_path if output_path else input_path
    with open(out_target, 'w') as f:
        f.write(patched)
    return out_target


def main():
    parser = argparse.ArgumentParser(description='Patch Nav2, EKF or SLAM YAML configuration')
    subparsers = parser.add_subparsers(dest='command', required=True)

    # Nav2 subparser
    p_nav2 = subparsers.add_parser('nav2', help='Patch Nav2 parameters')
    p_nav2.add_argument('-i', '--input', required=True, help='Input Nav2 YAML file')
    p_nav2.add_argument('-o', '--output', help='Output Nav2 YAML file')
    p_nav2.add_argument('-b', '--base', default='2wd', choices=['2wd', '4wd', 'mecanum'],
                        help='Robot base type (2wd, 4wd, mecanum)')
    p_nav2.add_argument('--max-vel-x', type=float, default=None, help='Max linear velocity (m/s)')
    p_nav2.add_argument('--max-vel-y', type=float, default=None, help='Max lateral velocity (m/s)')
    p_nav2.add_argument('--max-vel-theta', type=float, default=None, help='Max angular velocity (rad/s)')
    p_nav2.add_argument('--max-accel-x', type=float, default=None, help='Max linear accel (m/s^2)')
    p_nav2.add_argument('--max-accel-y', type=float, default=None, help='Max lateral accel (m/s^2)')
    p_nav2.add_argument('--max-accel-theta', type=float, default=None, help='Max angular accel (rad/s^2)')
    p_nav2.add_argument('--desired-linear-vel', type=float, default=None, help='Desired linear tracking speed')
    p_nav2.add_argument('--inflation-radius', type=float, default=None, help='Obstacle inflation radius (m)')
    p_nav2.add_argument('--cost-scaling-factor', type=float, default=None, help='Cost scaling factor')

    # EKF subparser
    p_ekf = subparsers.add_parser('ekf', help='Patch EKF parameters')
    p_ekf.add_argument('-i', '--input', required=True, help='Input EKF YAML file')
    p_ekf.add_argument('-o', '--output', help='Output EKF YAML file')
    p_ekf.add_argument('-b', '--base', default='2wd', choices=['2wd', '4wd', 'mecanum'],
                       help='Robot base type')
    p_ekf.add_argument('--frequency', type=float, default=None, help='EKF update frequency (Hz)')
    p_ekf.add_argument('--fuse-vy', action='store_true', default=None, help='Explicitly fuse lateral velocity')
    p_ekf.add_argument('--no-fuse-vy', dest='fuse_vy', action='store_false', help='Disable lateral velocity fusion')
    p_ekf.add_argument('--fuse-imu-yaw', action='store_true', default=None, help='Fuse IMU orientation yaw')
    p_ekf.add_argument('--two-d-mode', action='store_true', default=None, help='Enable 2D planar mode')

    # SLAM subparser
    p_slam = subparsers.add_parser('slam', help='Patch SLAM parameters')
    p_slam.add_argument('-i', '--input', required=True, help='Input SLAM YAML file')
    p_slam.add_argument('-o', '--output', help='Output SLAM YAML file')
    p_slam.add_argument('--resolution', type=float, default=None, help='Map resolution (m/pixel)')
    p_slam.add_argument('--max-laser-range', type=float, default=None, help='Laser max range (m)')
    p_slam.add_argument('--min-travel-dist', type=float, default=None, help='Min travel distance for keyframe (m)')

    # Preset subparser
    p_preset = subparsers.add_parser('preset', help='Apply tuning preset across configs')
    p_preset.add_argument('--preset', required=True, choices=list(PRESETS.keys()), help='Preset name')
    p_preset.add_argument('--nav2-file', help='Nav2 YAML file to patch')
    p_preset.add_argument('--ekf-file', help='EKF YAML file to patch')
    p_preset.add_argument('--slam-file', help='SLAM YAML file to patch')

    args = parser.parse_args()

    if args.command == 'nav2':
        out = patch_nav2_file(
            args.input,
            args.output,
            base_type=args.base,
            max_vel_x=args.max_vel_x,
            max_vel_y=args.max_vel_y,
            max_vel_theta=args.max_vel_theta,
            max_accel_x=args.max_accel_x,
            max_accel_y=args.max_accel_y,
            max_accel_theta=args.max_accel_theta,
            desired_linear_vel=args.desired_linear_vel,
            inflation_radius=args.inflation_radius,
            cost_scaling_factor=args.cost_scaling_factor
        )
        print(f'Successfully patched Nav2 configuration -> {out}')

    elif args.command == 'ekf':
        out = patch_ekf_file(
            args.input,
            args.output,
            base_type=args.base,
            frequency=args.frequency,
            two_d_mode=args.two_d_mode,
            fuse_vy=args.fuse_vy,
            fuse_imu_yaw=args.fuse_imu_yaw
        )
        print(f'Successfully patched EKF configuration -> {out}')

    elif args.command == 'slam':
        out = patch_slam_file(
            args.input,
            args.output,
            resolution=args.resolution,
            max_laser_range=args.max_laser_range,
            minimum_travel_distance=args.min_travel_dist
        )
        print(f'Successfully patched SLAM configuration -> {out}')

    elif args.command == 'preset':
        cfg = PRESETS[args.preset]
        print(f"Applying preset '{args.preset}': {cfg['label']}")
        if args.nav2_file:
            patch_nav2_file(args.nav2_file, **nav2_kwargs(cfg))
            print(f"  -> Nav2 patched: {args.nav2_file}")
        if args.ekf_file:
            patch_ekf_file(
                args.ekf_file,
                base_type=cfg["base"],
                frequency=cfg["ekf_frequency"],
                fuse_vy=cfg["fuse_vy"],
                fuse_imu_yaw=cfg["fuse_imu_yaw"],
            )
            print(f"  -> EKF patched: {args.ekf_file}")
        if args.slam_file:
            patch_slam_file(
                args.slam_file,
                resolution=cfg["slam_resolution"],
                max_laser_range=cfg["slam_max_range"],
            )
            print(f"  -> SLAM patched: {args.slam_file}")


if __name__ == '__main__':
    main()

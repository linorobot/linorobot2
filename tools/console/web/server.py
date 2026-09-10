#!/usr/bin/env python3
"""Linorobot2 Console -- zero-dependency local web UI for the ROS2/robot-computer
side of linorobot2: installing the package + sensor drivers, launching bringup/
teleop/SLAM/navigation, running magnetometer calibration, and a live LiDAR
viewer. Mirrors the server architecture of linorobot2_hardware's
tools/robot_config_engine/web/server.py: a generic SSE command runner the
frontend drives by generating shell command strings, not a purpose-built API
per action.

Usage: python3 server.py [port]
"""
import collections
import json
import queue
import math
import os
import platform
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    import patcher
except Exception:
    patcher = None
try:
    import yaml_merge
except Exception:
    yaml_merge = None

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # tools/console/..
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(WEB_DIR, "console_config.json")  # DEPRECATED: legacy migration source only
CONFIG_DIR = os.path.join(os.path.dirname(WEB_DIR), "config")
LINOROBOT2_ROOT = os.path.abspath(os.path.join(WEB_DIR, "../../.."))
SUPPORTED_DISTROS = ["jazzy", "lyrical", "rolling"]  # Humble dropped upstream (linorobot)
NAV2_CONFIG_PATH = os.path.join(WEB_DIR, "console_nav2_jazzy.yaml")

# ---------------------------------------------------------------------------
# Repo-based robot config: the single source of truth is
#   <linorobot2>/tools/console/config/<robot_name>_config.yaml
# One file per robot (multi-robot). It carries a `console:` section (workflow
# settings -- former console_config.json) plus the existing
# linorobot2:/nav2:/ekf:/slam: sections. The active robot is named in
# config/.active_robot. `~/.config/linorobot2/robot_config.yaml` is a legacy
# migration source only (never written after migration).
# ---------------------------------------------------------------------------
ROBOT_CONFIGS_DIR = os.path.join(os.path.dirname(WEB_DIR), "config")
ACTIVE_ROBOT_FILE = os.path.join(ROBOT_CONFIGS_DIR, ".active_robot")
DEFAULT_ROBOT_NAME = "linorobot2"
LEGACY_ROBOT_CONFIG_YAML_PATH = os.path.expanduser("~/.config/linorobot2/robot_config.yaml")


def _robot_name_ok(name):
    return bool(re.match(r"^[a-z0-9_]+$", name or ""))


def get_active_robot_name():
    """Name of the currently selected robot (config/.active_robot, default 'linorobot2')."""
    try:
        with open(ACTIVE_ROBOT_FILE) as f:
            nm = f.read().strip()
        if _robot_name_ok(nm):
            return nm
    except OSError:
        pass
    return DEFAULT_ROBOT_NAME


def set_active_robot_name(name):
    if not _robot_name_ok(name):
        raise ValueError("invalid robot name: %r" % (name,))
    os.makedirs(ROBOT_CONFIGS_DIR, exist_ok=True)
    with open(ACTIVE_ROBOT_FILE, "w") as f:
        f.write(name + "\n")
    return name


def get_robot_config_path(name=None):
    """Absolute path of <linorobot2>/config/<name>_config.yaml for the given
    (or active) robot."""
    if not _robot_name_ok(name):
        name = get_active_robot_name()
    return os.path.join(ROBOT_CONFIGS_DIR, name + "_config.yaml")


def list_robot_configs():
    """[{name, path, active}] for every *_config.yaml in config/."""
    active = get_active_robot_name()
    out = []
    try:
        names = sorted(os.listdir(ROBOT_CONFIGS_DIR))
    except OSError:
        names = []
    for fn in names:
        if fn.endswith("_config.yaml") and not fn.startswith("."):
            nm = fn[: -len("_config.yaml")]
            out.append({
                "name": nm,
                "path": os.path.join(ROBOT_CONFIGS_DIR, fn),
                "active": nm == active,
            })
    if not any(r["active"] for r in out):
        out.append({
            "name": active,
            "path": get_robot_config_path(active),
            "active": True,
        })
    return out


def _git(*args, cwd=None):
    """Run `git <args>` in the linorobot2 repo; stripped stdout, or '' on failure."""
    try:
        out = subprocess.run(
            ["git", *args], cwd=cwd or LINOROBOT2_ROOT,
            capture_output=True, text=True, timeout=5.0,
        )
        if out.returncode == 0:
            return out.stdout.strip()
    except Exception:
        pass
    return ""


# The commit the web server was started on ("the version we start the web").
GIT_VERSION_AT_START = (_git("rev-parse", "--short=7", "HEAD") or "unknown")[:7]


def collect_git_info():
    """Snapshot of the linorobot2 checkout: short commit, branch, local
    branches (current pinned first), dirty flag, last 10 commits."""
    remotes = []
    for line in _git("remote", "-v").splitlines():
        if "(fetch)" in line:
            parts = line.split()
            if len(parts) >= 2:
                remotes.append({"name": parts[0], "url": parts[1]})
    commits = []
    log = _git("log", "-10", "--pretty=format:%h\x1f%s\x1f%an\x1f%ad\x1f%ar", "--date=short")
    for line in log.splitlines():
        f = line.split("\x1f")
        if len(f) == 5:
            commits.append({
                "hash": f[0], "subject": f[1], "author": f[2],
                "date": f[3], "reldate": f[4],
            })
    cur_branch = _git("rev-parse", "--abbrev-ref", "HEAD") or "(detached)"
    branches = [
        b for b in _git(
            "for-each-ref", "--sort=-committerdate",
            "--format=%(refname:short)", "refs/heads",
        ).splitlines() if b
    ]
    if cur_branch in branches:
        branches = [cur_branch] + [b for b in branches if b != cur_branch]
    version = (_git("rev-parse", "--short=7", "HEAD") or "unknown")[:7]
    return {
        "version": version,
        "full": _git("rev-parse", "HEAD"),
        "version_at_start": GIT_VERSION_AT_START,
        "moved_since_start": (
            version != GIT_VERSION_AT_START
            and GIT_VERSION_AT_START != "unknown"
        ),
        "branch": cur_branch,
        "branches": branches,
        "dirty": bool(_git("status", "--porcelain")),
        "remotes": remotes,
        "commits": commits,
    }


def commit_robot_config_if_dirty(robot_name=None, action_label="action"):
    """Flush the active robot config to disk, then `git add` + `git commit` it
    on the current branch if it changed. Never raises -- a config-commit
    failure must not block the robot action. Returns the new commit hash or ''.
    """
    robot_name = robot_name if _robot_name_ok(robot_name) else get_active_robot_name()
    path = get_robot_config_path(robot_name)
    try:
        _flush_robot_config(robot_name)
    except Exception:
        pass
    if not os.path.exists(path):
        return ""
    try:
        subprocess.run(["git", "-C", LINOROBOT2_ROOT, "add", "--", path],
                       capture_output=True, timeout=5.0)
        changed = subprocess.run(
            ["git", "-C", LINOROBOT2_ROOT, "diff", "--cached", "--quiet", "--", path],
            capture_output=True, timeout=5.0,
        ).returncode != 0
        if not changed:
            return ""
        subprocess.run(
            ["git", "-C", LINOROBOT2_ROOT, "commit", "-m",
             "config(%s): auto-commit before %s" % (robot_name, action_label),
             "--", path],
            capture_output=True, timeout=10.0,
        )
        return (_git("rev-parse", "--short=7", "HEAD") or "")[:7]
    except Exception:
        return ""


def _flush_robot_config(robot_name=None):
    """Write the in-memory console config to the active robot's YAML file so a
    subsequent git commit captures the exact settings used for an action."""
    save_config(load_config(), robot_name=robot_name)


def indent_block(text, spaces=2):
    pad = " " * spaces
    return "\n".join(pad + line if line.strip() else "" for line in text.splitlines())


def generate_unified_yaml(lino_cfg, nav2_yaml, ekf_yaml, slam_yaml, console_cfg=None):
    base = lino_cfg.get("base") or lino_cfg.get("base_type") or "2wd"
    laser = lino_cfg.get("laser_sensor") or lino_cfg.get("laser") or ""
    depth = lino_cfg.get("depth_sensor") or lino_cfg.get("depth") or ""
    name = lino_cfg.get("robot_name") or "linorobot2"
    distro = lino_cfg.get("ros_distro") or detect_ros_distro()
    domain = lino_cfg.get("ros_domain_id", 0)
    transport = lino_cfg.get("micro_ros_transport") or lino_cfg.get("agent_transport") or "serial"
    device = lino_cfg.get("micro_ros_port") or lino_cfg.get("agent_device") or "/dev/ttyACM0"
    baud = lino_cfg.get("micro_ros_baudrate") or lino_cfg.get("agent_baud") or "1500000"
    laser_port = lino_cfg.get("laser_serial_port") or ""
    laser_baud = lino_cfg.get("laser_baud") or ""
    laser_transport = lino_cfg.get("laser_transport") or ""
    laser_udp_port = lino_cfg.get("laser_udp_port") or ""
    depth_port = lino_cfg.get("depth_serial_port") or ""
    madgwick = bool(lino_cfg.get("madgwick", True))

    lines = [
        "# ==============================================================================",
        "# Linorobot2 & Nav2 Unified Configuration",
        "# Auto-generated by Linorobot2 Console",
        "# Single config file for robot setup (kinematics, sensors, micro-ros) + Nav2 + EKF + SLAM",
        "# ==============================================================================",
        "",
    ]
    if console_cfg:
        lines += [
            "# ------------------------------------------------------------------------------",
            "# Console workflow settings (ROS distro, install/agent engine, transport, ...)",
            "# ------------------------------------------------------------------------------",
            render_console_section(console_cfg),
            "",
        ]
    lines += [
        "linorobot2:",
        f'  base: "{base}"',
        f'  laser_sensor: "{laser}"',
        f'  depth_sensor: "{depth}"',
        f'  robot_name: "{name}"',
        f'  ros_distro: "{distro}"',
        f'  ros_domain_id: {domain}',
        f'  micro_ros_transport: "{transport}"',
        f'  micro_ros_port: "{device}"',
        f'  micro_ros_baudrate: {baud}',
        f'  laser_serial_port: "{laser_port}"',
        f'  laser_baud: "{laser_baud}"',
        f'  laser_transport: "{laser_transport}"',
        f'  laser_udp_port: "{laser_udp_port}"',
        f'  depth_serial_port: "{depth_port}"',
        f'  madgwick: {"true" if madgwick else "false"}',
        "",
        "# ------------------------------------------------------------------------------",
        "# Nav2 Parameters",
        "# ------------------------------------------------------------------------------",
        "nav2:",
        indent_block(nav2_yaml.strip()),
        "",
        "# ------------------------------------------------------------------------------",
        "# EKF Sensor Fusion Parameters",
        "# ------------------------------------------------------------------------------",
        "ekf:",
        indent_block(ekf_yaml.strip()),
        "",
        "# ------------------------------------------------------------------------------",
        "# SLAM Toolbox Parameters",
        "# ------------------------------------------------------------------------------",
        "slam:",
        indent_block(slam_yaml.strip()),
        ""
    ]
    return "\n".join(lines)


def parse_unified_yaml(text):
    sections = {}
    current_section = None
    current_lines = []

    for line in text.splitlines():
        line_clean = line.strip()
        if not line_clean or line_clean.startswith("#"):
            if current_section:
                current_lines.append(line)
            continue
        if ":" in line:
            parts = line.split(":", 1)
            key = parts[0].strip()
            indent = len(line) - len(line.lstrip())
            if indent == 0 and key in ("console", "linorobot2", "nav2", "ekf", "slam"):
                if current_section:
                    sections[current_section] = "\n".join(current_lines)
                current_section = key
                current_lines = []
                continue
            elif indent == 0 and key in ("amcl", "bt_navigator", "controller_server", "planner_server",
                                         "behavior_server", "local_costmap", "global_costmap",
                                         "waypoint_follower", "velocity_smoother", "map_server"):
                if current_section != "nav2":
                    if current_section:
                        sections[current_section] = "\n".join(current_lines)
                    current_section = "nav2"
                    current_lines = []
            elif indent == 0 and key == "ekf_filter_node":
                if current_section:
                    sections[current_section] = "\n".join(current_lines)
                current_section = "ekf"
                current_lines = []
            elif indent == 0 and key == "slam_toolbox":
                if current_section:
                    sections[current_section] = "\n".join(current_lines)
                current_section = "slam"
                current_lines = []

        if current_section:
            current_lines.append(line)

    if current_section:
        sections[current_section] = "\n".join(current_lines)

    def deindent(sec_text):
        lines = sec_text.splitlines()
        indents = [len(l) - len(l.lstrip()) for l in lines if l.strip() and not l.strip().startswith("#")]

        def drop_indent(line, n):
            # Remove at most `n` leading WHITESPACE characters. A blind
            # line[n:] eats real content on any line indented less than the
            # section minimum -- a banner comment sitting at column 0 loses its
            # '#' and silently turns the extracted YAML into a syntax error.
            i = 0
            while i < n and i < len(line) and line[i] in " \t":
                i += 1
            return line[i:]

        if indents:
            min_indent = min(indents)
            if min_indent > 0:
                return "\n".join(drop_indent(l, min_indent) for l in lines)
        return sec_text

    def _scalars(sec_text):
        out = {}
        for line in sec_text.splitlines():
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            k, v = line.split(":", 1)
            k = k.strip()
            v = v.split("#")[0].strip().strip("'\"")
            if v.lower() == "true":
                out[k] = True
            elif v.lower() == "false":
                out[k] = False
            elif v.lstrip("-").isdigit():
                out[k] = int(v)
            else:
                out[k] = v
        return out

    return {
        "console": _scalars(sections.get("console", "")),
        "linorobot2": _scalars(sections.get("linorobot2", "")),
        "nav2": deindent(sections.get("nav2", "")).strip(),
        "ekf": deindent(sections.get("ekf", "")).strip(),
        "slam": deindent(sections.get("slam", "")).strip(),
    }


# Keys of DEFAULT_CONFIG persisted verbatim in the YAML `console:` section.
# (defined here as a name; the value list is built after DEFAULT_CONFIG below)
def _console_yaml_keys():
    return list(DEFAULT_CONFIG.keys())


def render_console_section(cfg):
    """The top-level `console:` YAML block (workflow settings). Scalar only."""
    lines = ["console:"]
    for k in _console_yaml_keys():
        v = cfg.get(k, DEFAULT_CONFIG[k])
        if isinstance(v, bool):
            lines.append("  %s: %s" % (k, "true" if v else "false"))
        elif isinstance(v, (int, float)):
            lines.append("  %s: %s" % (k, v))
        else:
            lines.append('  %s: "%s"' % (k, v))
    return "\n".join(lines)


def splice_yaml_section(text, section_name, section_block):
    """Return `text` with the top-level `<section_name>:` block replaced by
    `section_block` (no trailing newline). Inserts at the top if absent."""
    section_block = section_block.rstrip("\n")
    lines = (text or "").splitlines()
    start = None
    for i, ln in enumerate(lines):
        if re.match(r"^%s:\s*$" % re.escape(section_name), ln) or ln.strip() == section_name + ":":
            if len(ln) - len(ln.lstrip()) == 0:
                start = i
                break
    if start is None:
        prefix = section_block + "\n"
        if text and not text.startswith("\n"):
            prefix += "\n"
        return prefix + (text or "")
    end = len(lines)
    for j in range(start + 1, len(lines)):
        ln = lines[j]
        # Section ends at the first line that is neither blank nor indented
        # (a column-0 comment or a new top-level key), so surrounding banner
        # comments are preserved.
        if ln.strip() and not ln.startswith((" ", "\t")):
            end = j
            break
    # Trim trailing blank lines that belonged to the old section.
    while end - 1 > start and not lines[end - 1].strip():
        end -= 1
    return "\n".join(lines[:start] + section_block.splitlines() + lines[end:])


def get_unified_config(distro=None, base=None):
    distro = distro or detect_ros_distro()
    cfg = load_config()
    base = base or cfg.get("base_type", "2wd")
    robot_config_path = get_robot_config_path()

    if os.path.exists(robot_config_path):
        try:
            with open(robot_config_path, "r") as f:
                content = f.read()
            parsed = parse_unified_yaml(content)
            lino = parsed.get("linorobot2") or {}
            # Ensure the file always carries an up-to-date console: block.
            if not parsed.get("console"):
                content = splice_yaml_section(content, "console", render_console_section(cfg))
            return {
                "yaml": content,
                "path": robot_config_path,
                "distro": distro,
                "base": lino.get("base", base),
                "linorobot2": lino,
            }
        except Exception:
            pass

    lino_cfg = {
        "base": base,
        "laser_sensor": cfg.get("laser_sensor", ""),
        "depth_sensor": cfg.get("depth_sensor", ""),
        "robot_name": cfg.get("robot_name", "linorobot2"),
        "ros_distro": distro,
        "ros_domain_id": cfg.get("ros_domain_id", 0),
        "micro_ros_transport": cfg.get("agent_transport", "serial"),
        "micro_ros_port": cfg.get("agent_device", "/dev/ttyACM0"),
        "micro_ros_baudrate": cfg.get("agent_baud", "1500000"),
        "laser_serial_port": cfg.get("laser_serial_port", ""),
        "laser_baud": cfg.get("laser_baud", ""),
        "laser_transport": cfg.get("laser_transport", ""),
        "laser_udp_port": cfg.get("laser_udp_port", ""),
        "depth_serial_port": cfg.get("depth_serial_port", ""),
        "madgwick": cfg.get("madgwick", True),
    }
    nav2_yaml = get_nav2_config(distro)
    ekf_yaml = get_ekf_config(base)
    slam_yaml = get_slam_config()
    unified_yaml = generate_unified_yaml(lino_cfg, nav2_yaml, ekf_yaml, slam_yaml, console_cfg=cfg)
    return {
        "yaml": unified_yaml,
        "path": robot_config_path,
        "distro": distro,
        "base": base,
        "linorobot2": lino_cfg,
    }


def save_unified_config(unified_data, distro=None, base=None):
    distro = distro or detect_ros_distro()
    if isinstance(unified_data, dict) and "yaml" in unified_data:
        text = unified_data["yaml"]
    elif isinstance(unified_data, str):
        text = unified_data
    elif isinstance(unified_data, dict):
        cur = get_unified_config(distro=distro, base=base)
        parsed_cur = parse_unified_yaml(cur["yaml"])
        lino = parsed_cur.get("linorobot2", {})
        lino.update({k: v for k, v in unified_data.items() if k in ("base", "laser_sensor", "depth_sensor", "robot_name", "ros_domain_id", "micro_ros_transport", "micro_ros_port", "micro_ros_baudrate", "madgwick")})
        text = generate_unified_yaml(lino, parsed_cur.get("nav2", ""), parsed_cur.get("ekf", ""), parsed_cur.get("slam", ""))
    else:
        text = ""

    parsed = parse_unified_yaml(text)
    lino_cfg = parsed.get("linorobot2") or {}
    base = lino_cfg.get("base") or base or "2wd"

    # 1. Update active console_config.json
    cfg = load_config()
    if "base" in lino_cfg:
        cfg["base_type"] = lino_cfg["base"]
    if "laser_sensor" in lino_cfg:
        cfg["laser_sensor"] = lino_cfg["laser_sensor"]
    if "depth_sensor" in lino_cfg:
        cfg["depth_sensor"] = lino_cfg["depth_sensor"]
    if "robot_name" in lino_cfg:
        cfg["robot_name"] = lino_cfg["robot_name"]
    if "micro_ros_transport" in lino_cfg:
        cfg["agent_transport"] = lino_cfg["micro_ros_transport"]
    if "micro_ros_port" in lino_cfg:
        cfg["agent_device"] = lino_cfg["micro_ros_port"]
    if "micro_ros_baudrate" in lino_cfg:
        cfg["agent_baud"] = str(lino_cfg["micro_ros_baudrate"])
    if "madgwick" in lino_cfg:
        cfg["madgwick"] = bool(lino_cfg["madgwick"])
    # Adopt any console: section carried in the incoming YAML.
    for k, v in (parsed.get("console") or {}).items():
        if k in DEFAULT_CONFIG:
            cfg[k] = v

    # 2. Save Nav2 if present
    nav2_text = parsed.get("nav2", "")
    if nav2_text.strip():
        save_nav2_config(nav2_text, distro)

    # 3. Save EKF if present
    ekf_text = parsed.get("ekf", "")
    if ekf_text.strip():
        save_ekf_config(ekf_text)

    # 4. Save SLAM if present
    slam_text = parsed.get("slam", "")
    if slam_text.strip():
        save_slam_config(slam_text)

    # 5. Persist unified yaml to disk (always carries an up-to-date console: block)
    if not (parsed.get("console")):
        text = splice_yaml_section(text, "console", render_console_section(cfg))
    robot_config_path = get_robot_config_path()
    os.makedirs(os.path.dirname(robot_config_path), exist_ok=True)
    with open(robot_config_path, "w") as f:
        f.write(text if text.endswith("\n") else text + "\n")
    # Mirror the console: section into the flat cache used by load_config().
    save_config(cfg)

    return {
        "status": "saved",
        "path": robot_config_path,
        "distro": distro,
        "base": base,
        "linorobot2_updated": bool(lino_cfg),
        "nav2_updated": bool(nav2_text.strip()),
        "ekf_updated": bool(ekf_text.strip()),
        "slam_updated": bool(slam_text.strip()),
        "config": cfg,
    }



def get_nav2_config_path(distro=None):
    if not distro or distro not in SUPPORTED_DISTROS:
        distro = detect_ros_distro()
    return os.path.join(WEB_DIR, f"console_nav2_{distro}.yaml")


def get_nav2_default_path(distro=None):
    if not distro or distro not in SUPPORTED_DISTROS:
        distro = detect_ros_distro()
    distro_tpl = os.path.join(CONFIG_DIR, f"nav2_{distro}.yaml")
    if os.path.exists(distro_tpl):
        return distro_tpl
    return os.path.join(LINOROBOT2_ROOT, "linorobot2_navigation", "config", "navigation.yaml")


def get_nav2_config(distro=None):
    path = get_nav2_config_path(distro)
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return f.read()
        except Exception:
            pass
    tpl_path = get_nav2_default_path(distro)
    if os.path.exists(tpl_path):
        try:
            with open(tpl_path, "r") as f:
                return f.read()
        except Exception:
            pass
    return f"# Linorobot2 Nav2 Parameters ({distro})\n"


def save_nav2_config(text, distro=None):
    path = get_nav2_config_path(distro)
    with open(path, "w") as f:
        f.write(text)
    return path

def get_ekf_config_path():
    return os.path.join(WEB_DIR, "console_ekf.yaml")


def get_ekf_default_path(base=None):
    if not base:
        base = "2wd"
    base = base.lower()
    base_tpl = os.path.join(CONFIG_DIR, f"ekf_{base}.yaml")
    if os.path.exists(base_tpl):
        return base_tpl
    pkg_tpl = os.path.join(LINOROBOT2_ROOT, "linorobot2_base", "config", f"ekf_{base}.yaml")
    if os.path.exists(pkg_tpl):
        return pkg_tpl
    return os.path.join(LINOROBOT2_ROOT, "linorobot2_base", "config", "ekf.yaml")


def get_ekf_config(base=None):
    path = get_ekf_config_path()
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return f.read()
        except Exception:
            pass
    tpl = get_ekf_default_path(base)
    if os.path.exists(tpl):
        try:
            with open(tpl, "r") as f:
                return f.read()
        except Exception:
            pass
    return "# Linorobot2 EKF Parameters"


def save_ekf_config(text):
    path = get_ekf_config_path()
    with open(path, "w") as f:
        f.write(text)
    return path


def get_slam_config_path():
    return os.path.join(WEB_DIR, "console_slam.yaml")


def get_slam_default_path():
    tpl = os.path.join(CONFIG_DIR, "slam.yaml")
    if os.path.exists(tpl):
        return tpl
    return os.path.join(LINOROBOT2_ROOT, "linorobot2_navigation", "config", "slam.yaml")


def get_slam_config():
    path = get_slam_config_path()
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                return f.read()
        except Exception:
            pass
    tpl = get_slam_default_path()
    if os.path.exists(tpl):
        try:
            with open(tpl, "r") as f:
                return f.read()
        except Exception:
            pass
    return "# Linorobot2 SLAM Parameters"


def save_slam_config(text):
    path = get_slam_config_path()
    with open(path, "w") as f:
        f.write(text)
    return path


def analyze_robotics_ai(prompt, base="2wd", distro="jazzy", model=None):
    """Analyze robotics navigation/estimation problem and generate parameter patches."""
    p = prompt.lower()
    diagnosis = []
    recommendations = []
    nav2_patch = {}
    ekf_patch = {}
    slam_patch = {}

    target_base = base
    if any(k in p for k in ["mecanum", "omni", "holonomic"]) or (any(k in p for k in ["strafe", "lateral"]) and not any(k in p for k in ["drift", "slip", "wander"])):
        target_base = "mecanum"
        diagnosis.append("Robot requires holonomic (omnidirectional) kinematics: lateral strafe is enabled in velocity_smoother, AMCL OmniMotionModel is set, and EKF odom0 vy is fused.")
        recommendations.append("Enable lateral velocity (v_y = 0.5 m/s) and lateral acceleration in velocity_smoother.")
        recommendations.append("Switch AMCL robot_model_type to nav2_amcl::OmniMotionModel.")
        recommendations.append("Set EKF odom0_config to fuse lateral velocity (v_y).")
        nav2_patch.update({
            "base": "mecanum",
            "max_vel_x": 0.5,
            "max_vel_y": 0.5,
            "max_accel_x": 2.5,
            "max_accel_y": 2.5
        })
        ekf_patch["fuse_vy"] = True
    elif any(k in p for k in ["diff", "2wd", "4wd", "skid"]):
        target_base = "2wd"
        nav2_patch.update({
            "base": "2wd",
            "max_vel_y": 0.0,
            "max_accel_y": 0.0
        })
        ekf_patch["fuse_vy"] = False

    if any(k in p for k in ["door", "narrow", "tight", "hallway", "corridor", "hesitat", "stuck in door"]):
        diagnosis.append("Doorway hesitation is caused by default inflation radius (0.7m) creating overlapping high-cost gradients across narrow passages (<90cm).")
        recommendations.append("Reduce costmap inflation_radius to 0.52m.")
        recommendations.append("Increase cost_scaling_factor to 5.5 so wall penalty falls off sharply, opening a clear navigable path through the doorway center.")
        nav2_patch.update({
            "inflation_radius": 0.52,
            "cost_scaling_factor": 5.5,
            "desired_linear_vel": 0.35
        })

    if any(k in p for k in ["oscillat", "hunting", "wobble", "spin at goal", "overshoot", "shake", "jitter"]):
        diagnosis.append("End-goal oscillation is typically caused by high rotational acceleration and angular velocity overpowering the goal tolerance window.")
        recommendations.append("Reduce rotate_to_heading_angular_vel to 1.2 rad/s.")
        recommendations.append("Smooth max_angular_accel to 2.2 rad/s² to suppress motor hunting.")
        nav2_patch.update({
            "max_vel_theta": 2.0,
            "max_accel_theta": 2.2
        })

    # 1. DRIFT / STATE ESTIMATION ISSUE
    if any(k in p for k in ["drift", "ekf", "slip", "spinning drift", "lateral drift", "heading drift", "yaw drift", "wandering", "in-place drift"]):
        diagnosis.append("State estimation drift detected (lateral wandering or orientation error). In differential/skid robots, fusing lateral velocity (v_y) causes artificial sideways displacement from wheel slip during in-place spins. Low EKF frequency causes numerical integration error.")
        recommendations.append("Disable lateral velocity (v_y) in EKF odom0_config for differential/skid robots to eliminate spin slip.")
        recommendations.append("Enforce 2D planar mode (two_d_mode = true) and standardize EKF frequency to 50 Hz to match micro-ROS loop rate.")
        recommendations.append("Set controller_server min_y_velocity_threshold to 0.5 to filter encoder quantization noise.")
        if target_base != "mecanum":
            ekf_patch["fuse_vy"] = False
        ekf_patch["two_d_mode"] = True
        ekf_patch["frequency"] = 50.0

    # 2. OVERSHOOT / BRAKING ISSUE
    if any(k in p for k in ["overshoot", "blow past", "late braking", "fly by", "past goal", "stopping distance", "cant brake", "braking authority", "runaway"]):
        diagnosis.append("Goal overshoot and late braking detected. The robot carries excess kinetic momentum into the destination zone due to loose deceleration limits (-1.0 m/s²) or lack of approach velocity scaling in the local controller.")
        recommendations.append("Increase braking authority in velocity_smoother: max_decel = [-2.8 m/s², 0.0, -3.5 rad/s²].")
        recommendations.append("Enable Regulated Pure Pursuit approach velocity scaling starting at 0.75m from goal.")
        recommendations.append("Shorten lookahead distance near goal to 0.45m to prevent trajectory over-prediction.")
        recommendations.append("Set realistic goal tolerance (xy_goal_tolerance = 0.08m, yaw_goal_tolerance = 0.12 rad).")
        nav2_patch.update({
            "max_decel_x": 2.8,
            "max_decel_theta": 3.5,
            "approach_velocity_scaling_dist": 0.75,
            "lookahead_dist": 0.45,
            "xy_goal_tolerance": 0.08,
            "yaw_goal_tolerance": 0.12
        })

    # 3. UNABLE TO REACH DESTINATION / STUCK BEFORE GOAL
    if any(k in p for k in ["unable to reach", "cant reach", "dest", "destination", "stuck before goal", "goal timeout", "tolerance timeout", "aborted goal", "hunting at goal", "unreachable"]):
        diagnosis.append("Robot unable to complete navigation to destination (times out near goal, oscillates endlessly, or aborts path). Typically caused by overly strict goal tolerances (<5cm) triggering controller patience timeouts, or narrow corridor inflation overlap treating the destination as lethal.")
        recommendations.append("Expand goal tolerance window to 0.08m (8cm) and 0.12 rad (~7°) to accommodate real-world encoder backlash.")
        recommendations.append("Increase controller progress allowance: movement_time_allowance = 15.0s, required_movement_radius = 0.15m.")
        recommendations.append("Reduce costmap inflation_radius to 0.52m and steepen cost_scaling_factor to 5.5 so destination poses near walls are not marked lethal.")
        nav2_patch.update({
            "xy_goal_tolerance": 0.08,
            "yaw_goal_tolerance": 0.12,
            "movement_time_allowance": 15.0,
            "required_movement_radius": 0.15,
            "inflation_radius": 0.52,
            "cost_scaling_factor": 5.5
        })

    if any(k in p for k in ["fast", "speed", "quick", "warehouse", "large", "open", "accelerat"]):
        diagnosis.append("Optimizing parameter envelope for high-speed transit in open warehouse/arena environments.")
        recommendations.append("Increase linear velocity limit to 0.8 m/s and linear acceleration to 3.0 m/s².")
        nav2_patch.update({
            "max_vel_x": 0.8,
            "max_accel_x": 3.0,
            "desired_linear_vel": 0.65
        })

    if any(k in p for k in ["slam", "map", "blur", "smear", "resolution", "loop closure"]):
        diagnosis.append("Enhancing SLAM scan-matching density to prevent rotational map smearing and capture fine geometry.")
        recommendations.append("Increase SLAM resolution to 0.035m and decrease keyframe travel heading to 0.3 rad.")
        slam_patch.update({
            "resolution": 0.035,
            "max_laser_range": 12.0,
            "minimum_travel_heading": 0.3,
            "minimum_travel_distance": 0.3
        })

    # 4. ROTATION / IN-PLACE SPIN / ROTATIONAL OSCILLATION ISSUE
    if any(k in p for k in ["rotation", "spin in place", "pivot", "turn in place", "rotational oscillation", "hunting", "angular wobble", "rotation shim", "yaw hunting", "head shake", "spinning"]):
        diagnosis.append("Rotational instability or slip detected during in-place turns and final goal alignment. Often caused by abrupt angular acceleration (>3.0 rad/s²) overpowering floor traction, missing rotation shim alignment causing wide arc swinging, or tight yaw goal tolerance causing end-pose hunting.")
        recommendations.append("Smooth angular acceleration limit to 2.0 rad/s² to eliminate wheel slip and motor shudder during in-place spins.")
        recommendations.append("Tune Rotation Shim Controller: angular_dist_threshold = 0.785 rad (45°), rotate_to_heading_angular_vel = 1.5 rad/s.")
        recommendations.append("Expand yaw goal tolerance window to 0.12 rad (~7°) to prevent continuous hunting around target orientation.")
        recommendations.append("Ensure EKF odom0_config lateral velocity (v_y = false) is locked to prevent false lateral accumulation during spins.")
        nav2_patch.update({
            "max_vel_theta": 1.8,
            "max_accel_theta": 2.0,
            "rotate_to_heading_angular_vel": 1.5,
            "angular_dist_threshold": 0.785,
            "yaw_goal_tolerance": 0.12
        })
        if target_base != "mecanum":
            ekf_patch["fuse_vy"] = False
        ekf_patch["two_d_mode"] = True
        ekf_patch["frequency"] = 50.0

    # 5. UPSTREAM ISSUE #113 & #67: CONTINUOUS MAP SPINNING DURING SLAM
    if any(k in p for k in ["map spinning", "map continuously rotating", "map rotating", "slam spinning", "slam circle", "flickering pose", "noisy imu", "spinning map"]):
        diagnosis.append("[Upstream #113/#67] Continuous map spinning and pose flickering during SLAM detected. Uncalibrated or vibrating IMU (e.g. MPU6050 with spinning LiDAR vibration) causes EKF to integrate runaway orientation yaw.")
        recommendations.append("Disable IMU orientation yaw in EKF (fuse_imu_yaw = false) and only fuse angular velocity (vyaw).")
        recommendations.append("Enforce 2D planar mode (two_d_mode = true) and 50Hz update rate in EKF to stop false vertical/roll tilt.")
        recommendations.append("In SLAM Toolbox, decrease minimum_travel_heading to 0.25 rad for dense scan registration.")
        ekf_patch["fuse_imu_yaw"] = False
        ekf_patch["two_d_mode"] = True
        ekf_patch["frequency"] = 50.0
        slam_patch["minimum_travel_heading"] = 0.25

    # 6. UPSTREAM ISSUE #37: OBSTACLES CANNOT CLEAR FROM LOCAL COSTMAP
    if any(k in p for k in ["obstacle clear", "obstacle cant clear", "cant clear", "ghost obstacle", "persistent obstacle", "moved out", "obstacle stuck"]):
        diagnosis.append("[Upstream #37] Obstacles fail to clear from local costmap after moving out. Caused by raytrace clearing range (raytrace_range) being equal to or smaller than obstacle insertion range (obstacle_max_range).")
        recommendations.append("Increase raytrace_range to 3.5m, strictly exceeding obstacle_max_range (3.0m) to clear free space along raycasts.")
        recommendations.append("Slightly reduce costmap inflation_radius to 0.55m to prevent lingering inflation halos.")
        nav2_patch.update({
            "raytrace_range": 3.5,
            "obstacle_max_range": 3.0,
            "inflation_radius": 0.55
        })

    # 7. UPSTREAM ISSUE #76: HEAVY / LARGE ROBOT JERK & VIOLENT VIBRATION
    if any(k in p for k in ["heavy", "large robot", "jerks", "jerk", "vibrat", "50 kg", "fierce vibration", "violent", "shudder"]):
        diagnosis.append("[Upstream #76] Heavy robot (>50kg) jerking and severe vibration under default velocity smoother acceleration ramps (3.0 m/s²) which excite mechanical backlash and motor driver current limits.")
        recommendations.append("Smooth linear acceleration to 1.0 m/s² and angular acceleration to 1.2 rad/s².")
        recommendations.append("Set controlled deceleration to -1.5 m/s² to prevent sudden braking spikes and tipping.")
        recommendations.append("Reduce cruising velocity to 0.35 m/s for safe, stable transit.")
        nav2_patch.update({
            "max_vel_x": 0.35,
            "max_accel_x": 1.0,
            "max_decel_x": 1.5,
            "max_vel_theta": 1.2,
            "max_accel_theta": 1.2,
            "max_decel_theta": 1.8
        })

    # 8. UPSTREAM ISSUE #12 & #15: CONTINUOUS IMMEDIATE RECOVERY LOOPS
    if any(k in p for k in ["recovery loop", "recoveries server", "always runs recovery", "spin recovery", "recovery backup not working", "aborting handle"]):
        diagnosis.append("[Upstream #12/#15] Immediate recovery server spin loops occur when navigation goal or initial pose is inside obstacle inflation cost, or controller patience aborts immediately due to strict tolerance window.")
        recommendations.append("Expand goal tolerance window (xy_goal_tolerance = 0.08m, yaw_goal_tolerance = 0.12 rad).")
        recommendations.append("Steepen inflation decay (inflation_radius = 0.52m, cost_scaling_factor = 5.5) to clear space around obstacles.")
        recommendations.append("Increase controller progress allowance to 15.0s.")
        nav2_patch.update({
            "xy_goal_tolerance": 0.08,
            "yaw_goal_tolerance": 0.12,
            "inflation_radius": 0.52,
            "cost_scaling_factor": 5.5,
            "movement_time_allowance": 15.0
        })

    if not diagnosis:
        diagnosis.append("Custom robotic parameter optimization for smooth mobile robot navigation.")
        recommendations.append("Applied balanced velocity limits (0.5 m/s) and 50 Hz EKF state estimation.")

    return {
        "target_base": target_base,
        "diagnosis": " ".join(diagnosis),
        "recommendations": recommendations,
        "nav2_patch": nav2_patch,
        "ekf_patch": ekf_patch,
        "slam_patch": slam_patch
    }


# Turn a sparse patch/preset/spec dict into patch_nav2_text / patch_ekf_text
# kwargs (dropping absent keys) so a targeted fix leaves untouched keys alone.
# Canonical key lists live in patcher.py; fall back to no-op if patcher failed
# to import so the server still starts.
if patcher:
    _nav2_kwargs = patcher.nav2_kwargs
    _ekf_kwargs = patcher.ekf_kwargs
else:  # pragma: no cover - only when patcher.py is unavailable
    def _nav2_kwargs(src):
        return {}

    def _ekf_kwargs(src, base=None):
        return {}



def _find_default_workspace():
    # Look 5 directories up from web/server.py: tools/console -> tools -> linorobot2 -> src -> workspace
    cand = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", ".."))
    if os.path.isdir(os.path.join(cand, "src", "linorobot2")):
        return cand
    return os.path.expanduser("~/linorobot2_ws")


def build_base_install_cmd(ws=None, distro=None):
    """Command string to clone, resolve dependencies, and build the base linorobot2 workspace."""
    ws = os.path.abspath(os.path.expanduser(ws or _find_default_workspace()))
    distro = distro or detect_ros_distro()
    steps = [
        f"mkdir -p {ws}/src",
        f"cd {ws}/src",
        f"([ -d linorobot2 ] || git clone -b {distro} https://github.com/linorobot/linorobot2.git linorobot2 || git clone -b main https://github.com/linorobot/linorobot2.git linorobot2 || git clone https://github.com/linorobot/linorobot2.git linorobot2)",
        "touch linorobot2/linorobot2_gazebo/COLCON_IGNORE 2>/dev/null || true",
        f"cd {ws}",
        "rosdep update 2>/dev/null || true",
        "rosdep install --from-paths src --ignore-src -y --skip-keys microxrcedds_agent 2>/dev/null || true",
        "colcon build --symlink-install",
    ]
    return " && ".join(steps)

RUNTIME_DEP_PACKAGES = [
    "python3-colcon-common-extensions",
    "python3-rosdep",
    "python3-vcstool",
    "git",
    # ros-<distro>-ros-base is a runtime metapackage: it does not pull a C++
    # toolchain. Without these, the first thing the 1-Click chain does after
    # installing ROS 2 -- colcon build of micro_ros_agent -- dies with
    # "No CMAKE_CXX_COMPILER could be found", which reads like a broken
    # install rather than a missing compiler.
    "build-essential",
    "cmake",
]


def toolchain_ready():
    """colcon and a C++ compiler -- ros-<distro>-ros-base ships neither."""
    return all(shutil.which(t) for t in ("colcon", "c++", "cmake"))


def ros2_installed(distro=None):
    """True only when ROS 2 *and* the tools Console builds with are present.

    Reporting True on the strength of /opt/ros/<distro>/setup.bash alone let
    the 1-Click chain skip its install step and then fail in colcon instead.
    """
    distro = distro or detect_ros_distro()
    return os.path.isfile(f"/opt/ros/{distro}/setup.bash") and toolchain_ready()


def build_ros2_install_cmd(distro=None, packages="ros-base"):
    """Install ROS 2 <distro> plus the build tools Console needs.

    The 1-Click flows promise "run SLAM and it installs what it needs". Without
    this they ran `colcon build` on a box that had neither ROS 2 nor colcon and
    failed with "colcon: command not found", which tells the user nothing about
    what is actually missing.

    Uses the ros2-apt-source package rather than hand-writing
    /etc/apt/sources.list.d and a keyring: that is what upstream now documents,
    and it is the same mechanism the firmware repo's CI uses.
    """
    distro = distro or detect_ros_distro()
    deps = " ".join(RUNTIME_DEP_PACKAGES)
    return " && ".join([
        "export DEBIAN_FRONTEND=noninteractive",
        "sudo apt-get update",
        "sudo apt-get install -y curl ca-certificates gnupg lsb-release software-properties-common",
        "sudo add-apt-repository -y universe",
        'ROS_APT_SOURCE_VERSION=$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest '
        '| grep -F \'"tag_name"\' | awk -F\'"\' \'{print $4}\')',
        'UBUNTU_CODENAME_VAL=$(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME})',
        'curl -fsSL -o /tmp/ros2-apt-source.deb '
        '"https://github.com/ros-infrastructure/ros-apt-source/releases/download/'
        '${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME_VAL}_all.deb"',
        "sudo apt-get install -y /tmp/ros2-apt-source.deb",
        "sudo apt-get update",
        # A ROS 2 distro is not in the main `ros2` repo for every Ubuntu it
        # supports. While a new Ubuntu is being brought up, its builds land in
        # `ros2-testing` first: on 26.04 (resolute) the ros2 repo carries 2563
        # ros-lyrical-* packages and *zero* ros-rolling-*, while ros2-testing
        # carries 2205 rolling ones. Without this the 1-Click chain died on
        # "E: Unable to locate package ros-rolling-ros-base", which reads like
        # a broken box rather than a repo that simply does not have it yet.
        # Only switch on when the package is genuinely absent, and say so.
        f'if ! apt-cache policy ros-{distro}-{packages} 2>/dev/null | grep -qE "Candidate: [0-9]"; then '
        f'echo "[console] ros-{distro}-{packages} is not published in the ros2 repo for this Ubuntu -- '
        'enabling ros2-testing, where new-Ubuntu builds land first."; '
        # Derived from the file ros2-apt-source just installed, so the signing
        # key and suite come along without hand-writing either.
        'sudo sh -c "sed \'s|/ros2/ubuntu|/ros2-testing/ubuntu|\' '
        '/etc/apt/sources.list.d/ros2.sources > /etc/apt/sources.list.d/ros2-testing.sources"; '
        'sudo apt-get update; fi',
        f"sudo apt-get install -y ros-{distro}-{packages} {deps}",
        # Parenthesised: a bare `|| true` at this position also swallows a
        # failure of the apt install above it, and the whole command then
        # exits 0 with no ROS 2 on disk.
        "(sudo rosdep init 2>/dev/null || true)",
        "(rosdep update 2>/dev/null || true)",
        f"echo '[console] ROS 2 {distro} installed at /opt/ros/{distro}'",
    ])


DEFAULT_CONFIG = {
    "workspace_path": os.path.expanduser("~/linorobot2_ws"),
    # Blank, not "jazzy": a shipped default here wins over detect_ros_distro()
    # and would pin a fresh install on Ubuntu 26.04 to a distro that has no
    # native packages for it. Leave it empty and let detection choose (env
    # ROS_DISTRO, then an installed /opt/ros/<d>, then the OS release), so
    # 24.04 comes up Jazzy and 26.04 comes up Lyrical. The user can still pick
    # one explicitly; that choice is what gets written back here.
    "ros_distro": "",
    "install_mode": "native",       # "native" | "docker" | "podman"
    # Blank means "decide from install_mode + what is actually installed".
    # Shipping "docker" put a fresh native install into a container workflow
    # before the user had chosen anything.
    "agent_engine": "",             # "" | "docker" | "podman_systemd" | "podman" | "native"
    "container_registry": "auto",   # "auto" | "cluster" | "dockerhub" | custom
    "custom_registry": "",
    "auto_bringup": True,
    "agent_transport": "serial",   # "serial" | "udp4"
    "agent_device": "/dev/ttyACM0",
    "agent_port": "8888",
    "agent_baud": "921600",
    "base_type": "2wd",
    "laser_sensor": "",
    # Which model inside that driver family (ld06 / ld19 / stl27l ...). The
    # family alone is ambiguous and they differ in ways that matter -- an LD19
    # has 456 bins where an LD06 has 360 -- so record the exact model rather
    # than defaulting to whichever happens to be first in the registry.
    "laser_model": "",
    "depth_sensor": "",
    "robot_name": "linorobot2",
    "ros_domain_id": 0,
    "madgwick": True,
    "laser_serial_port": "",
    "laser_baud": "",
    # How the scan's bytes reach this computer -- not what produces them. A real
    # LD19 on a USB cable and the firmware's USE_FAKE_LD19 emulator are both
    # "serial" here, and both are "udp_bridge" when the MCU relays them over
    # WiFi (USE_LIDAR_UDP). Console never needs to know whether a scan is
    # simulated; it needs to know which device or socket to read.
    "laser_transport": "",          # "" | "serial" | "udp_bridge" | "udp_server" | "udp_client"
    "laser_udp_port": "",           # the firmware's LIDAR_PORT, for udp_bridge
    "depth_serial_port": "",
}

# ============================================================================
# Sensor registry -- the ONE source of truth for laser + depth sensors.
#
# Everything the frontend needs to build a dropdown, an install command, a
# bringup env var, a docker/.env line or a direct `ros2 run` invocation lives
# here. The browser fetches this whole structure once from /api/sensors and no
# longer keeps its own parallel copies. Install/udev command lists are ported
# from linorobot2's install.bash (kept as plain data, not sourced) and run as
# one `&&`-joined string via /api/sensor_install_cmd.
#
# Per-entry keys:
#   label          human label for the driver package
#   install/udev   command lists ({ws} -> workspace path); udev None = no rule
#   serial         True if it's a serial device with a user-selectable port
#   symlink        persistent /dev/<name> udev symlink this driver's rules make
#   default_baud   baud used when the UI field is left blank
#   docker_key     value for docker/.env LASER_SENSOR=/DEPTH_SENSOR= (None = n/a)
#   models         bringup `sensor:=` / LINOROBOT2_*_SENSOR codes this driver
#                  covers, each: {code, label, [product, bins, baud]}.
#                  product/bins/baud present => driver via `ros2 run
#                  ldlidar_stl_ros2` with a fully configurable port; absent =>
#                  delegated to linorobot2_bringup/launch/lasers.launch.py.
# ============================================================================
LASER_SENSORS = {
    "ydlidar": {
        "label": "YDLIDAR",
        "serial": True,
        "symlink": "/dev/ydlidar",
        "default_baud": "128000",
        "docker_key": "ydlidar",
        "driver_pkg": "ydlidar_ros2_driver",
        "models": [{"code": "ydlidar", "label": "YDLIDAR X4 / G4 / others"}],
        "install": [
            "cd /tmp",
            "rm -rf YDLidar-SDK",
            "git clone https://github.com/YDLIDAR/YDLidar-SDK.git",
            "mkdir -p YDLidar-SDK/build && cd YDLidar-SDK/build",
            "cmake .. && make",
            "sudo make install",
            "cd {ws}",
            "[ -d src/ydlidar_ros2_driver ] || git clone https://github.com/YDLIDAR/ydlidar_ros2_driver src/ydlidar_ros2_driver",
            "chmod 0777 src/ydlidar_ros2_driver/startup/*",
            "colcon build --symlink-install",
        ],
        "udev": [
            'echo \'KERNEL=="ttyUSB*", ATTRS{{idVendor}}=="10c4", ATTRS{{idProduct}}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"\' | sudo tee /etc/udev/rules.d/ydlidar.rules',
            'echo \'KERNEL=="ttyACM*", ATTRS{{idVendor}}=="0483", ATTRS{{idProduct}}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"\' | sudo tee /etc/udev/rules.d/ydlidar-V2.rules',
            'echo \'KERNEL=="ttyUSB*", ATTRS{{idVendor}}=="067b", ATTRS{{idProduct}}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"\' | sudo tee /etc/udev/rules.d/ydlidar-2303.rules',
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "xv11": {
        "label": "XV11",
        "serial": True,
        "symlink": None,
        "default_baud": "115200",
        "docker_key": "xv11",
        "driver_pkg": "xv_11_driver",
        "models": [{"code": "xv11", "label": "Neato XV11"}],
        "install": [
            "cd {ws}",
            "[ -d src/xv_11_driver ] || git clone https://github.com/mjstn/xv_11_driver src/xv_11_driver",
            "colcon build",
        ],
        "udev": None,
    },
    "ldlidar": {
        "label": "LDROBOT (LD06 / LD19 / STL27L)",
        "serial": True,
        "symlink": "/dev/ldlidar",
        "default_baud": "230400",
        "docker_key": "ldlidar",
        "driver_pkg": "ldlidar_stl_ros2",
        "models": [
            {"code": "ld06", "label": "LD06", "product": "LDLiDAR_LD06", "bins": 456, "baud": "230400"},
            {"code": "ld19", "label": "LD19", "product": "LDLiDAR_LD19", "bins": 456, "baud": "230400"},
            {"code": "stl27l", "label": "STL27L", "product": "LDLiDAR_STL27L", "bins": 2160, "baud": "921600"},
        ],
        "install": [
            "cd {ws}",
            "[ -d src/ldlidar_stl_ros2 ] || git clone https://github.com/hippo5329/ldlidar_stl_ros2.git src/ldlidar_stl_ros2",
            "colcon build",
        ],
        "udev": [
            "cd /tmp && wget -q https://raw.githubusercontent.com/linorobot/ldlidar/ros2/ldlidar.rules",
            "sudo cp ldlidar.rules /etc/udev/rules.d",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "sllidar": {
        "label": "RPLIDAR (A1/A2/A3/C1/S1/S2/S3)",
        "serial": True,
        "symlink": "/dev/rplidar",
        "default_baud": "115200",
        "docker_key": "rplidar",
        "driver_pkg": "sllidar_ros2",
        "models": [
            {"code": "a1", "label": "RPLIDAR A1"},
            {"code": "a2", "label": "RPLIDAR A2"},
            {"code": "a3", "label": "RPLIDAR A3"},
            {"code": "c1", "label": "RPLIDAR C1"},
            {"code": "s1", "label": "RPLIDAR S1"},
            {"code": "s2", "label": "RPLIDAR S2"},
            {"code": "s3", "label": "RPLIDAR S3"},
        ],
        "install": [
            "cd {ws}",
            "[ -d src/sllidar_ros2 ] || git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2",
            "colcon build",
        ],
        "udev": [
            "sudo cp {ws}/src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
}

DEPTH_SENSORS = {
    "realsense": {
        "label": "Intel RealSense",
        "serial": False,
        "symlink": None,
        "docker_key": "realsense",
        "models": [{"code": "realsense", "label": "RealSense D4xx"}],
        "install": ["sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera"],
        "udev": [
            "cd /tmp && wget -q https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules",
            "sudo cp 99-realsense-libusb.rules /etc/udev/rules.d",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "oakd": {
        "label": "Luxonis OAK-D / Lite / Pro",
        "serial": False,
        "symlink": None,
        "docker_key": None,
        "models": [
            {"code": "oakd", "label": "OAK-D"},
            {"code": "oakdlite", "label": "OAK-D Lite"},
            {"code": "oakdpro", "label": "OAK-D Pro"},
        ],
        "install": ["sudo apt-get install -y ros-$ROS_DISTRO-depthai-ros"],
        "udev": [
            'echo \'SUBSYSTEM=="usb", ATTRS{{idVendor}}=="03e7", MODE="0666"\' | sudo tee /etc/udev/rules.d/80-movidius.rules',
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "astra": {
        "label": "Orbbec Astra",
        "serial": False,
        "symlink": None,
        "docker_key": None,
        "models": [{"code": "astra", "label": "Orbbec Astra"}],
        "install": [
            "sudo apt-get install -y libuvc-dev libopenni2-dev",
            "cd {ws}",
            "[ -d src/ros_astra_camera ] || git clone https://github.com/linorobot/ros_astra_camera src/ros_astra_camera",
            "colcon build",
        ],
        "udev": [
            "sudo cp {ws}/src/ros_astra_camera/56-orbbec-usb.rules /etc/udev/rules.d/",
            "sudo udevadm control --reload-rules && sudo udevadm trigger",
        ],
    },
    "zed": {
        "label": "Stereolabs ZED (SDK-managed)",
        "serial": False,
        "symlink": None,
        "docker_key": "zed",
        "models": [
            {"code": "zed", "label": "ZED"},
            {"code": "zedm", "label": "ZED Mini"},
            {"code": "zed2", "label": "ZED 2"},
            {"code": "zed2i", "label": "ZED 2i"},
        ],
        # ZED needs the proprietary SDK + zed-ros2-wrapper; not scripted here.
        "install": None,
        "udev": None,
    },
}


def _sensor_table(kind):
    return LASER_SENSORS if kind == "laser" else DEPTH_SENSORS


def sensor_registry():
    """The full sensor registry the browser builds every sensor dropdown from.

    Command lists are included so the client no longer keeps its own copy;
    it still displays them, but assembly happens in build_sensor_install_cmd.
    """
    def entries(table):
        out = {}
        for key, e in table.items():
            out[key] = {
                "label": e["label"],
                "serial": e.get("serial", False),
                "symlink": e.get("symlink"),
                "default_baud": e.get("default_baud"),
                "docker_key": e.get("docker_key"),
                "models": e.get("models", []),
                "has_install": bool(e.get("install")),
                "has_udev": bool(e.get("udev")),
            }
        return out
    return {"laser": entries(LASER_SENSORS), "depth": entries(DEPTH_SENSORS)}


def build_sensor_install_cmd(kind, key, skip_udev=False, udev_only=False, ws=None):
    """Join a sensor's install (and/or udev) steps into one `&&` command string."""
    entry = _sensor_table(kind).get(key)
    if not entry:
        return None
    ws = ws or os.path.expanduser("~/linorobot2_ws")
    steps = []
    if not udev_only and entry.get("install"):
        steps += list(entry["install"])
    if (udev_only or not skip_udev) and entry.get("udev"):
        # udev rules are best-effort. Inside a container /etc/udev/rules.d does
        # not exist, so `sudo cp ... /etc/udev/rules.d` fails with "No such file
        # or directory" and took the whole driver install down with it -- after
        # the driver had already built and installed successfully. The device
        # still works there; only the persistent /dev/<name> symlink is missing.
        steps.append("sudo mkdir -p /etc/udev/rules.d 2>/dev/null || true")
        steps += [f"{{ {u} ; }} || echo '[console] udev rule step skipped "
                  f"(no udev in this environment) -- the device still works, "
                  f"only its stable /dev symlink is missing'"
                  for u in entry["udev"]]
    if not steps:
        return None
    return " && ".join(s.replace("{ws}", ws) for s in steps)


_TTY_RE = re.compile(r"^tty(USB|ACM)\d+$")


def _udev_usb_props(dev):
    """idVendor:idProduct + vendor/model/serial strings for a /dev/tty* node."""
    try:
        out = subprocess.run(
            ["udevadm", "info", "--query=property", "--name", dev],
            capture_output=True, text=True, timeout=4,
        ).stdout
    except Exception:
        return {}
    props = {}
    for line in out.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            props[k] = v
    vid = props.get("ID_VENDOR_ID", "")
    pid = props.get("ID_MODEL_ID", "")
    return {
        "usb_id": f"{vid}:{pid}" if vid and pid else "",
        "vendor": props.get("ID_VENDOR_FROM_DATABASE") or props.get("ID_VENDOR", ""),
        "model": props.get("ID_MODEL_FROM_DATABASE") or props.get("ID_MODEL", ""),
        "serial": props.get("ID_SERIAL_SHORT", ""),
        "driver": props.get("ID_USB_DRIVER", ""),
    }


def _by_path_for(tty_dev):
    """The /dev/serial/by-path/<x> symlink that resolves to tty_dev, if any."""
    d = "/dev/serial/by-path"
    if not os.path.isdir(d):
        return None
    for name in os.listdir(d):
        link = os.path.join(d, name)
        try:
            if os.path.realpath(link) == os.path.realpath(tty_dev):
                return link
        except OSError:
            continue
    return None


def to_by_path(dev):
    """Normalize a serial device to its stable /dev/serial/by-path/ form.

    A by-path or by-id path is returned unchanged. A raw /dev/ttyUSBn is
    resolved to its by-path symlink when one exists, else returned as given.
    """
    if not dev:
        return dev
    if dev.startswith(("/dev/serial/by-path/", "/dev/serial/by-id/")):
        return dev
    return _by_path_for(dev) or dev


def list_dir(path, only="any", exts=""):
    """Directory listing for the browser's path pickers.

    only: "dir" (folders only), "file", or "any". exts: comma-separated
    extensions filter for files (e.g. "yaml,yml"). Blank/'.'/'~' -> $HOME.
    Returns {path, parent, entries:[{name, path, is_dir}]} (sorted, dirs first,
    hidden entries dropped). Never raises -- an unreadable path returns an
    error field and falls back to $HOME.
    """
    home = os.path.expanduser("~")
    raw = (path or "").strip()
    if raw in ("", ".", "~"):
        raw = home
    target = os.path.abspath(os.path.expanduser(raw))
    if not os.path.isdir(target):
        target = os.path.dirname(target) if target else home
        if not os.path.isdir(target):
            target = home
    ext_set = {e.strip().lstrip(".").lower() for e in exts.split(",") if e.strip()}
    entries, err = [], None
    try:
        for name in sorted(os.listdir(target), key=str.lower):
            if name.startswith("."):
                continue
            full = os.path.join(target, name)
            is_dir = os.path.isdir(full)
            if only == "dir" and not is_dir:
                continue
            if only == "file" and is_dir:
                pass  # still show dirs so the user can navigate
            if not is_dir and ext_set and name.rsplit(".", 1)[-1].lower() not in ext_set:
                continue
            entries.append({"name": name, "path": full, "is_dir": is_dir})
    except OSError as e:
        err = str(e)
    entries.sort(key=lambda x: (not x["is_dir"], x["name"].lower()))
    parent = os.path.dirname(target)
    return {
        "path": target,
        "parent": parent if parent != target else None,
        "home": home,
        "entries": entries,
        **({"error": err} if err else {}),
    }


def list_serial_ports():
    """Serial devices present now, each with its stable path + USB identity."""
    seen = {}  # realpath -> record

    def add(path, kind):
        try:
            real = os.path.realpath(path)
        except OSError:
            return
        if not _TTY_RE.match(os.path.basename(real)):
            return
        rec = seen.setdefault(real, {"tty": real, "by_path": None, "by_id": None})
        if kind == "by-path":
            rec["by_path"] = path
        elif kind == "by-id":
            rec["by_id"] = path

    for base, kind in (("/dev/serial/by-path", "by-path"), ("/dev/serial/by-id", "by-id")):
        if os.path.isdir(base):
            for name in sorted(os.listdir(base)):
                add(os.path.join(base, name), kind)
    for name in sorted(os.listdir("/dev")):
        if _TTY_RE.match(name):
            add(os.path.join("/dev", name), "tty")

    ports = []
    for real, rec in sorted(seen.items()):
        props = _udev_usb_props(real)
        ports.append({
            # preferred = the most stable path available for this device
            "preferred": rec["by_path"] or rec["by_id"] or rec["tty"],
            "by_path": rec["by_path"],
            "by_id": rec["by_id"],
            "tty": rec["tty"],
            **props,
        })
    return ports


# ============================================================================
# Params export / merge / promote
# ============================================================================
PKG_CONFIG_DIR = os.path.join(LINOROBOT2_ROOT, "linorobot2_navigation", "config")
_EKF_BASES = ("2wd", "4wd", "mecanum")


def _params_paths(kind, distro=None, base=None):
    """(active_path, template_path, package_path) for a nav2/ekf/slam config."""
    distro = distro or detect_ros_distro()
    if kind == "nav2":
        return (
            get_nav2_config_path(distro),
            os.path.join(CONFIG_DIR, f"nav2_{distro}.yaml"),
            os.path.join(PKG_CONFIG_DIR, f"navigation_{distro}.yaml"),
        )
    if kind == "ekf":
        b = (base or "").lower()
        suffix = f"_{b}" if b in _EKF_BASES else ""
        return (
            get_ekf_config_path(),
            os.path.join(CONFIG_DIR, f"ekf{suffix}.yaml"),
            os.path.join(PKG_CONFIG_DIR, f"ekf{suffix}.yaml"),
        )
    if kind == "slam":
        return (
            get_slam_config_path(),
            os.path.join(CONFIG_DIR, "slam.yaml"),
            os.path.join(PKG_CONFIG_DIR, "slam.yaml"),
        )
    raise ValueError(f"unknown params kind: {kind}")


def _read_params(kind, distro=None, base=None):
    """Current effective text for a config: active file if present, else template."""
    if kind == "nav2":
        return get_nav2_config(distro)
    if kind == "ekf":
        return get_ekf_config(base)
    if kind == "slam":
        return get_slam_config()
    raise ValueError(kind)


def _resolve_target_path(kind, target, distro=None, base=None):
    active, template, package = _params_paths(kind, distro, base)
    return {"active": active, "template": template, "package": package}.get(target)


EXPORT_LAUNCH_TEMPLATE = '''#!/usr/bin/env python3
"""Standalone Nav2 launcher exported by linorobot2_console on {ts}.

Runs the exported params through the fork's own navigation.launch.py without
needing the console. Adjust the paths / args below or override on the CLI:

    ros2 launch nav2.launch.py map:=/path/to/map.yaml slam:=false
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

BUNDLE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # this file is <bundle>/launch/
NAV2_PARAMS = os.path.join(BUNDLE, "config", "navigation_{distro}.yaml")
SLAM_PARAMS = os.path.join(BUNDLE, "config", "slam.yaml")
DEPTH_COSTMAP_DEFAULT = "{depth_costmap}"   # auto | true | false


def _launch(context, *a, **k):
    nav = os.path.join(
        FindPackageShare("linorobot2_navigation").find("linorobot2_navigation"),
        "launch", "navigation.launch.py",
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav),
        launch_arguments={{
            "slam": LaunchConfiguration("slam"),
            "distro": "{distro}",
            "base": "{base}",
            "params_file": NAV2_PARAMS,
            "slam_params_file": SLAM_PARAMS,
            "map": LaunchConfiguration("map"),
            "sim": LaunchConfiguration("sim"),
            "rviz": LaunchConfiguration("rviz"),
        }}.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_launch),
    ])
'''


def export_params_bundle(dest_dir, distros=None, base="2wd", depth_costmap="auto"):
    """Write the active nav2/ekf/slam configs + a standalone launcher to dest_dir."""
    import time
    dest_dir = os.path.abspath(os.path.expanduser(dest_dir))
    cfg_out = os.path.join(dest_dir, "config")
    launch_out = os.path.join(dest_dir, "launch")
    os.makedirs(cfg_out, exist_ok=True)
    os.makedirs(launch_out, exist_ok=True)
    distros = distros or [detect_ros_distro()]
    written = []

    def _w(path, text):
        with open(path, "w") as f:
            f.write(text if text.endswith("\n") else text + "\n")
        written.append({"path": path, "bytes": len(text)})

    for d in distros:
        if d not in SUPPORTED_DISTROS:
            continue
        _w(os.path.join(cfg_out, f"navigation_{d}.yaml"), _read_params("nav2", d))
    _w(os.path.join(cfg_out, "ekf.yaml"), _read_params("ekf", base=None))
    for b in _EKF_BASES:
        _, tpl, _pkg = _params_paths("ekf", base=b)
        src = get_ekf_config_path()
        text = None
        if os.path.exists(src):
            with open(src) as f:
                text = f.read()
        elif os.path.exists(tpl):
            with open(tpl) as f:
                text = f.read()
        if text is not None:
            _w(os.path.join(cfg_out, f"ekf_{b}.yaml"), text)
    _w(os.path.join(cfg_out, "slam.yaml"), _read_params("slam"))
    _w(os.path.join(launch_out, "nav2.launch.py"),
       EXPORT_LAUNCH_TEMPLATE.format(
           ts=time.strftime("%Y-%m-%d %H:%M:%S%z"),
           distro=(distros[0] if distros else detect_ros_distro()),
           base=base, depth_costmap=depth_costmap))
    _w(os.path.join(dest_dir, "README.md"),
       "# Exported linorobot2 Nav2 params\n\n"
       f"Generated by linorobot2_console, {time.strftime('%Y-%m-%d %H:%M:%S%z')}.\n\n"
       "```\nros2 launch launch/nav2.launch.py map:=/path/to/map.yaml slam:=false\n```\n")
    return {"dest_dir": dest_dir, "files": written, "count": len(written)}


def _coerce_console_types(sect):
    """Match each value's type to its DEFAULT_CONFIG counterpart (the YAML
    parser turns "8888" into an int; DEFAULT_CONFIG keeps agent_port a str)."""
    out = {}
    for k, v in sect.items():
        if k not in DEFAULT_CONFIG:
            continue
        ref = DEFAULT_CONFIG[k]
        if isinstance(ref, bool):
            out[k] = v if isinstance(v, bool) else str(v).strip().lower() in ("true", "1", "yes")
        elif isinstance(ref, str):
            out[k] = str(v)
        elif isinstance(ref, int):
            try:
                out[k] = int(v)
            except (TypeError, ValueError):
                out[k] = ref
        else:
            out[k] = v
    return out


def _read_console_section_from_yaml(path):
    """Flat dict from the `console:` section of a robot config YAML, or None."""
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return None
    sect = parse_unified_yaml(text).get("console") or {}
    return _coerce_console_types(sect) or None


def load_config(robot_name=None):
    """Console workflow settings for the given (or active) robot.

    Source of truth: the `console:` section of
    <linorobot2>/config/<robot>_config.yaml. Falls back, in order, to the
    legacy per-web console_config.json and then ~/.config/linorobot2/
    robot_config.yaml so an un-migrated install still works.
    """
    cfg = dict(DEFAULT_CONFIG)
    yaml_path = get_robot_config_path(robot_name)
    from_yaml = _read_console_section_from_yaml(yaml_path)
    if from_yaml:
        cfg.update(from_yaml)
    elif os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH) as f:
                cfg.update({k: v for k, v in json.load(f).items() if k in DEFAULT_CONFIG})
        except Exception:
            pass
    elif os.path.exists(LEGACY_ROBOT_CONFIG_YAML_PATH):
        legacy = _read_console_section_from_yaml(LEGACY_ROBOT_CONFIG_YAML_PATH)
        if legacy:
            cfg.update(legacy)
    if cfg.get("workspace_path"):
        cfg["workspace_path"] = os.path.expanduser(cfg["workspace_path"])
        # Only fallback if workspace_path is default ~/linorobot2_ws and hosting repo workspace is built
        if cfg["workspace_path"] == os.path.expanduser("~/linorobot2_ws"):
            def_ws = _find_default_workspace()
            if def_ws != cfg["workspace_path"] and os.path.exists(os.path.join(def_ws, "install", "setup.bash")):
                cfg["workspace_path"] = def_ws
    return cfg


def save_config(cfg, robot_name=None):
    """Persist console workflow settings into the active robot's YAML
    `console:` section (creating the file if needed)."""
    yaml_path = get_robot_config_path(robot_name)
    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
    try:
        with open(yaml_path) as f:
            text = f.read()
    except OSError:
        text = ""
    new_text = splice_yaml_section(text, "console", render_console_section(cfg))
    if not new_text.endswith("\n"):
        new_text += "\n"
    with open(yaml_path, "w") as f:
        f.write(new_text)


def migrate_legacy_config():
    """One-time move to repo-based config/<robot>_config.yaml.

    Seeds config/<robot>_config.yaml with a `console:` section built from the
    legacy web/console_config.json (the nav2:/ekf:/slam: sections are
    regenerated on demand from the console's own working copies). Idempotent:
    does nothing once any *_config.yaml exists in config/.
    """
    os.makedirs(ROBOT_CONFIGS_DIR, exist_ok=True)
    if any(f.endswith("_config.yaml") for f in os.listdir(ROBOT_CONFIGS_DIR)):
        return None
    if not os.path.exists(CONFIG_PATH):
        return None

    try:
        with open(CONFIG_PATH) as f:
            legacy_json = json.load(f)
    except Exception:
        legacy_json = {}

    robot_name = legacy_json.get("robot_name") or DEFAULT_ROBOT_NAME
    if not _robot_name_ok(robot_name):
        robot_name = DEFAULT_ROBOT_NAME
    dest = get_robot_config_path(robot_name)

    cfg = dict(DEFAULT_CONFIG)
    cfg.update({k: v for k, v in legacy_json.items() if k in DEFAULT_CONFIG})
    cfg["robot_name"] = robot_name
    text = render_console_section(cfg) + "\n"
    with open(dest, "w") as f:
        f.write(text)
    set_active_robot_name(robot_name)
    return {"robot": robot_name, "path": dest}


def get_host_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


class ProcessRunner:
    """One tracked subprocess slot: start (SSE-streamed), stop, status.
    Two independent instances are used -- `main` (single-shot install/build/
    launch commands) and `agent` (the long-lived micro-ROS agent) -- so the
    agent can keep running while a Bringup/Teleop/SLAM command uses `main`.
    Maintains a rolling ring buffer of recent output lines and supports
    broadcasting to multiple subscribers.
    """

    def __init__(self, name, max_history=1000):
        self.name = name
        self.process = None
        self.lock = threading.Lock()
        self.max_history = max_history
        self.history = collections.deque(maxlen=max_history)
        self.subscribers = []

    def get_history(self):
        with self.lock:
            return list(self.history)

    def subscribe(self, q):
        with self.lock:
            self.subscribers.append(q)

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def _broadcast(self, event_type, payload):
        with self.lock:
            if event_type == "output" and "line" in payload:
                self.history.append(payload["line"])
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait((event_type, payload))
            except Exception:
                pass

    def is_busy(self):
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def start_streaming(self, command, cwd, send_event):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return False
            self.history.clear()
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            self.process = subprocess.Popen(
                ["bash", "-lc", command],
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
                preexec_fn=os.setsid,
            )
        proc = self.process
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                stripped = line.rstrip("\n")
                send_event("output", {"line": stripped})
                self._broadcast("output", {"line": stripped})
        finally:
            proc.wait()
            exit_code = proc.returncode
            with self.lock:
                if self.process is proc:
                    self.process = None
            send_event("done", {"exit_code": exit_code})
            self._broadcast("done", {"exit_code": exit_code})
        return True

    def kill(self):
        with self.lock:
            proc = self.process
            if proc is None or proc.poll() is not None:
                return False
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.process = None
            return True


class GamepadRunner:
    """The virtual gamepad's cmd_vel publisher.

    Unlike the ProcessRunner slots this one is never streamed to the terminal
    and never single-shot: it is a long-lived node that Console writes velocity
    lines into as the on-screen stick moves. It keeps its own slot so driving
    the robot does not tie up `main` (which Bringup/SLAM/Nav2 commands use).
    """

    def __init__(self):
        self.process = None
        self.lock = threading.Lock()
        self.target = (0.0, 0.0, 0.0)

    def is_running(self):
        with self.lock:
            return self.process is not None and self.process.poll() is None

    def start(self, topic="/cmd_vel"):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                return True
            script = os.path.join(REPO_ROOT, "gamepad_publisher.py")
            if not os.path.isfile(script):
                return False
            cmd = _ros_env_prefix() + "exec python3 %s --topic %s" % (
                shlex.quote(script), shlex.quote(topic))
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            self.process = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
                env=env,
                preexec_fn=os.setsid,
            )
            return True

    def send(self, linear_x, linear_y, angular_z):
        with self.lock:
            proc = self.process
            if proc is None or proc.poll() is not None:
                return False
            try:
                proc.stdin.write("%f %f %f\n" % (linear_x, linear_y, angular_z))
                proc.stdin.flush()
                self.target = (linear_x, linear_y, angular_z)
                return True
            except (BrokenPipeError, ValueError):
                # the node died underneath us; report it so the UI can restart
                self.process = None
                return False

    def kill(self):
        with self.lock:
            proc = self.process
            self.process = None
        if proc is None or proc.poll() is not None:
            return False
        try:
            # closing stdin makes the node publish a zero twist before exiting
            try:
                proc.stdin.close()
            except Exception:
                pass
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        return True


main_runner = ProcessRunner("main")
# The laser driver is long-running and has to stay up *while* SLAM or Nav2 run,
# so it cannot share the "main" slot with them: whichever started second was
# refused with a 409 the user only saw as nothing happening. Bringup does not
# launch the LiDAR either, so this button is the only way to get /scan.
laser_runner = ProcessRunner("laser")
# One-shot helpers (saving a map, and the like) that must run *while* the
# long-lived slots are busy -- a map save needs SLAM still publishing /map.
tool_runner = ProcessRunner("tool")
agent_runner = ProcessRunner("agent")
bringup_runner = ProcessRunner("bringup")
gamepad_runner = GamepadRunner()

SLOT_RUNNERS = {
    "main": main_runner,
    "agent": agent_runner,
    "bringup": bringup_runner,
    "laser": laser_runner,
    "tool": tool_runner,
}


def detect_ros_distro():
    """Detect or load configured ROS 2 distribution, with automatic heuristics."""
    try:
        cfg = load_config()
        if cfg.get("ros_distro") and cfg["ros_distro"] in SUPPORTED_DISTROS:
            return cfg["ros_distro"]
    except Exception:
        pass

    env_distro = os.environ.get("ROS_DISTRO", "").strip().lower()
    if env_distro in SUPPORTED_DISTROS:
        return env_distro

    for d in SUPPORTED_DISTROS:
        if os.path.isdir(f"/opt/ros/{d}"):
            return d

    try:
        with open("/etc/os-release") as f:
            c = f.read().lower()
            if "noble" in c or "24.04" in c:
                return "jazzy"
            if "resolute" in c or "26.04" in c:
                return "lyrical"
    except Exception:
        pass

    return "jazzy"


def workspace_built(ws):
    return os.path.exists(os.path.join(ws, "install", "setup.bash"))


def agent_externally_alive():
    """An agent started outside Console (e.g. by config-engine, or a plain
    terminal) -- detected via process list, not by us tracking it."""
    try:
        out = subprocess.run(
            ["pgrep", "-f", "micro_ros_agent"], capture_output=True, text=True
        )
        return out.returncode == 0
    except Exception:
        return False


# Console launches its *own* launcher, tools/console/launch_bringup.py, so a
# pattern looking only for the upstream names never matched it: a bringup left
# over from a previous Console process was invisible, and starting again simply
# added a second one. Nothing errors -- ekf_node, madgwick and
# robot_state_publisher all happily run twice -- but /odom then carries two
# independent pose estimates, and a subscriber sees them interleaved as the
# robot teleporting metres between consecutive samples. Measured with three
# stacked instances: an 8.72 m step in 19 ms, and a SLAM map smeared to four
# times the size of the room.
BRINGUP_PROC_PATTERN = "linorobot2_bringup|bringup.launch.py|launch_bringup.py"

# Every long-lived slot can be orphaned, not just bringup, and each one hurts in
# its own way when it is: duplicate slam_toolbox nodes publish independent maps
# to /map, duplicate laser drivers fight over the same serial port. Measured
# with three SLAM instances stacked up, the map of a 10 x 6 m room came out as a
# 44 x 41 m collage of three disagreeing maps.
SLOT_PROC_PATTERNS = {
    "bringup": BRINGUP_PROC_PATTERN,
    "main": "slam\\.launch\\.py|async_slam_toolbox_node|sync_slam_toolbox_node|"
            "navigation\\.launch\\.py|launch_nav2\\.py|nav2_bringup",
    "laser": "ldlidar_stl_ros2_node|lasers\\.launch\\.py",
}


def slot_orphan_pids(slot):
    """PIDs matching a slot's signature that this Console does not own."""
    pattern = SLOT_PROC_PATTERNS.get(slot)
    if not pattern:
        return []
    try:
        out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
        if out.returncode != 0:
            return []
        mine = set()
        for runner in SLOT_RUNNERS.values():
            proc = runner.process
            if proc is not None and proc.poll() is None:
                # the whole group belongs to a live slot, not just its leader
                mine.add(proc.pid)
                try:
                    mine.add(os.getpgid(proc.pid))
                except Exception:
                    pass
        keep = set()
        for p in out.stdout.split():
            if not p.isdigit():
                continue
            pid = int(p)
            try:
                if os.getpgid(pid) in mine:
                    continue
            except Exception:
                pass
            if pid not in mine:
                keep.add(pid)
        return sorted(keep)
    except Exception:
        return []


def bringup_orphan_pids():
    return slot_orphan_pids("bringup")


def reap_stale_slot(slot):
    """Kill process groups for a slot left behind by a previous Console instance.

    Console restarts (or crashes) abandon whatever it had launched: the child
    keeps running, but the process handle that could stop it is gone. Without
    this, every restart-then-start stacks another copy.
    """
    killed = []
    for pid in slot_orphan_pids(slot):
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(pid), sig)
                killed.append(pid)
                break
            except ProcessLookupError:
                break
            except Exception:
                continue
    if killed:
        time.sleep(1.0)  # let the launch tear its children down
    return killed


def reap_stale_bringup():
    return reap_stale_slot("bringup")


def bringup_externally_alive():
    """Bringup started outside Console (e.g. via terminal or docker) -- detected via process list."""
    try:
        out = subprocess.run(
            ["pgrep", "-f", BRINGUP_PROC_PATTERN], capture_output=True, text=True
        )
        if out.returncode == 0:
            return True
        for engine in ["docker", "podman"]:
            d_out = subprocess.run(
                [engine, "ps", "-q", "--filter", "name=bringup"],
                capture_output=True, text=True, timeout=2
            )
            if d_out.returncode == 0 and d_out.stdout.strip():
                return True
    except Exception:
        pass
    return False


# ---------------------------------------------------------------------------
# Bringup health: is the robot actually PUBLISHING, or is it just a live process?
#
# A running bringup process proves nothing -- the microcontroller may be
# unplugged, the agent may not have a session, the LiDAR may be on the wrong
# port. This probes the ROS graph itself: rate on /odom (raw + EKF-filtered),
# /imu/data and /scan, plus the map->odom->base_link->laser TF chain.
# ---------------------------------------------------------------------------
BRINGUP_HEALTH_TOPICS = [
    # (key,        topic,             what it proves,                     min_hz)
    ("odom_raw",   "/odom/unfiltered", "microcontroller wheel odometry",   5.0),
    ("odom",       "/odom",            "EKF-fused odometry",               5.0),
    ("imu",        "/imu/data",        "IMU / attitude filter",            5.0),
    ("scan",       "/scan",            "LiDAR scans",                      1.0),
]

BRINGUP_TF_CHAIN = [
    ("odom", "base_footprint"),   # published by the EKF
    ("base_footprint", "laser"),  # published by the robot description
]


def _ros_env_prefix(cfg=None, distro=None):
    """`source` line pair that puts ros2 on PATH for a subprocess `bash -lc`."""
    cfg = cfg or load_config()
    distro = distro or detect_ros_distro()
    ws = cfg.get("workspace_path") or os.path.expanduser("~/linorobot2_ws")
    # rmw_fastrtps holds a service reply until the server's response writer has
    # matched the client's response reader, waiting at most max_blocking_time --
    # 100 ms by default. Bringing nav2 up uncomposed starts ~30 participants at
    # once and that match often takes longer, so the reply is dropped and the
    # lifecycle manager waits forever for an answer that never comes. The XML
    # raises the ceiling for service endpoints only; see the file for details.
    qos_xml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "..", "config", "fastdds_service_qos.xml")
    qos_xml = os.path.normpath(qos_xml)
    qos_export = ""
    if os.path.isfile(qos_xml):
        qos_export = f"export FASTDDS_DEFAULT_PROFILES_FILE={shlex.quote(qos_xml)}; "

    return (
        f"source /opt/ros/{distro}/setup.bash 2>/dev/null || true; "
        f"[ -f {shlex.quote(ws)}/install/setup.bash ] && "
        f"source {shlex.quote(ws)}/install/setup.bash 2>/dev/null || true; "
        f"export ROS_DOMAIN_ID={int(cfg.get('ros_domain_id') or 0)}; "
        f"{qos_export}"
    )


def check_drive_stalled(target, timeout=2.0, settle=1.2):
    """Is the robot being told to move while going nowhere?

    Deliberately measured on *position*, not velocity. A robot driven into a
    wall keeps turning its wheels, so the encoders -- and therefore the odometry
    twist -- still report the commanded speed. That is true of real hardware
    slipping against a skirting board and of the simulated robot held at a
    simulated wall alike, so comparing commanded against measured velocity
    detects nothing. What stops is the pose.

    There is no bump sensor to ask; this samples where the robot says it is,
    waits, and asks again.
    """
    lin_cmd = abs(float(target[0]))
    ang_cmd = abs(float(target[2]))
    if lin_cmd < 0.05 and ang_cmd < 0.15:
        return {"stalled": False, "reason": "not driving"}

    prefix = _ros_env_prefix()

    def _read(field):
        try:
            out = subprocess.run(
                ["bash", "-lc", prefix +
                 f"timeout {timeout} ros2 topic echo /odom/unfiltered --once "
                 f"--field {field} 2>/dev/null"],
                capture_output=True, text=True, timeout=timeout + 3)
            for line in out.stdout.splitlines():
                line = line.strip()
                if line and line[0] in "-0123456789":
                    return float(line)
        except Exception:
            pass
        return None

    x0, y0 = _read("pose.pose.position.x"), _read("pose.pose.position.y")
    if x0 is None or y0 is None:
        return {"stalled": False, "reason": "no odometry"}
    time.sleep(settle)
    x1, y1 = _read("pose.pose.position.x"), _read("pose.pose.position.y")
    if x1 is None or y1 is None:
        return {"stalled": False, "reason": "no odometry"}

    moved = math.hypot(x1 - x0, y1 - y0)
    # how far it should have gone in the sampling window, with generous slack
    expected = lin_cmd * settle
    stalled = lin_cmd >= 0.05 and moved < expected * 0.25

    return {
        "stalled": bool(stalled),
        "commanded_speed": lin_cmd,
        "moved": round(moved, 4),
        "expected": round(expected, 4),
        "reason": "commanded to move but the pose is not advancing" if stalled else "moving",
    }


def _parse_topic_hz(output):
    """Average rate out of `ros2 topic hz` output, or None if it never printed
    one (no publisher, or nothing published inside the window)."""
    m = None
    for line in output.splitlines():
        hit = re.search(r"average rate:\s*([0-9]+\.?[0-9]*)", line)
        if hit:
            m = hit  # keep the last one -- it has seen the most samples
    return float(m.group(1)) if m else None


def check_bringup_health(timeout=4.0):
    """Topic- and TF-level bringup readiness.

    Returns {status, ready, topics: {...}, tf: [...], summary}. `ready` is True
    only when odometry, the IMU and the TF chain are live -- the LiDAR is
    reported but not required (a robot may legitimately run without one).
    """
    res = {
        "status": "ok",
        "ready": False,
        "ros_available": False,
        "topics": {},
        "tf": [],
        "summary": "",
    }
    cfg = load_config()
    prefix = _ros_env_prefix(cfg)

    if not shutil.which("ros2") and not os.path.isdir(f"/opt/ros/{detect_ros_distro()}"):
        res["status"] = "no_ros"
        res["summary"] = "ROS 2 not found on this machine"
        return res

    # One `ros2 topic list` tells us which of the expected topics even exist,
    # so we only pay the per-topic hz timeout for topics that have a publisher.
    try:
        listed = subprocess.run(
            ["bash", "-lc", prefix + "ros2 topic list 2>/dev/null"],
            capture_output=True, text=True, timeout=timeout + 2,
        )
        present = {t.strip() for t in listed.stdout.splitlines() if t.strip()}
        res["ros_available"] = listed.returncode == 0 and bool(present)
    except Exception as e:
        res["status"] = "error"
        res["summary"] = f"could not query the ROS graph: {e}"
        return res

    if not res["ros_available"]:
        # No usable ROS graph at all: report that plainly instead of running
        # (and misreporting) four topic probes and two tf2_echo calls.
        res["status"] = "no_graph"
        res["topics"] = {
            key: {"topic": topic, "what": what, "min_hz": min_hz,
                  "advertised": False, "hz": None, "ok": False}
            for key, topic, what, min_hz in BRINGUP_HEALTH_TOPICS
        }
        res["tf"] = [{"parent": p, "child": c, "ok": False,
                      "detail": "no ROS graph"} for p, c in BRINGUP_TF_CHAIN]
        res["summary"] = (
            "No ROS 2 graph reachable -- is ROS sourced, and is "
            f"ROS_DOMAIN_ID={int(cfg.get('ros_domain_id') or 0)} correct?"
        )
        return res

    for key, topic, what, min_hz in BRINGUP_HEALTH_TOPICS:
        entry = {"topic": topic, "what": what, "min_hz": min_hz,
                 "advertised": topic in present, "hz": None, "ok": False}
        if entry["advertised"]:
            try:
                out = subprocess.run(
                    ["bash", "-lc",
                     prefix + f"timeout {timeout} ros2 topic hz {shlex.quote(topic)} 2>&1"],
                    capture_output=True, text=True, timeout=timeout + 3,
                )
                entry["hz"] = _parse_topic_hz(out.stdout)
            except Exception:
                entry["hz"] = None
            entry["ok"] = entry["hz"] is not None and entry["hz"] >= min_hz
        res["topics"][key] = entry

    # TF: `ros2 run tf2_ros tf2_echo <parent> <child>` prints a transform only
    # when the chain actually resolves.
    for parent, child in BRINGUP_TF_CHAIN:
        link = {"parent": parent, "child": child, "ok": False, "detail": ""}
        try:
            out = subprocess.run(
                ["bash", "-lc",
                 prefix + f"timeout {timeout} ros2 run tf2_ros tf2_echo "
                 f"{shlex.quote(parent)} {shlex.quote(child)} 2>&1"],
                capture_output=True, text=True, timeout=timeout + 3,
            )
            text = out.stdout
            link["ok"] = "Translation:" in text
            if not link["ok"]:
                hit = re.search(r"(?:Exception|Failure|Invalid frame|does not exist)[^\n]*", text)
                link["detail"] = hit.group(0)[:160] if hit else "no transform received"
        except Exception as e:
            link["detail"] = str(e)[:160]
        res["tf"].append(link)

    t = res["topics"]
    odom_ok = t.get("odom", {}).get("ok") or t.get("odom_raw", {}).get("ok")
    tf_ok = all(l["ok"] for l in res["tf"]) if res["tf"] else False
    res["ready"] = bool(odom_ok and tf_ok)

    problems = []
    if not t.get("odom_raw", {}).get("ok"):
        problems.append("no /odom/unfiltered (microcontroller or micro-ROS agent down)")
    elif not t.get("odom", {}).get("ok"):
        problems.append("no /odom (EKF not running)")
    if not t.get("imu", {}).get("ok"):
        problems.append("no /imu/data")
    if not t.get("scan", {}).get("ok"):
        problems.append("no /scan (LiDAR driver down or wrong port)")
    for link in res["tf"]:
        if not link["ok"]:
            problems.append(f"TF {link['parent']}->{link['child']} missing")

    # "Advertised but silent" has a second, easily-missed cause: the publisher
    # is in a container and DDS shared-memory transport can't cross into this
    # process, so discovery succeeds but no samples ever arrive. Worth naming --
    # it looks identical to dead hardware otherwise.
    silent = [e["topic"] for e in t.values() if e["advertised"] and e["hz"] is None]
    if silent:
        res["advertised_but_silent"] = silent
        problems.append(
            "advertised but no messages on " + ", ".join(silent) +
            " -- publisher died, or it is in a container and DDS shared memory "
            "cannot reach this process (try ROS_LOCALHOST_ONLY=0 / a shared "
            "--ipc=host, or run the check where the publisher runs)"
        )

    if res["ready"] and not problems:
        hz = t.get("odom", {}).get("hz") or t.get("odom_raw", {}).get("hz") or 0.0
        res["summary"] = f"Bringup healthy -- odometry {hz:.1f} Hz, TF chain complete"
    elif res["ready"]:
        res["summary"] = "Odometry + TF OK, but: " + "; ".join(problems)
    else:
        res["summary"] = "Bringup not ready: " + ("; ".join(problems) or "no topics published")
    return res





def _parse_port_check_output(output, port, mode, udp_port, res):
    fuser_section = ""
    container_section = ""
    process_section = ""

    current_sec = None
    for line in output.splitlines():
        line_s = line.strip()
        if line_s == "---FUSER---":
            current_sec = "fuser"
            continue
        elif line_s == "---CONTAINERS---":
            current_sec = "containers"
            continue
        elif line_s == "---PROCESSES---":
            current_sec = "processes"
            continue

        if current_sec == "fuser":
            fuser_section += line + " "
        elif current_sec == "containers":
            container_section += line + chr(10)
        elif current_sec == "processes":
            process_section += line + chr(10)

    port_basename = os.path.basename(port) if mode != "udp" else str(udp_port)
    for line in container_section.splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split("|")
        cid = parts[0]
        cname = parts[1] if len(parts) > 1 else ""
        cimg = parts[2] if len(parts) > 2 else ""
        ccmd = parts[3] if len(parts) > 3 else ""

        # A container only holds THIS port if its command actually references
        # it. Matching on the image/name alone (any "microros" image, any
        # "uros*" container) made a single agent on /dev/ttyUSB0 report every
        # other device -- even a nonexistent one -- as occupied.
        looks_like_agent = (
            "microros" in cimg or "micro_ros" in ccmd
            or "uros_agent" in cname or "microros_agent" in cname or "uros" in cname
        )
        is_cont_target = False
        if mode == "udp":
            names_a_port = bool(re.search(r"--port\s+\d+|:\d+/udp|:\d{2,5}\b", ccmd + " " + line))
            if f":{udp_port}" in ccmd or f"--port {udp_port}" in ccmd or f"{udp_port}/udp" in line:
                is_cont_target = True
            elif looks_like_agent and "udp" in ccmd and not names_a_port:
                # agent in UDP mode whose command hides the port number
                is_cont_target = True
        else:
            names_a_device = "/dev/tty" in ccmd
            if port in ccmd or (port_basename and port_basename in ccmd):
                is_cont_target = True
            elif looks_like_agent and not names_a_device:
                # agent container whose command does not disclose its device --
                # can't rule it out, so report it rather than miss a conflict
                is_cont_target = True

        if is_cont_target:
            res["in_use"] = True
            res["holder_type"] = "container"
            res["container_id"] = cid
            res["container_name"] = cname
            res["is_microros"] = "microros" in cimg or "uros" in cname or "micro_ros" in ccmd
            res["details"] = f"Container '{cname}' ({cid[:12]}): {cimg}"
            res["summary"] = f"Occupied by container '{cname}' ({cid[:8]})"
            return res

    raw_pids = [p for p in fuser_section.replace(":", " ").split() if p.isdigit()]
    if raw_pids:
        res["in_use"] = True
        res["pids"] = raw_pids
        res["holder_type"] = "process"
        if "micro_ros_agent" in process_section:
            res["is_microros"] = True
            res["process_names"] = ["micro_ros_agent"]
            res["summary"] = f"In use by micro_ros_agent (PID {raw_pids[0]})"
        else:
            res["summary"] = f"Port in use by PID {raw_pids[0]}"
        res["details"] = f"PIDs: {raw_pids}"
        return res

    if mode == "udp" and str(udp_port) in process_section:
        res["in_use"] = True
        res["holder_type"] = "socket"
        res["summary"] = f"UDP socket :{udp_port} in use"
        return res

    if mode != "udp" and "micro_ros_agent" in process_section and (port in process_section or port_basename in process_section):
        res["in_use"] = True
        res["holder_type"] = "process"
        res["is_microros"] = True
        res["summary"] = f"micro_ros_agent active on {port}"
        res["details"] = process_section.strip()
        return res

    return res


def check_agent_port_status(port="/dev/ttyUSB0", mode="serial", udp_port=8888, host=None, user=None):
    res = {
        "status": "ok",
        "in_use": False,
        "mode": mode,
        "target": port if mode in ["serial", "multiserial"] else f"UDP:{udp_port}",
        "holder_type": "none",
        "pids": [],
        "process_names": [],
        "container_id": "",
        "container_name": "",
        "is_microros": False,
        "details": "",
        "summary": "Port is available"
    }

    if host:
        ssh_cmd = [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
            "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            f"{user}@{host}" if user else host
        ]
        if mode in ["serial", "multiserial"]:
            script = (
                f"echo '---FUSER---'; fuser '{port}' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"pgrep -fa 'micro_ros_agent' 2>/dev/null || true"
            )
        else:
            script = (
                f"echo '---FUSER---'; fuser '{udp_port}/udp' 2>/dev/null || true; "
                f"echo '---CONTAINERS---'; "
                f"docker ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"podman ps --no-trunc --format '{{{{.ID}}}}|{{{{.Names}}}}|{{{{.Image}}}}|{{{{.Command}}}}' 2>/dev/null || true; "
                f"echo '---PROCESSES---'; "
                f"ss -ulnp 'sport = :{udp_port}' 2>/dev/null || true"
            )
        try:
            r = subprocess.run(ssh_cmd + [script], capture_output=True, text=True, timeout=8)
            output = r.stdout
        except Exception as e:
            res["status"] = "error"
            res["details"] = f"SSH error: {e}"
            return res
        return _parse_port_check_output(output, port, mode, udp_port, res)

    # Local port checks
    output_parts = []
    if mode in ["serial", "multiserial"]:
        output_parts.append("---FUSER---")
        if os.path.exists(port):
            try:
                f = subprocess.run(["fuser", port], capture_output=True, text=True, timeout=2)
                output_parts.append(f.stdout)
            except Exception:
                pass
        output_parts.append("---CONTAINERS---")
        try:
            d = subprocess.run(["docker", "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(d.stdout)
        except Exception:
            pass
        try:
            p = subprocess.run(["podman", "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(p.stdout)
        except Exception:
            pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["pgrep", "-fa", "micro_ros_agent"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass
    else:
        output_parts.append("---FUSER---")
        try:
            f = subprocess.run(["fuser", f"{udp_port}/udp"], capture_output=True, text=True, timeout=2)
            output_parts.append(f.stdout)
        except Exception:
            pass
        output_parts.append("---CONTAINERS---")
        try:
            d = subprocess.run(["docker", "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(d.stdout)
        except Exception:
            pass
        try:
            p = subprocess.run(["podman", "ps", "--no-trunc", "--format", "{{.ID}}|{{.Names}}|{{.Image}}|{{.Command}}"], capture_output=True, text=True, timeout=2)
            output_parts.append(p.stdout)
        except Exception:
            pass
        output_parts.append("---PROCESSES---")
        try:
            pr = subprocess.run(["ss", "-ulnp", f"sport = :{udp_port}"], capture_output=True, text=True, timeout=2)
            output_parts.append(pr.stdout)
        except Exception:
            pass

    return _parse_port_check_output("\n".join(output_parts), port, mode, udp_port, res)


def release_agent_port(port="/dev/ttyUSB0", mode="serial", udp_port=8888, host=None, user=None):
    if host:
        ssh_cmd = [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
            "-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null",
            f"{user}@{host}" if user else host
        ]
        if mode == "udp":
            script = (
                f"docker stop microros_agent uros_agent_udp 2>/dev/null || true; "
                f"podman stop microros_agent uros_agent_udp 2>/dev/null || true; "
                f"fuser -k -TERM {udp_port}/udp 2>/dev/null || true; "
                f"pkill -f 'micro_ros_agent.*udp' 2>/dev/null || true; "
                f"sleep 0.5"
            )
        else:
            port_base = os.path.basename(port)
            script = (
                f"docker stop microros_agent uros_agent_serial 2>/dev/null || true; "
                f"podman stop microros_agent uros_agent_serial 2>/dev/null || true; "
                f"[ -e '{port}' ] && fuser -k -TERM '{port}' 2>/dev/null || true; "
                f"pkill -f '[m]icro_ros_agent.*{port_base}' 2>/dev/null || true; "
                f"sleep 0.5"
            )
        try:
            r = subprocess.run(ssh_cmd + [script], capture_output=True, text=True, timeout=10)
            return {"status": "ok", "released": True, "output": r.stdout}
        except Exception as e:
            return {"status": "error", "released": False, "error": str(e)}

    # Local release
    res = {"status": "ok", "released": True, "actions": []}
    if mode == "udp":
        subprocess.run(["fuser", "-k", "-TERM", f"{udp_port}/udp"], capture_output=True, text=True)
        subprocess.run(["pkill", "-f", "micro_ros_agent.*udp"], capture_output=True, text=True)
        for engine in ["docker", "podman"]:
            try:
                subprocess.run([engine, "stop", "microros_agent", "uros_agent_udp"], capture_output=True, text=True)
            except Exception:
                pass
    else:
        port_base = os.path.basename(port)
        if os.path.exists(port):
            subprocess.run(["fuser", "-k", "-TERM", port], capture_output=True, text=True)
            res["actions"].append(f"fuser -k on {port}")
        subprocess.run(["pkill", "-f", f"[m]icro_ros_agent.*{port_base}"], capture_output=True, text=True)
        for engine in ["docker", "podman"]:
            try:
                d = subprocess.run([engine, "ps", "-q", "--filter", "ancestor=microros/micro-ros-agent"], capture_output=True, text=True)
                for cid in d.stdout.split():
                    subprocess.run([engine, "stop", cid], capture_output=True, text=True)
                    res["actions"].append(f"stopped container {cid}")
            except Exception:
                pass
    import time
    time.sleep(0.5)
    return res


def check_container_status():
    has_docker = shutil.which("docker") is not None
    has_podman = shutil.which("podman") is not None
    is_rootless_docker = False

    if has_docker:
        try:
            info_res = subprocess.run(
                ["docker", "info", "-f", "{{.SecurityOptions}}"],
                capture_output=True, text=True, timeout=3
            )
            if "rootless" in (info_res.stdout or "").lower():
                is_rootless_docker = True
            elif "docker.sock" in os.environ.get("DOCKER_HOST", "") and "run/user" in os.environ.get("DOCKER_HOST", ""):
                is_rootless_docker = True
            else:
                res_u = subprocess.run(
                    ["systemctl", "--user", "is-active", "docker"],
                    capture_output=True, text=True, timeout=2
                )
                if res_u.stdout.strip() == "active":
                    is_rootless_docker = True
        except Exception:
            pass

    return {
        "status": "ok",
        "has_docker": has_docker,
        "is_rootless_docker": is_rootless_docker,
        "has_podman": has_podman,
        "platform_system": platform.system(),
    }






def install_container_engine(engine="docker"):
    logs = []
    engine = engine.lower().strip()

    if platform.system() != "Linux":
        return {
            "status": "ok",
            "installed": True,
            "engine": engine,
            "message": "On Windows/macOS, please install Docker Desktop or Podman Desktop.",
            "logs": "Non-Linux platform."
        }

    has_apt = shutil.which("apt-get") is not None
    has_dnf = shutil.which("dnf") is not None

    if engine in ("podman", "podman_systemd"):
        if shutil.which("podman"):
            return {
                "status": "ok",
                "installed": True,
                "engine": "podman",
                "message": "Podman is already installed.",
                "logs": "podman binary present."
            }
        logs.append("Installing Podman...")
        if has_apt:
            subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True, timeout=60)
            r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "podman", "podman-compose"], capture_output=True, text=True, timeout=180)
            logs.append(r.stdout or r.stderr)
        elif has_dnf:
            r = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "podman", "podman-compose"], capture_output=True, text=True, timeout=180)
            logs.append(r.stdout or r.stderr)
        else:
            return {
                "status": "error",
                "installed": False,
                "engine": "podman",
                "message": "No supported package manager found (apt-get or dnf).",
                "logs": "Unsupported package manager."
            }

        installed = shutil.which("podman") is not None
        return {
            "status": "ok" if installed else "error",
            "installed": installed,
            "engine": "podman",
            "message": "Podman installed successfully." if installed else "Podman installation failed.",
            "logs": "\n".join(logs)
        }

    elif engine == "docker":
        if not shutil.which("docker"):
            logs.append("Installing Docker packages...")
            if has_apt:
                subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True, timeout=60)
                r = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "docker.io", "uidmap", "dbus-user-session", "slirp4netns"], capture_output=True, text=True, timeout=180)
                logs.append(r.stdout or r.stderr)
            elif has_dnf:
                r = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "docker-ce", "shadow-utils-subid", "docker-ce-rootless-extras", "slirp4netns", "fuse-overlayfs"], capture_output=True, text=True, timeout=180)
                logs.append(r.stdout or r.stderr)
            else:
                return {
                    "status": "error",
                    "installed": False,
                    "engine": "docker",
                    "message": "No supported package manager found (apt-get or dnf).",
                    "logs": "Unsupported package manager."
                }

        rootless_res = setup_rootless_docker()
        logs.append(rootless_res.get("logs", ""))

        status = check_container_status()
        installed = status["has_docker"]
        return {
            "status": "ok" if installed else "error",
            "installed": installed,
            "is_rootless": status["is_rootless_docker"],
            "engine": "docker",
            "message": "Docker (Rootless) installed and configured." if (installed and status["is_rootless_docker"]) else ("Docker installed." if installed else "Docker installation failed."),
            "logs": "\n".join(logs)
        }

    return {
        "status": "error",
        "installed": False,
        "engine": engine,
        "message": f"Unknown container engine '{engine}'.",
        "logs": "Invalid engine requested."
    }


def setup_rootless_docker():
    logs = []
    user = os.environ.get("USER", "ubuntu")
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    if platform.system() != "Linux":
        return {
            "status": "ok",
            "success": True,
            "message": "Rootless setup is only needed on Linux. Windows and macOS map volume permissions automatically.",
            "is_rootless": True,
            "logs": "Non-Linux platform."
        }

    status = check_container_status()
    if status["is_rootless_docker"]:
        return {
            "status": "ok",
            "success": True,
            "message": f"Rootless Docker is already active for user '{user}' (UID {uid}).",
            "is_rootless": True,
            "logs": "Rootless daemon is already active."
        }

    setuptool = shutil.which("dockerd-rootless-setuptool.sh")
    if not setuptool:
        for p in ["/usr/bin/dockerd-rootless-setuptool.sh", os.path.expanduser("~/.docker/bin/dockerd-rootless-setuptool.sh")]:
            if os.path.exists(p):
                setuptool = p
                break

    if not setuptool:
        logs.append("Attempting to install rootless Docker prerequisites...")
        if shutil.which("apt-get"):
            subprocess.run(["sudo", "-n", "apt-get", "update"], capture_output=True, text=True)
            r_inst = subprocess.run(["sudo", "-n", "apt-get", "install", "-y", "uidmap", "dbus-user-session", "slirp4netns", "docker-ce-rootless-extras"], capture_output=True, text=True)
            logs.append(r_inst.stdout or r_inst.stderr)
        elif shutil.which("dnf"):
            r_inst = subprocess.run(["sudo", "-n", "dnf", "install", "-y", "shadow-utils-subid", "docker-ce-rootless-extras", "slirp4netns", "fuse-overlayfs"], capture_output=True, text=True)
            logs.append(r_inst.stdout or r_inst.stderr)
        setuptool = shutil.which("dockerd-rootless-setuptool.sh")

    if not setuptool:
        return {
            "status": "error",
            "success": False,
            "message": "dockerd-rootless-setuptool.sh not found. Install uidmap and docker-ce-rootless-extras.",
            "is_rootless": False,
            "logs": "\n".join(logs)
        }

    logs.append("Running dockerd-rootless-setuptool.sh install -f...")
    try:
        r_install = subprocess.run([setuptool, "install", "-f"], capture_output=True, text=True, timeout=30)
        logs.append(r_install.stdout)
        if r_install.stderr:
            logs.append(r_install.stderr)
    except Exception as e:
        logs.append(f"Execution error: {e}")

    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "enable", "--now", "docker.service"], capture_output=True, text=True)
    subprocess.run(["loginctl", "enable-linger", user], capture_output=True, text=True)

    docker_sock = f"unix://{xdg_runtime}/docker.sock"
    os.environ["DOCKER_HOST"] = docker_sock

    bashrc = os.path.expanduser("~/.bashrc")
    try:
        if os.path.exists(bashrc):
            with open(bashrc, "r", encoding="utf-8") as f:
                b_cnt = f.read()
            if "DOCKER_HOST" not in b_cnt:
                with open(bashrc, "a", encoding="utf-8") as f:
                    f.write(f'\nexport DOCKER_HOST="{docker_sock}"\n')
                logs.append("Appended DOCKER_HOST to ~/.bashrc")
    except Exception as e:
        logs.append(f"Notice: could not update ~/.bashrc: {e}")

    new_status = check_container_status()
    success = new_status["is_rootless_docker"]

    return {
        "status": "ok" if success else "warning",
        "success": success,
        "is_rootless": success,
        "message": f"Rootless Docker {'configured and active' if success else 'setup completed with warnings'}.",
        "logs": "\n".join(logs)
    }


def get_rootless_info():
    c_status = check_container_status()
    uid = os.getuid() if hasattr(os, "getuid") else 1000
    user = os.environ.get("USER", "user")
    xdg_runtime = os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{uid}")

    return {
        "status": "ok",
        "is_rootless": c_status["is_rootless_docker"],
        "has_docker": c_status["has_docker"],
        "has_podman": c_status["has_podman"],
        "platform_system": c_status["platform_system"],
        "user": user,
        "uid": uid,
        "xdg_runtime_dir": xdg_runtime,
        "commands": {
            "ubuntu_debian": [
                "sudo apt-get update && sudo apt-get install -y uidmap dbus-user-session slirp4netns",
                "dockerd-rootless-setuptool.sh install",
                "systemctl --user enable --now docker.service",
                f"loginctl enable-linger {user}",
                f'export DOCKER_HOST="unix://{xdg_runtime}/docker.sock"',
                f'grep -q "DOCKER_HOST" ~/.bashrc || echo \'export DOCKER_HOST="unix://{xdg_runtime}/docker.sock"\' >> ~/.bashrc'
            ],
            "podman_alternative": [
                "sudo apt-get install -y podman  # or: sudo dnf install -y podman",
                "podman run --rm hello-world"
            ]
        }
    }




def get_autostart_status():
    service_file = os.path.expanduser("~/.config/systemd/user/linorobot2-autostart.service")
    has_service = os.path.exists(service_file)
    enabled = False
    active = False
    lingering = False
    status_output = ""

    user = os.environ.get("USER", "ubuntu")
    try:
        r_l = subprocess.run(["loginctl", "show-user", user, "--property=Linger"], capture_output=True, text=True, timeout=2)
        lingering = "yes" in r_l.stdout.lower()
    except Exception:
        pass

    if has_service:
        try:
            r_e = subprocess.run(["systemctl", "--user", "is-enabled", "linorobot2-autostart.service"], capture_output=True, text=True, timeout=2)
            enabled = r_e.stdout.strip() == "enabled"
        except Exception:
            pass

        try:
            r_a = subprocess.run(["systemctl", "--user", "is-active", "linorobot2-autostart.service"], capture_output=True, text=True, timeout=2)
            active = r_a.stdout.strip() == "active"
        except Exception:
            pass

        try:
            r_s = subprocess.run(["systemctl", "--user", "status", "linorobot2-autostart.service", "--no-pager"], capture_output=True, text=True, timeout=3)
            status_output = r_s.stdout
        except Exception:
            pass

    return {
        "status": "ok",
        "has_service": has_service,
        "enabled": enabled,
        "active": active,
        "lingering": lingering,
        "service_name": "linorobot2-autostart.service",
        "details": status_output.strip()
    }


def enable_autostart(data):
    cfg = load_config()
    stack = data.get("stack", "full_nav2")  # "full_nav2" | "agent_bringup" | "agent_only"
    mode = data.get("mode") or cfg.get("install_mode", "native")
    agent_engine = data.get("agent_engine") or cfg.get("agent_engine", "docker")
    distro = data.get("distro") or cfg.get("ros_distro", "jazzy")
    map_path = data.get("map_path") or os.path.expanduser("~/.config/linorobot2/maps/map.yaml")
    base_type = cfg.get("base_type", "2wd")
    agent_dev = cfg.get("agent_device", "/dev/ttyACM0")
    agent_baud = cfg.get("agent_baud", "921600")
    agent_port = cfg.get("agent_port", "8888")
    agent_transport = cfg.get("agent_transport", "serial")
    ws_path = cfg.get("workspace_path", os.path.expanduser("~/linorobot2_ws"))
    user = os.environ.get("USER", "ubuntu")

    bin_dir = os.path.expanduser("~/bin")
    systemd_dir = os.path.expanduser("~/.config/systemd/user")
    os.makedirs(bin_dir, exist_ok=True)
    os.makedirs(systemd_dir, exist_ok=True)

    script_path = os.path.join(bin_dir, "linorobot2-autostart.sh")
    service_path = os.path.join(systemd_dir, "linorobot2-autostart.service")

    script_lines = [
        "#!/usr/bin/env bash",
        "# Auto-generated by Linorobot2 Console",
        "set -e",
        "",
        f'export ROS_DISTRO="{distro}"',
        f'export LINOROBOT2_BASE="{base_type}"',
        f'export WS="{ws_path}"',
        "",
        'echo "=========================================="',
        f'echo " Linorobot2 Headless Autonomous Bootup"',
        f'echo " Stack: {stack} | Mode: {mode} | Distro: {distro}"',
        'echo "=========================================="',
        "",
        "# Pre-cleanup agent port",
        f'fuser -k -TERM {agent_dev} 2>/dev/null || true',
        "sleep 1",
        "",
        "# 1. Start micro-ROS Agent"
    ]

    if agent_engine in ("docker", "podman"):
        img = f"microros/micro-ros-agent:{distro}"
        eng = agent_engine
        if agent_transport == "udp4":
            script_lines.append(f'{eng} run --rm --net=host --name linorobot2_agent_boot {img} udp4 --port {agent_port} &')
        else:
            script_lines.append(f'{eng} run --rm --net=host --privileged -v /dev:/dev --device {agent_dev} --name linorobot2_agent_boot {img} serial --dev {agent_dev} -b {agent_baud} &')
    else:
        script_lines.append(f'[ -f /opt/ros/{distro}/setup.bash ] && source /opt/ros/{distro}/setup.bash')
        script_lines.append('[ -f ~/uros_ws/install/setup.bash ] && source ~/uros_ws/install/setup.bash')
        if agent_transport == "udp4":
            script_lines.append(f'ros2 run micro_ros_agent micro_ros_agent udp4 -p {agent_port} &')
        else:
            script_lines.append(f'ros2 run micro_ros_agent micro_ros_agent serial --dev {agent_dev} -b {agent_baud} &')

    script_lines.extend([
        "AGENT_PID=$!",
        'echo "micro-ROS agent launched (PID: $AGENT_PID)"',
        "sleep 3",
        ""
    ])

    if stack in ("agent_bringup", "full_nav2"):
        script_lines.extend([
            "# 2. Start Robot Bringup",
            f'[ -f /opt/ros/{distro}/setup.bash ] && source /opt/ros/{distro}/setup.bash',
            f'[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"',
            f'ros2 launch linorobot2_bringup bringup.launch.py base_type:={base_type} &',
            "BRINGUP_PID=$!",
            'echo "Robot bringup launched (PID: $BRINGUP_PID)"',
            "sleep 4",
            ""
        ])

    if stack == "full_nav2":
        script_lines.extend([
            "# 3. Start Nav2 Autonomous Navigation",
            f'[ -f /opt/ros/{distro}/setup.bash ] && source /opt/ros/{distro}/setup.bash',
            f'[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"',
        ])
        if map_path and os.path.exists(map_path):
            script_lines.append(f'ros2 launch linorobot2_navigation navigation.launch.py map:="{map_path}" &')
        else:
            script_lines.append('ros2 launch linorobot2_navigation navigation.launch.py slam:=true &')
        script_lines.extend([
            "NAV2_PID=$!",
            'echo "Nav2 Navigation launched (PID: $NAV2_PID)"',
            "",
            "# Wait for processes",
            "wait $NAV2_PID $BRINGUP_PID $AGENT_PID"
        ])
    elif stack == "agent_bringup":
        script_lines.append("wait $BRINGUP_PID $AGENT_PID")
    else:
        script_lines.append("wait $AGENT_PID")

    with open(script_path, "w", encoding="utf-8") as f:
        f.write("\n".join(script_lines) + "\n")
    os.chmod(script_path, 0o755)

    unit_content = f"""[Unit]
Description=Linorobot2 Autonomous Headless Robotics Stack ({stack})
After=network.target
Wants=network.target

[Service]
Type=simple
Restart=always
RestartSec=5s
ExecStart={script_path}
ExecStop=/bin/kill -TERM $MAINPID
TimeoutStopSec=10

[Install]
WantedBy=default.target
"""
    with open(service_path, "w", encoding="utf-8") as f:
        f.write(unit_content)

    subprocess.run(["loginctl", "enable-linger", user], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "daemon-reload"], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "enable", "linorobot2-autostart.service"], capture_output=True, text=True)

    return {
        "status": "ok",
        "enabled": True,
        "script_path": script_path,
        "service_path": service_path,
        "message": f"Autostart enabled ({stack}). Systemd unit created and lingering enabled."
    }


def disable_autostart():
    subprocess.run(["systemctl", "--user", "stop", "linorobot2-autostart.service"], capture_output=True, text=True)
    subprocess.run(["systemctl", "--user", "disable", "linorobot2-autostart.service"], capture_output=True, text=True)
    return {
        "status": "ok",
        "enabled": False,
        "message": "Autostart service stopped and disabled."
    }


def get_autostart_logs():
    res = subprocess.run(
        ["journalctl", "--user", "-u", "linorobot2-autostart.service", "-n", "60", "--no-pager"],
        capture_output=True, text=True, timeout=3
    )
    return {
        "status": "ok",
        "logs": res.stdout or res.stderr or "No journal logs found for linorobot2-autostart.service"
    }


def generate_custom_robot_specs(description):
    d = description.lower()
    
    # 1. KINEMATICS & CHASSIS DESIGN
    if any(k in d for k in ["mecanum", "omni", "strafe", "holonomic"]):
        base = "mecanum"
        base_title = "4WD Mecanum (Omnidirectional / Holonomic)"
        fuse_vy = True
        default_wheel_diam = 0.097
        default_track = 0.30
        default_wheelbase = 0.25
        motion_model = "nav2_amcl::OmniMotionModel"
        min_y_thresh = 0.001
    elif any(k in d for k in ["4wd", "skid", "four wheel"]):
        base = "4wd"
        base_title = "4WD Skid Steer (Differential Kinematics)"
        fuse_vy = False
        default_wheel_diam = 0.130
        default_track = 0.35
        default_wheelbase = 0.28
        motion_model = "nav2_amcl::DifferentialMotionModel"
        min_y_thresh = 0.5
    else:
        base = "2wd"
        base_title = "2WD Differential Drive"
        fuse_vy = False
        default_wheel_diam = 0.066
        default_track = 0.16
        default_wheelbase = 0.0
        motion_model = "nav2_amcl::DifferentialMotionModel"
        min_y_thresh = 0.5

    # Extract or infer custom wheel dimensions
    wheel_diam = default_wheel_diam
    if "97mm" in d or "0.097" in d: wheel_diam = 0.097
    elif "66mm" in d or "0.066" in d: wheel_diam = 0.066
    elif "130mm" in d or "0.13" in d: wheel_diam = 0.130
    elif "150mm" in d or "0.15" in d: wheel_diam = 0.150

    track_width = default_track
    if "30cm" in d or "0.3m" in d: track_width = 0.30
    elif "35cm" in d or "0.35m" in d: track_width = 0.35
    elif "16cm" in d or "0.16m" in d: track_width = 0.16
    elif "50cm" in d or "0.5m" in d: track_width = 0.50

    wheelbase = default_wheelbase
    if "25cm" in d or "0.25m" in d: wheelbase = 0.25
    elif "28cm" in d or "0.28m" in d: wheelbase = 0.28
    elif "26cm" in d or "0.26m" in d: wheelbase = 0.26

    # Motor & Encoder Specifications
    gear_ratio = 30.0
    cpr = 1320
    motor_rpm = 330
    if "jgb37" in d or "330rpm" in d or "30:1" in d:
        gear_ratio = 30.0
        motor_rpm = 330
        cpr = 1320
    elif "500rpm" in d or "20:1" in d:
        gear_ratio = 20.0
        motor_rpm = 500
        cpr = 880

    theoretical_max_speed = round(motor_rpm * 3.14159 * wheel_diam / 60.0, 2)
    safe_cruise_speed = round(theoretical_max_speed * 0.6, 2)
    if "fast" in d or "warehouse" in d:
        safe_cruise_speed = min(0.85, theoretical_max_speed)
    elif "door" in d or "narrow" in d or "cautious" in d:
        safe_cruise_speed = min(0.35, safe_cruise_speed)

    # MCU selection
    mcu = "pico2"
    if "esp32" in d or "esp32s3" in d: mcu = "esp32"
    elif "teensy" in d: mcu = "teensy41"

    # Motor Driver
    driver = "MOTOR_DRIVER_MDD10A"
    if "l298" in d or "l298n" in d: driver = "MOTOR_DRIVER_L298"
    elif "tb6612" in d: driver = "MOTOR_DRIVER_TB6612FNG"
    elif "bts7960" in d: driver = "MOTOR_DRIVER_BTS7960"

    # LiDAR
    laser = "ldlidar"
    laser_name = "LD19 / LD06 (12m range)"
    laser_max_range = 12.0
    if "ydlidar" in d:
        laser = "ydlidar"
        laser_name = "YDLIDAR X4 / G4 (10m range)"
        laser_max_range = 10.0
    elif "rplidar" in d:
        laser = "rplidar"
        laser_name = "RPLIDAR A1 / A2 (12m range)"
        laser_max_range = 12.0

    # Footprint Calculation
    half_x = round((wheelbase + wheel_diam) / 2.0 + 0.05, 3) if wheelbase > 0 else round(wheel_diam / 2.0 + 0.10, 3)
    half_y = round(track_width / 2.0 + 0.06, 3)
    footprint = f"[[-{half_x}, -{half_y}], [-{half_x}, {half_y}], [{half_x}, {half_y}], [{half_x}, -{half_y}]]"

    # 2. TUNING CONFIGURATION
    is_narrow = ("door" in d or "narrow" in d or "tight" in d)
    inflation_radius = 0.52 if is_narrow else 0.70
    cost_scaling = 5.5 if is_narrow else 3.0

    nav2_tuning = {
        "base": base,
        "max_vel_x": safe_cruise_speed,
        "max_vel_y": safe_cruise_speed if base == "mecanum" else 0.0,
        "max_vel_theta": 2.5,
        "max_accel_x": 2.5,
        "max_accel_y": 2.5 if base == "mecanum" else 0.0,
        "max_accel_theta": 3.2,
        "desired_linear_vel": round(safe_cruise_speed * 0.8, 2),
        "inflation_radius": inflation_radius,
        "cost_scaling_factor": cost_scaling,
        "robot_model_type": motion_model,
        "min_y_velocity_threshold": min_y_thresh,
        "footprint": footprint
    }

    ekf_tuning = {
        "base": base,
        "frequency": 50.0,
        "two_d_mode": True,
        "fuse_vy": fuse_vy,
        "fuse_imu_yaw": False,
        "rationale": f"Fuse vy={fuse_vy} for {base.upper()} kinematics; 50Hz update rate synchronized with firmware."
    }

    slam_tuning = {
        "resolution": 0.025 if "high res" in d or "precision" in d else 0.05,
        "max_laser_range": laser_max_range,
        "minimum_travel_distance": 0.3 if is_narrow else 0.5,
        "minimum_travel_heading": 0.3 if is_narrow else 0.5
    }

    # Firmware config header preview
    firmware_config = f"""#ifndef CUSTOM_CONFIG_H
#define CUSTOM_CONFIG_H

#define KINEMATICS LINO_BASE_{base.upper()}
#define MOTOR_DRIVER {driver}
#define WHEEL_DIAMETER {wheel_diam}
#define LR_WHEELS_DISTANCE {track_width}
#define FR_WHEELS_DISTANCE {wheelbase}
#define COUNTS_PER_REV {cpr}
#define MOTOR_MAX_RPM {motor_rpm}

// Recommended Closed-Loop PID Velocity Control
#define K_P 0.6
#define K_I 0.3
#define K_D 0.1

#endif"""

    return {
        "design": {
            "base_type": base,
            "title": base_title,
            "mcu": mcu,
            "motor_driver": driver,
            "wheel_diameter_m": wheel_diam,
            "track_width_m": track_width,
            "wheelbase_m": wheelbase,
            "gear_ratio": gear_ratio,
            "cpr": cpr,
            "motor_max_rpm": motor_rpm,
            "theoretical_max_speed_mps": theoretical_max_speed,
            "recommended_cruise_speed_mps": safe_cruise_speed,
            "laser_sensor": laser,
            "laser_name": laser_name,
            "footprint": footprint,
            "firmware_config_h": firmware_config
        },
        "tuning": {
            "nav2": nav2_tuning,
            "ekf": ekf_tuning,
            "slam": slam_tuning
        },
        "workflow": [
            f"1. [Hardware Design] Configured {base.upper()} kinematics (D={wheel_diam*1000:.0f}mm, Track={track_width*1000:.0f}mm, CPR={cpr}).",
            f"2. [Firmware Config] Generated custom_config.h with closed-loop PID gains and motor driver {driver}.",
            f"3. [ROS 2 Description] Calculated rectangular footprint polygon {footprint}.",
            f"4. [EKF Estimation] Tuned robot_localization at 50Hz (fuse_vy={fuse_vy}, planar 2D mode).",
            f"5. [Nav2 Autonomous Navigation] Configured velocity smoother (v_x={safe_cruise_speed} m/s) & costmap inflation ({inflation_radius}m).",
            f"6. [SLAM Toolbox] Configured {laser_name} with {laser_max_range}m laser range and {slam_tuning['resolution']}m grid resolution."
        ]
    }

def find_laser_driver_info(model_code):
    """Find key, entry, driver_pkg for a given model code or sensor key."""
    if not model_code:
        return None, None, None
    m_lower = model_code.strip().lower()
    for key, entry in LASER_SENSORS.items():
        if key.lower() == m_lower:
            pkg = entry.get("driver_pkg")
            return key, entry, pkg
        for m in entry.get("models", []):
            if m.get("code", "").lower() == m_lower:
                pkg = entry.get("driver_pkg")
                return key, entry, pkg
    return None, None, None



# Packages that are not published as binaries on every distro. nav2_bringup in
# particular has no ros-lyrical-nav2-bringup: apt answers "Unable to locate
# package" and the 1-Click chain launched anyway, straight into
# "package 'nav2_bringup' not found". Build those from source into the
# workspace instead.
SOURCE_FALLBACK_REPOS = {
    # Rolling on Ubuntu 26.04 publishes the nav2 *components* and
    # robot_localization, but neither slam_toolbox nor nav2_bringup. Without an
    # entry here, ensureRosPackages fell through to apt, got "E: Unable to
    # locate package ros-rolling-slam-toolbox", warned, and launched SLAM
    # anyway -- so the stack came up with no mapper at all.
    "slam_toolbox": {
        "repo": "https://github.com/SteveMacenski/slam_toolbox.git",
        "keep": ["slam_toolbox"],
    },
    "nav2_bringup": {
        "repo": "https://github.com/ros-navigation/navigation2.git",
        # nav2_bringup's CMakeLists find_package()s the `navigation2`
        # metapackage, and that has no binary package either -- building
        # nav2_bringup alone stops at "Could not find a package configuration
        # file provided by navigation2". Keep and build both.
        "keep": ["nav2_bringup", "navigation2"],
        # nav2_bringup only *launches* the rest of the stack; without the
        # runtime packages the launch fails on the first node it cannot find.
        "pre": "nav2_stack",
        # ...and for the same reason rosdep cannot resolve `navigation2` from
        # apt: it reports "Unable to locate package ros-<distro>-navigation2"
        # and then refuses to install any of the other dependencies either.
        # nav2_smac_planner is not published for every distro either (Lyrical
        # ships navfn, planner and theta-star but not smac), and the
        # navigation2 metapackage depends on it, so rosdep tried
        # "apt-get install ros-lyrical-nav2-smac-planner" and failed. Console
        # configures nav2_navfn_planner::NavfnPlanner, so nothing we launch
        # needs smac -- skip it rather than fail on a planner we do not use.
        "skip_keys": ["navigation2", "nav2_smac_planner"],
    },
}


def build_nav2_stack_cmd(distro=None):
    """apt-install every published nav2 package for this distro.

    On distros where ros-<distro>-navigation2 exists, installing it pulls the
    whole stack. Lyrical publishes the individual nav2_* packages but not that
    metapackage, so nav2_bringup's launch files went looking for nodes nobody
    had installed -- discovered one at a time, one failed launch per package
    ("package 'nav2_waypoint_follower' not found", then the next one).
    Expanding the apt name pattern gets the same set in one step and does not
    need a hand-maintained list that drifts with every nav2 release.
    """
    distro = distro or detect_ros_distro()
    # opennav_* too: nav2_bringup launches opennav_docking, and that name does
    # not start with nav2_, so a ^ros-<distro>-nav2- pattern alone left the
    # launch failing on "package 'opennav_docking' not found".
    return (
        f"NAV2_PKGS=$(apt-cache --names-only search '^ros-{distro}-(nav2|opennav)-' "
        "| awk '{print $1}' | grep -v -- '-dbgsym$' | tr '\n' ' ') && "
        "if [ -n \"$NAV2_PKGS\" ]; then sudo apt-get update && "
        "sudo apt-get install -y $NAV2_PKGS; "
        f"else echo '[console] no ros-{distro}-nav2-* / opennav-* packages in apt'; fi"
    )


# The nodes nav2_bringup's launch files actually spawn. Checking these tells us
# whether the stack is usable; the apt expansion above installs it.
NAV2_RUNTIME_PACKAGES = [
    "nav2_controller",
    "nav2_planner",
    "nav2_behaviors",
    "nav2_bt_navigator",
    "nav2_waypoint_follower",
    "nav2_velocity_smoother",
    "nav2_smoother",
    "nav2_lifecycle_manager",
    "nav2_map_server",
    "nav2_amcl",
    "nav2_collision_monitor",
    "opennav_docking",
]


def nav2_stack_status(distro=None, ws=None):
    distro = distro or detect_ros_distro()
    ws = os.path.abspath(os.path.expanduser(ws or DEFAULT_CONFIG["workspace_path"]))
    missing = [p for p in NAV2_RUNTIME_PACKAGES
               if not check_sensor_driver_installed(p, ws=ws)[0]]
    return {
        "distro": distro,
        "installed": not missing,
        "missing": missing,
        "command": build_nav2_stack_cmd(distro) if missing else None,
    }


def _source_fallback_entry(pkg):
    entry = SOURCE_FALLBACK_REPOS.get(pkg)
    if entry is None:
        return None
    if isinstance(entry, str):
        entry = {"repo": entry}
    return {
        "repo": entry["repo"],
        "keep": entry.get("keep") or [pkg],
        "skip_keys": entry.get("skip_keys") or [],
        "pre": entry.get("pre"),
    }


def apt_package_available(apt_pkg):
    """Whether apt knows this package at all (not whether it is installed)."""
    try:
        out = subprocess.run(["apt-cache", "policy", apt_pkg],
                             capture_output=True, text=True, timeout=20).stdout
    except Exception:
        return True   # can't tell -- let apt speak for itself
    return bool(out.strip()) and "Candidate: (none)" not in out


def build_source_package_cmd(pkg, distro=None, ws=None):
    """Clone the upstream repo and colcon build just this package into ws."""
    entry = _source_fallback_entry(pkg)
    if not entry:
        return None
    repo = entry["repo"]
    keep = entry["keep"]
    skip_keys = " ".join(["microxrcedds_agent"] + entry["skip_keys"])
    keep_re = "/(" + "|".join(keep) + ")$"
    distro = distro or detect_ros_distro()
    ws = os.path.abspath(os.path.expanduser(ws or DEFAULT_CONFIG["workspace_path"]))
    name = repo.rstrip("/").rsplit("/", 1)[-1].removesuffix(".git")
    steps = []
    if entry["pre"] == "nav2_stack":
        steps.append(build_nav2_stack_cmd(distro))
    # Source ROS 2 before rosdep and colcon. Console runs commands through
    # `bash -lc`, and a *fresh* box has nothing sourced in its profile -- so on
    # the first Rolling box this ran with no ROS environment at all and failed
    # twice over: rosdep said "ROS distro is not set ... Cannot locate rosdep
    # definition for [ament_cmake]", and colcon then died on "Could not find a
    # package configuration file provided by ament_cmake". It only ever worked
    # on boxes where something else had already sourced setup.bash.
    steps = [
        f"export ROS_DISTRO={distro}",
        f"source /opt/ros/{distro}/setup.bash",
        # Packages built into the workspace earlier in the same 1-Click chain
        # (nav2_bringup before slam_toolbox, say) must be visible too.
        f"([ -f {ws}/install/setup.bash ] && source {ws}/install/setup.bash || true)",
    ] + steps
    return " && ".join(steps + [
        f"mkdir -p {ws}/src",
        f"cd {ws}/src",
        f"([ -d {name} ] || git clone --depth 1 -b {distro} {repo} {name} "
        f"|| git clone --depth 1 -b main {repo} {name} || git clone --depth 1 {repo} {name})",
        # Hide every other package in the clone. A monorepo like navigation2
        # drops ~40 packages into the workspace, and colcon then insists on
        # building each dependency from the workspace even though they are
        # installed in /opt/ros -- the build dies on "Failed to find
        # .../install/nav2_common/share/nav2_common/package.sh" without
        # compiling anything. COLCON_IGNORE sends it back to /opt/ros.
        # Re-clearing the marker on the kept packages matters on a second run,
        # where an earlier, narrower `keep` list may have ignored one of them.
        f"find {ws}/src/{name} -name package.xml -printf '%h\\n' "
        f"| grep -vE '{keep_re}' | xargs -r -I@ touch @/COLCON_IGNORE",
        f"find {ws}/src/{name} -name package.xml -printf '%h\\n' "
        f"| grep -E '{keep_re}' | xargs -r -I@ rm -f @/COLCON_IGNORE",
        f"cd {ws}",
        # Scope rosdep to the packages actually being built. Pointed at the
        # whole workspace it also resolves the ignored ones and drags in the
        # entire simulation stack.
        f"(rosdep install --from-paths $(find {ws}/src/{name} -name package.xml "
        f"-printf '%h\\n' | grep -E '{keep_re}' | tr '\\n' ' ') "
        f"--ignore-src -y --rosdistro {distro} --skip-keys '{skip_keys}' || true)",
        f"colcon build --symlink-install --packages-select {' '.join(keep)}",
        f"echo '[console] built {pkg} from source into {ws}'",
    ])


def get_package_install_info(pkg, distro=None, ws=None):
    """Check if a ROS package is installed; if not, return an install command."""
    if not pkg:
        return {"package": "", "installed": True, "install_cmd": None}
    distro = distro or detect_ros_distro()
    ws = os.path.abspath(os.path.expanduser(ws or DEFAULT_CONFIG["workspace_path"]))
    installed, reason = check_sensor_driver_installed(pkg, ws=ws)
    if installed:
        return {"package": pkg, "installed": True, "install_cmd": None, "reason": reason}
    apt_pkg = f"ros-{distro}-{pkg.replace('_', '-')}"
    if apt_package_available(apt_pkg):
        return {
            "package": pkg,
            "installed": False,
            "apt_package": apt_pkg,
            "source": "apt",
            "install_cmd": f"sudo apt-get update && sudo apt-get install -y {apt_pkg}",
            "reason": reason,
        }
    src_cmd = build_source_package_cmd(pkg, distro=distro, ws=ws)
    return {
        "package": pkg,
        "installed": False,
        "apt_package": apt_pkg,
        "source": "source" if src_cmd else "apt",
        "install_cmd": src_cmd or f"sudo apt-get update && sudo apt-get install -y {apt_pkg}",
        "reason": (reason or "") + f" (no {apt_pkg} in apt)",
    }

def check_sensor_driver_installed(pkg, ws=None):
    """Check whether a ROS 2 driver package is installed in /opt/ros, workspace install, or workspace src."""
    if not pkg:
        return True, "No package specified"
    ws = os.path.abspath(os.path.expanduser(ws or "~/linorobot2_ws"))
    distro = detect_ros_distro()

    # 1. System ROS install: /opt/ros/<distro>/share/<pkg>
    if os.path.isdir(f"/opt/ros/{distro}/share/{pkg}"):
        return True, f"Installed in /opt/ros/{distro}/share/{pkg}"

    # 2. Workspace install: <ws>/install/<pkg>
    if os.path.isdir(os.path.join(ws, "install", pkg)):
        return True, f"Installed in {ws}/install/{pkg}"

    # A source tree in <ws>/src is NOT the package being available: a source
    # build that failed leaves the clone behind, and counting it as installed
    # made the next 1-Click run report the package as satisfied and launch
    # without it. Seen on Rolling, where the slam_toolbox build failed on its
    # own missing dependencies and the leftover clone then masked it. Only
    # <ws>/install/<pkg> (checked above) means built.

    # 4. Check via ros2 CLI if accessible in environment
    try:
        res = subprocess.run(["ros2", "pkg", "prefix", pkg], capture_output=True, text=True, timeout=2.0)
        if res.returncode == 0 and res.stdout.strip():
            return True, f"Found via ros2 pkg prefix: {res.stdout.strip()}"
    except Exception:
        pass

    return False, f"Package '{pkg}' not found in /opt/ros/{distro} or workspace {ws}"


def get_sensor_driver_status(sensor_code, ws=None):
    """Full driver status info dictionary for the frontend."""
    if not sensor_code:
        return {"sensor": "", "package": None, "installed": True, "reason": "No sensor configured"}
    ws = ws or os.path.expanduser("~/linorobot2_ws")
    key, entry, pkg = find_laser_driver_info(sensor_code)
    if not pkg:
        return {"sensor": sensor_code, "key": key, "package": None, "installed": True, "needs_driver": False, "reason": "No separate ROS 2 driver package required"}

    installed, reason = check_sensor_driver_installed(pkg, ws=ws)
    install_cmd = build_sensor_install_cmd("laser", key, ws=ws) if (not installed and key) else None
    return {
        "sensor": sensor_code,
        "key": key,
        "package": pkg,
        "installed": installed,
        "needs_driver": True,
        "reason": reason,
        "install_cmd": install_cmd
    }


def _find_bringup_container():
    for engine in ["docker", "podman"]:
        if shutil.which(engine):
            try:
                res = subprocess.run([engine, "ps", "--filter", "name=bringup", "--format", "{{.Names}}"], capture_output=True, text=True, timeout=2.0)
                if res.returncode == 0 and "bringup" in res.stdout:
                    return True, engine
            except Exception:
                pass
    return False, None


def imported_config_to_console_config(result):
    """Map the fields parsed out of a <robot>_config.h onto Console's config keys."""
    mapped = {}
    if result.get("base"):
        mapped["base_type"] = result["base"]
    if result.get("agent_baud"):
        mapped["agent_baud"] = str(result["agent_baud"])
    if result.get("transport"):
        mapped["agent_transport"] = result["transport"]
    if "has_imu" in result:
        # madgwick fuses a real IMU; with a fake one there is nothing to fuse
        mapped["madgwick"] = bool(result["has_imu"])
    if result.get("agent_port"):
        mapped["agent_port"] = str(result["agent_port"])
    if result.get("lidar_transport"):
        mapped["laser_transport"] = result["lidar_transport"]
        # Both routes carry LD19-format frames at LIDAR_BAUDRATE, but the port
        # only means anything on the UDP route -- LIDAR_PORT sits in every
        # header, including the ones that read the LiDAR off a UART pin.
        if result.get("lidar_baud"):
            mapped["laser_baud"] = str(result["lidar_baud"])
        if result["lidar_transport"] == "udp_bridge" and result.get("lidar_udp_port"):
            mapped["laser_udp_port"] = str(result["lidar_udp_port"])
    if result.get("fake_ld19"):
        # The emulator speaks LD19 frames, so the driver family and the model
        # are both known exactly. A *relayed* real LiDAR could be any LDROBOT
        # unit, so nothing is assumed about the model there.
        mapped["laser_sensor"] = "ldlidar"
        mapped["laser_model"] = "ld19"
    return {k: v for k, v in mapped.items() if k in DEFAULT_CONFIG}


def persist_imported_config(result):
    """Write an imported header through to robot_config.yaml.

    robot_config.yaml is the single source of truth: the launchers read the
    base, port and baud from it, and nothing depends on a shell export. The
    parsed values used to be handed back to the browser and dropped, so the
    header showed the imported robot while bringup still launched with the
    previous port and baud rate.
    """
    mapped = imported_config_to_console_config(result)
    if not mapped:
        return {}
    cfg = load_config()
    cfg.update(mapped)
    save_config(cfg)
    return mapped


def parse_robot_config_header(text):
    """Parse a linorobot2_hardware `<robot>_config.h` into a flat dict.

    Module level, not a handler method, so the regex discipline below can be
    tested directly -- these patterns are the only thing standing between a
    commented-out `// #define USE_LIDAR_UDP` and a robot configured for the
    wrong LiDAR route."""
    result = {}

    # All patterns below require an uncommented, line-anchored #define --
    # matching parser.py's _define_bool discipline -- so a `// #define
    # USE_WIFI` or similar commented-out line is correctly ignored rather
    # than misread as active.
    m = re.search(r"^[ \t]*#define\s+LINO_BASE\s+(\w+)", text, re.MULTILINE)
    if m:
        base_map = {"DIFFERENTIAL_DRIVE": "2wd", "SKID_STEER": "4wd", "MECANUM": "mecanum"}
        result["base"] = base_map.get(m.group(1), "")

    m = re.search(r"^[ \t]*#define\s+BAUDRATE\s+(\d+)", text, re.MULTILINE)
    if m:
        result["agent_baud"] = m.group(1)

    # USE_WIFI is config-engine's actual macro for this (parser.py's
    # _define_bool("USE_WIFI")) -- not a "WIFI_UDP"/"USE_WIFI_TRANSPORT"
    # name, which don't exist in generated headers.
    result["transport"] = "udp4" if re.search(r"^[ \t]*#define\s+USE_WIFI\b", text, re.MULTILINE) \
        else "serial"

    m = re.search(r"^[ \t]*#define\s+AGENT_IP\s*\{\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)",
                   text, re.MULTILINE)
    if m:
        result["agent_ip"] = ".".join(m.groups())

    m = re.search(r"^[ \t]*#define\s+AGENT_PORT\s+(\d+)", text, re.MULTILINE)
    if m:
        result["agent_port"] = m.group(1)

    # LiDAR routing. Whether the scan is real or emulated makes no
    # difference to Console -- what it has to know is which route the bytes
    # take, because that decides whether the driver opens a serial device or
    # a socat bridge over a UDP socket:
    #   USE_LIDAR_UDP -> the MCU relays them to LIDAR_PORT, no pin involved
    #   LIDAR_RXD     -> they leave a UART pin for a USB-TTL adapter
    # A serial config missing LIDAR_RXD compiles, runs and silently
    # publishes nothing (firmware.ino calls fake_ld19.begin() with no pin,
    # which opens no UART), so absence of the pin is meaningful, not a
    # default to paper over.
    lidar_rxd = re.search(r"^[ \t]*#define\s+LIDAR_RXD\s+(-?\d+)", text, re.MULTILINE)
    if re.search(r"^[ \t]*#define\s+USE_LIDAR_UDP\b", text, re.MULTILINE):
        result["lidar_transport"] = "udp_bridge"
    elif lidar_rxd:
        result["lidar_transport"] = "serial"
    if lidar_rxd:
        result["lidar_rxd"] = lidar_rxd.group(1)

    m = re.search(r"^[ \t]*#define\s+LIDAR_PORT\s+(\d+)", text, re.MULTILINE)
    if m:
        result["lidar_udp_port"] = m.group(1)

    m = re.search(r"^[ \t]*#define\s+LIDAR_BAUDRATE\s+(\d+)", text, re.MULTILINE)
    if m:
        result["lidar_baud"] = m.group(1)

    result["fake_ld19"] = bool(
        re.search(r"^[ \t]*#define\s+USE_FAKE_LD19\b", text, re.MULTILINE))
    if result["fake_ld19"] and not result.get("lidar_transport"):
        # The one combination that gives no symptom on the board: fake_ld19.begin()
        # with no pin opens no UART, so the firmware runs happily and publishes
        # nothing. Say so at import time -- by the time /scan is missing, the
        # search has usually started on the ROS side.
        result.setdefault("warnings", []).append(
            "USE_FAKE_LD19 is on but the header sets neither USE_LIDAR_UDP nor "
            "LIDAR_RXD, so the firmware opens no UART and publishes no scan. "
            "Pick a route in the config engine and reflash.")

    m = re.search(r"^[ \t]*#define\s+USE_(\w+)_IMU\b", text, re.MULTILINE)
    result["has_imu"] = bool(m and m.group(1) != "FAKE")
    m = re.search(r"^[ \t]*#define\s+USE_(\w+)_MAG\b", text, re.MULTILINE)
    result["has_mag"] = bool(m and m.group(1) not in (None, "FAKE"))

    m = re.search(r"^[ \t]*#define\s+MAG_BIAS\s*\{([^}]*)\}", text, re.MULTILINE)
    if m:
        result["mag_bias"] = [v.strip() for v in m.group(1).split(",")]
    return result


class Handler(BaseHTTPRequestHandler):
    server_version = "Linorobot2Console/0.1"

    def log_message(self, fmt, *args):
        pass  # keep stdout clean; the console pane is the log the user wants

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw)
        except Exception:
            return {}

    def _serve_static(self, path):
        if path == "/":
            path = "/index.html"
        full = os.path.normpath(os.path.join(WEB_DIR, path.lstrip("/")))
        if not full.startswith(WEB_DIR) or not os.path.isfile(full):
            self.send_response(404)
            self.end_headers()
            return
        ctype = "text/html"
        if full.endswith(".js"):
            ctype = "application/javascript"
        elif full.endswith(".css"):
            ctype = "text/css"
        elif full.endswith(".json"):
            ctype = "application/json"
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    # ---- GET ----
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("/api/robot_config", "/api/robot/unified_config"):
            query_params = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            req_distro = query_params.get("distro") or detect_ros_distro()
            req_base = query_params.get("base") or "2wd"
            self._send_json(get_unified_config(distro=req_distro, base=req_base))
            return

        if path == "/api/agent/port_check":
            qs = parse_qs(parsed.query)
            port = qs.get("port", ["/dev/ttyUSB0"])[0]
            mode = qs.get("mode", ["serial"])[0]
            udp_port = int(qs.get("udp_port", [8888])[0])
            host = qs.get("host", [""])[0]
            user = qs.get("user", ["ubuntu"])[0]
            res = check_agent_port_status(port, mode, udp_port, host=host if host else None, user=user)
            self._send_json(res)
            return

        if path == "/api/docker/status":
            self._send_json(check_container_status())
            return

        if path == "/api/docker/rootless_info":
            self._send_json(get_rootless_info())
            return

        if path == "/api/docker/setup_rootless":
            self._send_json(setup_rootless_docker())
            return

        if path == "/api/container/install":
            qs = parse_qs(parsed.query)
            eng = qs.get("engine", ["docker"])[0]
            self._send_json(install_container_engine(eng))
            return

        if path == "/api/autostart/status":
            self._send_json(get_autostart_status())
            return

        if path == "/api/autostart/logs":
            self._send_json(get_autostart_logs())
            return

        if path == "/api/status":
            cfg = load_config()
            distro = detect_ros_distro()
            ws = cfg["workspace_path"]
            git = collect_git_info()
            self._send_json({
                "ros_distro": distro,
                "supported_distros": SUPPORTED_DISTROS,
                "ros2_installed": ros2_installed(distro),
                "workspace_path": ws,
                "workspace_built": workspace_built(ws),
                "host_ip": get_host_ip(),
                "os": sys.platform,
                "agent_busy_console": agent_runner.is_busy(),
                "agent_alive_external": agent_externally_alive(),
                "bringup_busy_console": bringup_runner.is_busy(),
                "bringup_alive_external": bringup_externally_alive(),
                "main_busy": main_runner.is_busy(),
                "laser_busy": laser_runner.is_busy(),
                "gamepad_running": gamepad_runner.is_running(),
                "web_dir": WEB_DIR,
                "robot_name": get_active_robot_name(),
                "robot_config_path": get_robot_config_path(),
                "robots": list_robot_configs(),
                "git_branch": git.get("branch", ""),
                "config": cfg,
            })
            return

        if path == "/api/config":
            self._send_json(load_config())
            return

        if path == "/api/robots":
            self._send_json({
                "robots": list_robot_configs(),
                "active": get_active_robot_name(),
            })
            return

        if path == "/api/gitinfo":
            self._send_json(collect_git_info())
            return

        if path == "/api/bringup/health":
            qs = parse_qs(parsed.query)
            try:
                timeout = min(float(qs.get("timeout", [4.0])[0]), 15.0)
            except ValueError:
                timeout = 4.0
            self._send_json(check_bringup_health(timeout=timeout))
            return

        if path == "/api/sensors":
            self._send_json(sensor_registry())
            return

        if path == "/api/serial_ports":
            self._send_json({"ports": list_serial_ports()})
            return

        if path == "/api/list_dir":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            self._send_json(list_dir(q.get("path", ""), only=q.get("only", "any"),
                                    exts=q.get("exts", "")))
            return

        if path == "/api/nav2_config":
            query_params = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            requested_distro = query_params.get("distro") or detect_ros_distro()
            requested_base = query_params.get("base") or "2wd"
            cfg_path = get_nav2_config_path(requested_distro)
            cfg_text = get_nav2_config(requested_distro)
            self._send_json({
                "distro": requested_distro,
                "base": requested_base,
                "config": cfg_text,
                "path": cfg_path,
                "exists": os.path.exists(cfg_path),
                "supported_distros": SUPPORTED_DISTROS,
                "supported_bases": ["2wd", "4wd", "mecanum"],
                "depth_pointcloud_active": patcher.costmap_depth_active(cfg_text) if patcher else None,
            })
            return

        if path == "/api/ekf_config":
            query_params = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            requested_base = query_params.get("base") or "2wd"
            cfg_path = get_ekf_config_path()
            self._send_json({
                "base": requested_base,
                "config": get_ekf_config(requested_base),
                "path": cfg_path,
                "exists": os.path.exists(cfg_path),
                "supported_bases": ["2wd", "4wd", "mecanum"],
            })
            return

        if path == "/api/slam_config":
            cfg_path = get_slam_config_path()
            self._send_json({
                "config": get_slam_config(),
                "path": cfg_path,
                "exists": os.path.exists(cfg_path),
            })
            return

        if path == "/api/presets":
            presets_data = patcher.PRESETS if patcher else {}
            self._send_json({"presets": presets_data})
            return

        if path == "/api/maps":
            cfg = load_config()
            maps_dir = os.path.join(cfg["workspace_path"], "src", "linorobot2",
                                     "linorobot2_navigation", "maps")
            maps = []
            if os.path.isdir(maps_dir):
                maps = sorted(f[:-5] for f in os.listdir(maps_dir) if f.endswith(".yaml"))
            self._send_json({"maps": maps, "maps_dir": maps_dir})
            return

        if path == "/api/bringup/stream":
            self._handle_bringup_stream()
            return

        if path == "/api/package/check":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            pkg = q.get("pkg", "")
            distro = q.get("distro") or detect_ros_distro()
            ws_path = q.get("ws") or _find_default_workspace()
            self._send_json(get_package_install_info(pkg, distro=distro, ws=ws_path))
            return

        if path == "/api/nav2/stack":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            self._send_json(nav2_stack_status(q.get("distro"), q.get("ws")))
            return

        if path == "/api/ros2/install_cmd":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            distro = q.get("distro") or detect_ros_distro()
            if distro not in SUPPORTED_DISTROS:
                self._send_json({"error": f"unsupported distro '{distro}'"}, 400)
                return
            self._send_json({
                "distro": distro,
                "installed": ros2_installed(distro),
                "command": build_ros2_install_cmd(distro),
            })
            return

        if path == "/api/workspace/build_cmd":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            ws_path = q.get("ws") or _find_default_workspace()
            distro = q.get("distro") or detect_ros_distro()
            self._send_json({"command": build_base_install_cmd(ws_path, distro), "workspace": ws_path})
            return

        if path == "/api/sensors/driver_status":
            q = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            sensor = q.get("sensor", "")
            ws_path = q.get("ws", "")
            self._send_json(get_sensor_driver_status(sensor, ws=ws_path))
            return

        if path == "/api/lidar_stream":
            self._handle_lidar_stream()
            return

        self._serve_static(path)

    def _handle_bringup_stream(self):
        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        def send_sse(event, data_dict):
            payload = json.dumps(data_dict)
            try:
                self.wfile.write(f"event: {event}\ndata: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                return True
            except (BrokenPipeError, ConnectionResetError):
                return False

        if bringup_runner.is_busy():
            history = bringup_runner.get_history()
            if not send_sse("init", {"status": "running", "source": "native", "history_count": len(history)}):
                return
            for line in history:
                if not send_sse("output", {"line": line}):
                    return
            sub_q = queue.Queue(maxsize=1000)
            bringup_runner.subscribe(sub_q)
            try:
                while bringup_runner.is_busy():
                    try:
                        ev_type, payload = sub_q.get(timeout=1.0)
                        if not send_sse(ev_type, payload):
                            break
                        if ev_type == "done":
                            break
                    except queue.Empty:
                        try:
                            self.wfile.write(b": ping\n\n")
                            self.wfile.flush()
                        except Exception:
                            break
            finally:
                bringup_runner.unsubscribe(sub_q)
            return

        container_alive, engine = _find_bringup_container()
        if container_alive:
            if not send_sse("init", {"status": "running", "source": engine, "container": "bringup"}):
                return
            try:
                proc = subprocess.Popen(
                    [engine, "logs", "-f", "--tail", "200", "bringup"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1
                )
                try:
                    for line in iter(proc.stdout.readline, ""):
                        if not line:
                            break
                        if not send_sse("output", {"line": line.rstrip("\n")}):
                            break
                finally:
                    proc.terminate()
                    proc.wait()
            except Exception as e:
                send_sse("output", {"line": f"[console] Error attaching to container logs: {e}"})
            return

        history = bringup_runner.get_history()
        send_sse("idle", {"status": "idle", "history": history[-50:] if history else []})


    def _stream_command(self, command, runner, action_label=None):
        if not command:
            self._send_json({"error": "Empty command"}, 400)
            return
        if runner.is_busy():
            self._send_json({"error": f"{runner.name} slot is already running a command"}, 409)
            return

        # Auto-commit the active robot config on the current branch before the
        # action runs -- git log then records exactly what config each run used.
        # A no-op when the config file is unchanged; never blocks the action.
        commit_hash = ""
        if action_label:
            try:
                commit_hash = commit_robot_config_if_dirty(action_label=action_label)
            except Exception:
                commit_hash = ""

        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        client_gone = {"v": False}

        def send_event(event_type, payload):
            if client_gone["v"]:
                return
            msg = f"event: {event_type}\ndata: {json.dumps(payload)}\n\n"
            try:
                self.wfile.write(msg.encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                client_gone["v"] = True

        if commit_hash:
            send_event("output", {"line": f"[console] auto-committed robot config @ {commit_hash} before {action_label}"})

        # cwd is intentionally NOT the configured workspace_path: that directory
        # may not exist yet (the base-install command's job is to create it),
        # so every generated command `cd`s explicitly where it needs to instead.
        runner.start_streaming(command, cwd=os.path.expanduser("~"), send_event=send_event)

    def _handle_lidar_stream(self):
        """Live /scan viewer rendered in-browser: run `ros2 topic echo /scan`
        and forward just the fields the canvas plot needs as SSE JSON frames.
        Deliberately NOT the LiDAR driver's own bundled `view_*`/`*_view`
        launch file -- that just opens a local RViz window on the robot
        computer's own display, which isn't visible through Console's
        browser UI. Same idea as config-engine's topic-echo SSE consumer --
        no rosbridge/roslibjs dependency."""
        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        cfg = load_config()
        distro = detect_ros_distro()
        cmd = (
            f"source /opt/ros/{distro}/setup.bash 2>/dev/null; "
            f"[ -f {cfg['workspace_path']}/install/setup.bash ] && source {cfg['workspace_path']}/install/setup.bash; "
            "ros2 topic echo --full-length /scan"
        )
        proc = subprocess.Popen(
            ["bash", "-lc", cmd], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, preexec_fn=os.setsid,
        )
        doc = {}
        field_re = re.compile(r"^(\w+):\s*(.*)$")
        in_ranges = False
        ranges_buf = []
        try:
            for line in iter(proc.stdout.readline, ""):
                if not line:
                    break
                stripped = line.rstrip("\n")
                if stripped == "---":
                    if "ranges" not in doc and ranges_buf:
                        doc["ranges"] = ranges_buf
                    try:
                        raw_ranges = doc.get("ranges", [])
                        clean_ranges = []
                        for x in raw_ranges:
                            xs = str(x).strip().strip("'\"")
                            if xs in (".nan", "nan", "inf", "-inf"):
                                clean_ranges.append(0.0)
                            elif xs and xs != "...":
                                try:
                                    v = float(xs)
                                    clean_ranges.append(0.0 if (math.isnan(v) or math.isinf(v)) else v)
                                except ValueError:
                                    pass
                        payload = {
                            "angle_min": float(doc.get("angle_min", 0)),
                            "angle_max": float(doc.get("angle_max", 0)),
                            "angle_increment": float(doc.get("angle_increment", 0)),
                            "ranges": clean_ranges,
                        }
                        msg = f"event: scan\ndata: {json.dumps(payload)}\n\n"
                        self.wfile.write(msg.encode("utf-8"))
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    except Exception:
                        pass
                    doc = {}
                    ranges_buf = []
                    in_ranges = False
                    continue
                m = field_re.match(stripped.strip())
                if m:
                    key, val = m.group(1), m.group(2)
                    if key == "ranges":
                        in_ranges = True
                        inline = val.strip()
                        if inline.startswith("[") and inline.endswith("]"):
                            doc["ranges"] = [v for v in inline[1:-1].split(",") if v.strip()]
                            in_ranges = False
                        continue
                    in_ranges = False
                    doc[key] = val
                elif in_ranges and stripped.strip().startswith("-"):
                    ranges_buf.append(stripped.strip().lstrip("- "))
        finally:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass

    # ---- POST ----
    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        data = self._read_json()

        if path in ("/api/robot_config", "/api/robot/unified_config"):
            distro = data.get("distro") or detect_ros_distro()
            base = data.get("base") or "2wd"
            res = save_unified_config(data, distro=distro, base=base)
            self._send_json(res)
            return

        if path == "/api/config":
            cfg = load_config()
            incoming = {k: v for k, v in data.items() if k in DEFAULT_CONFIG}
            # Persist sensor ports in their stable by-path form.
            for k in ("laser_serial_port", "depth_serial_port"):
                if incoming.get(k):
                    incoming[k] = to_by_path(incoming[k])
            cfg.update(incoming)
            save_config(cfg)
            self._send_json(cfg)
            return

        if path == "/api/workspace/build_cmd":
            ws_path = data.get("workspace_path") or _find_default_workspace()
            distro = data.get("ros_distro") or detect_ros_distro()
            self._send_json({"command": build_base_install_cmd(ws_path, distro), "workspace": ws_path})
            return

        if path == "/api/sensor_install_cmd":
            kind = data.get("kind", "laser")
            key = data.get("key", "")
            cmd = build_sensor_install_cmd(
                kind, key,
                skip_udev=bool(data.get("skip_udev")),
                udev_only=bool(data.get("udev_only")),
                ws=data.get("workspace_path") or load_config()["workspace_path"],
            )
            if cmd is None:
                self._send_json({"error": f"no commands for {kind}:{key}"}, 404)
            else:
                self._send_json({"command": cmd})
            return

        if path == "/api/robot/select":
            name = (data.get("name") or "").strip()
            if not _robot_name_ok(name):
                self._send_json({"error": "invalid robot name (use [a-z0-9_])"}, 400)
                return
            set_active_robot_name(name)
            if not os.path.exists(get_robot_config_path(name)):
                save_config(dict(DEFAULT_CONFIG), robot_name=name)
            self._send_json({
                "status": "ok",
                "active": name,
                "robots": list_robot_configs(),
                "config": load_config(),
                "robot_config_path": get_robot_config_path(name),
            })
            return

        if path == "/api/gitinfo/branch":
            branch = (data.get("branch") or "").strip()
            if not re.match(r"^[A-Za-z0-9._/-]+$", branch or ""):
                self._send_json({"error": "invalid branch name"}, 400)
                return
            qb = shlex.quote(branch)
            cmd = (
                f'cd {shlex.quote(LINOROBOT2_ROOT)} && '
                f'if git show-ref --verify --quiet refs/heads/{qb}; then '
                f'git checkout {qb}; else git checkout -b {qb}; fi'
            )
            self._stream_command(cmd, main_runner, action_label=f"checkout {branch}")
            return

        if path == "/api/exec":
            command = data.get("command", "")
            slot = data.get("slot", "main")
            runner = SLOT_RUNNERS.get(slot, main_runner)
            if slot in SLOT_PROC_PATTERNS:
                # Clear anything a previous Console instance left running before
                # adding another. Duplicates do not conflict loudly -- they just
                # publish the same topics side by side, and everything
                # downstream sees the two sources interleaved.
                stale = reap_stale_slot(slot)
                if stale:
                    runner._broadcast("output", {
                        "line": f"[console] stopped {len(stale)} orphaned bringup "
                                f"process(es) from a previous session: {stale}"})
            self._stream_command(command, runner, action_label=data.get("action") or slot)
            return

        if path == "/api/kill":
            slot = data.get("slot", "main")
            runner = SLOT_RUNNERS.get(slot, main_runner)
            killed = runner.kill()
            self._send_json({"killed": killed})
            return

        if path == "/api/bringup/exec":
            command = data.get("command", "")
            stale = reap_stale_bringup()
            if stale:
                bringup_runner._broadcast("output", {
                    "line": f"[console] stopped {len(stale)} orphaned bringup "
                            f"process(es) from a previous session: {stale}"})
            self._stream_command(command, bringup_runner, action_label=data.get("action") or "bringup")
            return

        if path == "/api/bringup/kill":
            killed = bringup_runner.kill()
            self._send_json({"killed": killed})
            return

        if path == "/api/gamepad/start":
            topic = data.get("topic") or "/cmd_vel"
            started = gamepad_runner.start(topic)
            self._send_json({"started": started, "running": gamepad_runner.is_running()})
            return

        if path == "/api/gamepad/cmd":
            # the stick sends these continuously while held, so keep it cheap:
            # no config reload, no shelling out, just a line down the pipe
            sent = gamepad_runner.send(
                float(data.get("linear_x") or 0.0),
                float(data.get("linear_y") or 0.0),
                float(data.get("angular_z") or 0.0),
            )
            self._send_json({"sent": sent, "running": gamepad_runner.is_running()})
            return

        if path == "/api/gamepad/stall":
            if not gamepad_runner.is_running():
                self._send_json({"stalled": False, "reason": "gamepad not running"})
                return
            self._send_json(check_drive_stalled(gamepad_runner.target))
            return

        if path == "/api/gamepad/kill":
            killed = gamepad_runner.kill()
            self._send_json({"killed": killed})
            return

        if path == "/api/agent/exec":
            command = data.get("command", "")
            self._stream_command(command, agent_runner, action_label=data.get("action") or "agent")
            return

        if path == "/api/agent/port_check":
            port = data.get("port", "/dev/ttyUSB0")
            mode = data.get("mode", "serial")
            udp_port = int(data.get("udp_port", 8888))
            host = data.get("host", "")
            user = data.get("user", "ubuntu")
            res = check_agent_port_status(port, mode, udp_port, host=host if host else None, user=user)
            self._send_json(res)
            return

        if path == "/api/agent/port_release":
            port = data.get("port", "/dev/ttyUSB0")
            mode = data.get("mode", "serial")
            udp_port = int(data.get("udp_port", 8888))
            host = data.get("host", "")
            user = data.get("user", "ubuntu")
            res = release_agent_port(port, mode, udp_port, host=host if host else None, user=user)
            self._send_json(res)
            return

        if path == "/api/docker/status":
            self._send_json(check_container_status())
            return

        if path == "/api/docker/rootless_info":
            self._send_json(get_rootless_info())
            return

        if path == "/api/docker/setup_rootless":
            self._send_json(setup_rootless_docker())
            return

        if path == "/api/container/install":
            qs = parse_qs(parsed.query)
            eng = qs.get("engine", ["docker"])[0]
            self._send_json(install_container_engine(eng))
            return

        if path == "/api/agent/kill":
            killed = agent_runner.kill()
            self._send_json({"killed": killed})
            return

        if path == "/api/nav2_config":
            distro = data.get("distro") or detect_ros_distro()
            cfg_text = data.get("config", "")
            if not cfg_text.strip():
                self._send_json({"error": "Empty configuration"}, 400)
                return
            saved_path = save_nav2_config(cfg_text, distro)
            self._send_json({"status": "ok", "distro": distro, "path": saved_path, "length": len(cfg_text)})
            return

        if path == "/api/nav2_config/costmap_sources":
            # Gate the depth camera in/out of the costmap `observation_sources`
            # to match the robot's selected depth sensor -- avoids a stale
            # observation-buffer warning when there is no camera.
            distro = data.get("distro") or detect_ros_distro()
            depth_enabled = bool(data.get("depth_enabled"))
            if not patcher:
                self._send_json({"error": "patcher unavailable"}, 500)
                return
            new_cfg = patcher.patch_costmap_sources(get_nav2_config(distro), depth_enabled)
            saved_path = save_nav2_config(new_cfg, distro)
            self._send_json({
                "status": "ok",
                "distro": distro,
                "depth_pointcloud_active": patcher.costmap_depth_active(new_cfg),
                "path": saved_path,
                "config": new_cfg,
            })
            return

        if path == "/api/nav2_config/reset":
            distro = data.get("distro") or detect_ros_distro()
            base = data.get("base") or "2wd"
            tpl_path = os.path.join(CONFIG_DIR, f"nav2_{distro}_{base}.yaml")
            if not os.path.exists(tpl_path):
                tpl_path = get_nav2_default_path(distro)
            if os.path.exists(tpl_path):
                with open(tpl_path, "r") as f:
                    reset_content = f.read()
                saved_path = save_nav2_config(reset_content, distro)
                self._send_json({"status": "reset", "distro": distro, "base": base, "config": reset_content, "path": saved_path})
            else:
                self._send_json({"error": f"Default template for {distro} not found"}, 404)
            return

        if path == "/api/nav2_config/patch":
            distro = data.get("distro") or detect_ros_distro()
            base = data.get("base") or "2wd"
            current_cfg = get_nav2_config(distro)
            if patcher:
                patched_cfg = patcher.patch_nav2_text(
                    current_cfg, **_nav2_kwargs(dict(data, base_type=base))
                )
            else:
                patched_cfg = current_cfg
            saved_path = save_nav2_config(patched_cfg, distro)
            self._send_json({"status": "patched", "distro": distro, "base": base, "config": patched_cfg, "path": saved_path})
            return

        if path == "/api/ekf_config":
            cfg_text = data.get("config", "")
            if not cfg_text.strip():
                self._send_json({"error": "Empty EKF configuration"}, 400)
                return
            saved_path = save_ekf_config(cfg_text)
            self._send_json({"status": "ok", "path": saved_path, "length": len(cfg_text)})
            return

        if path == "/api/ekf_config/patch":
            base = data.get("base") or "2wd"
            current_cfg = get_ekf_config(base)
            if patcher:
                patched_cfg = patcher.patch_ekf_text(current_cfg, **_ekf_kwargs(data, base=base))
            else:
                patched_cfg = current_cfg
            saved_path = save_ekf_config(patched_cfg)
            self._send_json({"status": "patched", "base": base, "config": patched_cfg, "path": saved_path})
            return

        if path == "/api/ekf_config/reset":
            base = data.get("base") or "2wd"
            tpl_path = get_ekf_default_path(base)
            if os.path.exists(tpl_path):
                with open(tpl_path, "r") as f:
                    reset_content = f.read()
                saved_path = save_ekf_config(reset_content)
                self._send_json({"status": "reset", "base": base, "config": reset_content, "path": saved_path})
            else:
                self._send_json({"error": "Default EKF template not found"}, 404)
            return

        if path == "/api/slam_config":
            cfg_text = data.get("config", "")
            if not cfg_text.strip():
                self._send_json({"error": "Empty SLAM configuration"}, 400)
                return
            saved_path = save_slam_config(cfg_text)
            self._send_json({"status": "ok", "path": saved_path, "length": len(cfg_text)})
            return

        if path == "/api/slam_config/patch":
            current_cfg = get_slam_config()
            if patcher:
                patched_cfg = patcher.patch_slam_text(
                    current_cfg,
                    resolution=data.get("resolution"),
                    max_laser_range=data.get("max_laser_range"),
                    minimum_travel_distance=data.get("minimum_travel_distance"),
                    minimum_travel_heading=data.get("minimum_travel_heading")
                )
            else:
                patched_cfg = current_cfg
            saved_path = save_slam_config(patched_cfg)
            self._send_json({"status": "patched", "config": patched_cfg, "path": saved_path})
            return

        if path == "/api/slam_config/reset":
            tpl_path = get_slam_default_path()
            if os.path.exists(tpl_path):
                with open(tpl_path, "r") as f:
                    reset_content = f.read()
                saved_path = save_slam_config(reset_content)
                self._send_json({"status": "reset", "config": reset_content, "path": saved_path})
            else:
                self._send_json({"error": "Default SLAM template not found"}, 404)
            return

        if path == "/api/presets/apply":
            preset_name = data.get("preset", "")
            if not patcher or preset_name not in patcher.PRESETS:
                self._send_json({"error": f"Unknown preset: {preset_name}"}, 400)
                return
            pinfo = patcher.PRESETS[preset_name]
            distro = data.get("distro") or detect_ros_distro()
            base = pinfo["base"]

            # 1. Patch Nav2 -- forward every tuning key the preset actually carries
            nav2_in = get_nav2_config(distro)
            nav2_out = patcher.patch_nav2_text(nav2_in, **_nav2_kwargs(pinfo))
            save_nav2_config(nav2_out, distro)

            # 2. Patch EKF
            ekf_in = get_ekf_config(base)
            ekf_out = patcher.patch_ekf_text(
                ekf_in,
                base_type=base,
                frequency=pinfo["ekf_frequency"],
                fuse_vy=pinfo["fuse_vy"],
                fuse_imu_yaw=pinfo["fuse_imu_yaw"],
            )
            save_ekf_config(ekf_out)

            # 3. Patch SLAM
            slam_in = get_slam_config()
            slam_out = patcher.patch_slam_text(
                slam_in,
                resolution=pinfo["slam_resolution"],
                max_laser_range=pinfo["slam_max_range"]
            )
            save_slam_config(slam_out)

            self._send_json({
                "status": "applied",
                "preset": preset_name,
                "label": pinfo["label"],
                "base": base,
                "distro": distro,
                "nav2_config": nav2_out,
                "ekf_config": ekf_out,
                "slam_config": slam_out
            })
            return

        if path == "/api/ai/tune":
            prompt = data.get("prompt", "")
            base = data.get("base", "2wd")
            distro = data.get("distro") or detect_ros_distro()
            analysis = analyze_robotics_ai(prompt, base=base, distro=distro)
            self._send_json(analysis)
            return

        if path == "/api/ai/apply":
            distro = data.get("distro") or detect_ros_distro()
            nav2_p = data.get("nav2_patch") or {}
            ekf_p = data.get("ekf_patch") or {}
            slam_p = data.get("slam_patch") or {}
            # analyze_robotics_ai resolves the real kinematics into target_base;
            # the client echoes it here so a mecanum robot's lateral-velocity
            # fusion is not silently disabled by a "2wd" fallback.
            resolved_base = data.get("base") or nav2_p.get("base") or ekf_p.get("base") or "2wd"

            # Nav2 -- apply only the keys the analysis actually set; everything
            # else in the tuned config is left untouched.
            if nav2_p and patcher:
                cur_nav2 = get_nav2_config(distro)
                patched_nav2 = patcher.patch_nav2_text(
                    cur_nav2, **_nav2_kwargs(dict(nav2_p, base_type=resolved_base))
                )
                save_nav2_config(patched_nav2, distro)

            # EKF
            if ekf_p and patcher:
                cur_ekf = get_ekf_config(resolved_base)
                patched_ekf = patcher.patch_ekf_text(cur_ekf, **_ekf_kwargs(ekf_p, base=resolved_base))
                save_ekf_config(patched_ekf)

            # SLAM
            if slam_p and patcher:
                cur_slam = get_slam_config()
                patched_slam = patcher.patch_slam_text(
                    cur_slam,
                    resolution=slam_p.get("resolution"),
                    max_laser_range=slam_p.get("max_laser_range"),
                    minimum_travel_distance=slam_p.get("minimum_travel_distance"),
                    minimum_travel_heading=slam_p.get("minimum_travel_heading")
                )
                save_slam_config(patched_slam)

            self._send_json({
                "status": "ai_applied",
                "distro": distro,
                "nav2_updated": bool(nav2_p),
                "ekf_updated": bool(ekf_p),
                "slam_updated": bool(slam_p)
            })
            return

        if path == "/api/ai/robot_builder":
            description = data.get("description", "")
            specs = generate_custom_robot_specs(description)
            self._send_json(specs)
            return

        if path == "/api/ai/deploy_robot":
            specs = data.get("specs") or {}
            distro = data.get("distro") or detect_ros_distro()
            # generate_custom_robot_specs() returns {design, tuning:{nav2,ekf,slam}, workflow};
            # accept that nested shape or a pre-flattened one.
            tuning = specs.get("tuning") or specs
            design = specs.get("design") or {}
            nav2_cfg = tuning.get("nav2") or {}
            ekf_cfg = tuning.get("ekf") or {}
            slam_cfg = tuning.get("slam") or {}
            base = (design.get("base_type") or nav2_cfg.get("base")
                    or ekf_cfg.get("base") or specs.get("base") or "2wd")
            laser_sensor = design.get("laser_sensor") or specs.get("laser_sensor")

            # 1. Update EKF
            if ekf_cfg and patcher:
                cur_ekf = get_ekf_config(base)
                patched_ekf = patcher.patch_ekf_text(cur_ekf, **_ekf_kwargs(ekf_cfg, base=base))
                save_ekf_config(patched_ekf)

            # 2. Update Nav2
            if nav2_cfg and patcher:
                cur_nav2 = get_nav2_config(distro)
                patched_nav2 = patcher.patch_nav2_text(
                    cur_nav2, **_nav2_kwargs(dict(nav2_cfg, base_type=base))
                )
                save_nav2_config(patched_nav2, distro)

            # 3. Update SLAM
            if slam_cfg and patcher:
                cur_slam = get_slam_config()
                patched_slam = patcher.patch_slam_text(
                    cur_slam,
                    resolution=slam_cfg.get("resolution"),
                    max_laser_range=slam_cfg.get("max_laser_range"),
                    minimum_travel_distance=slam_cfg.get("minimum_travel_distance"),
                    minimum_travel_heading=slam_cfg.get("minimum_travel_heading"),
                )
                save_slam_config(patched_slam)

            # 4. Update console_config.json
            cfg = load_config()
            cfg["base_type"] = base
            if laser_sensor:
                cfg["laser_sensor"] = laser_sensor
            save_config(cfg)

            self._send_json({
                "status": "deployed",
                "base": base,
                "distro": distro,
                "laser_sensor": laser_sensor,
                "nav2_updated": bool(nav2_cfg),
                "ekf_updated": bool(ekf_cfg),
                "slam_updated": bool(slam_cfg),
                "message": f"Successfully configured and deployed custom {base.upper()} robot!"
            })
            return

        if path == "/api/import_config":
            self._handle_import_config(data)
            return

        if path == "/api/params/export":
            dest = (data.get("dest_dir") or "").strip()
            if not dest:
                self._send_json({"error": "dest_dir required"}, 400)
                return
            try:
                result = export_params_bundle(
                    dest,
                    distros=data.get("distros") or SUPPORTED_DISTROS[:3],
                    base=data.get("base") or "2wd",
                    depth_costmap=data.get("depth_costmap") or "auto",
                )
            except OSError as e:
                self._send_json({"error": f"export failed: {e}"}, 400)
                return
            self._send_json({"status": "exported", **result})
            return

        if path == "/api/params/merge":
            if not yaml_merge:
                self._send_json({"error": "yaml_merge unavailable"}, 500)
                return
            kind = data.get("kind", "nav2")
            distro = data.get("distro") or detect_ros_distro()
            base = data.get("base") or "2wd"
            target = data.get("target", "template")
            target_path = _resolve_target_path(kind, target, distro, base)
            if not target_path:
                self._send_json({"error": f"bad target: {target}"}, 400)
                return
            # source = explicit text (imported file), else the current active config
            source_text = data.get("source_text")
            if source_text is None:
                source_text = _read_params(kind, distro, base)
            try:
                with open(target_path) as f:
                    target_text = f.read()
            except OSError:
                # target file doesn't exist yet -> the merge is just the source
                target_text = source_text
            merged, report = yaml_merge.merge_yaml(target_text, source_text)
            if not data.get("dry_run"):
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with open(target_path, "w") as f:
                    f.write(merged)
            self._send_json({
                "status": "dry-run" if data.get("dry_run") else "merged",
                "kind": kind, "target": target, "target_path": target_path,
                "report": report, "config": merged,
            })
            return

        if path == "/api/params/promote":
            kind = data.get("kind", "nav2")
            distro = data.get("distro") or detect_ros_distro()
            base = data.get("base") or "2wd"
            direction = data.get("direction", "active_to_template")
            active, template, package = _params_paths(kind, distro, base)
            routes = {
                "active_to_template": (active, template),
                "template_to_active": (template, active),
                "active_to_package": (active, package),
                "package_to_active": (package, active),
            }
            if direction not in routes:
                self._send_json({"error": f"bad direction: {direction}"}, 400)
                return
            src_path, dst_path = routes[direction]
            if not os.path.exists(src_path):
                # active may be unwritten -> fall back to effective text
                if src_path == active:
                    text = _read_params(kind, distro, base)
                else:
                    self._send_json({"error": f"source not found: {src_path}"}, 404)
                    return
            else:
                with open(src_path) as f:
                    text = f.read()
            os.makedirs(os.path.dirname(dst_path), exist_ok=True)
            with open(dst_path, "w") as f:
                f.write(text if text.endswith("\n") else text + "\n")
            self._send_json({"status": "promoted", "kind": kind, "direction": direction,
                             "from": src_path, "to": dst_path, "bytes": len(text)})
            return

        if path == "/api/autostart/enable":
            self._send_json(enable_autostart(data))
            return

        if path == "/api/autostart/disable":
            self._send_json(disable_autostart())
            return

        self._send_json({"error": "Not found"}, 404)

    def _handle_import_config(self, data):
        """Read unified YAML, robot.env, or config/custom/<name>_config.h and
        apply to Console's configuration."""
        path = data.get("path", "")
        text = data.get("text", "")
        if path and not text:
            try:
                with open(path) as f:
                    text = f.read()
            except Exception as e:
                self._send_json({"error": f"Could not read {path}: {e}"}, 400)
                return
        if not text:
            self._send_json({"error": "No path or text provided"}, 400)
            return

        # 1. Check for Unified robot_config.yaml
        #
        # The `console:` test is not redundant: the file Console itself writes
        # to config/<robot>_config.yaml has a top-level `console:` key and no
        # `linorobot2:` key at all, so importing Console's own robot_config.yaml
        # used to fall through to the <name>_config.h regex branch below, match
        # none of its #define patterns, and silently save nothing -- while the
        # Settings tab invites exactly that file.
        if ("linorobot2:" in text or re.search(r"^console:", text, re.MULTILINE)
                or ("amcl:" in text and "controller_server:" in text)):
            res = save_unified_config(text)
            self._send_json({"type": "robot_config", **res})
            return


        result = parse_robot_config_header(text)
        saved = persist_imported_config(result)
        if saved:
            result["saved"] = saved

        self._send_json(result)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
    try:
        migrated = migrate_legacy_config()
        if migrated:
            print(f"[migrate] seeded {migrated['path']} (robot '{migrated['robot']}')")
    except Exception as e:
        print(f"[migrate] skipped: {e}")
    if not os.path.exists(get_robot_config_path()):
        save_config(dict(DEFAULT_CONFIG))
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"Linorobot2 Console serving on http://0.0.0.0:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

# Backwards-compatible alias
ConsoleHandler = Handler

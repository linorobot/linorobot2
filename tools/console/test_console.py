import re
import socketserver
import threading
import urllib.request
#!/usr/bin/env python3
# Copyright (c) 2026 Linorobot contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import os
import sys
import tempfile
import shutil
import subprocess
import unittest

try:
    import yaml  # optional: only used to assert extracted sections still parse
except ImportError:
    yaml = None
from urllib.request import urlopen, Request
from urllib.error import HTTPError

# Import server module from web/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "web"))
import server

class TestLinorobot2Console(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp()
        self.orig_config_path = server.CONFIG_PATH
        self.orig_nav2_config_path = server.NAV2_CONFIG_PATH
        self.orig_robot_configs_dir = server.ROBOT_CONFIGS_DIR
        self.orig_active_robot_file = server.ACTIVE_ROBOT_FILE
        self.orig_legacy_yaml = server.LEGACY_ROBOT_CONFIG_YAML_PATH
        server.CONFIG_PATH = os.path.join(self.temp_dir, "test_console_config.json")
        server.NAV2_CONFIG_PATH = os.path.join(self.temp_dir, "test_nav2_params.yaml")
        # Repo-based robot config -> redirect into the temp dir so tests never
        # touch the real <repo>/config/ tree.
        server.ROBOT_CONFIGS_DIR = os.path.join(self.temp_dir, "config")
        server.ACTIVE_ROBOT_FILE = os.path.join(server.ROBOT_CONFIGS_DIR, ".active_robot")
        server.LEGACY_ROBOT_CONFIG_YAML_PATH = os.path.join(self.temp_dir, "legacy_robot_config.yaml")
        os.makedirs(server.ROBOT_CONFIGS_DIR, exist_ok=True)

    def tearDown(self):
        server.CONFIG_PATH = self.orig_config_path
        server.NAV2_CONFIG_PATH = self.orig_nav2_config_path
        server.ROBOT_CONFIGS_DIR = self.orig_robot_configs_dir
        server.ACTIVE_ROBOT_FILE = self.orig_active_robot_file
        server.LEGACY_ROBOT_CONFIG_YAML_PATH = self.orig_legacy_yaml
        if os.path.exists(self.temp_dir):
            for root, dirs, files in os.walk(self.temp_dir, topdown=False):
                for f in files:
                    os.remove(os.path.join(root, f))
                for d in dirs:
                    os.rmdir(os.path.join(root, d))
            os.rmdir(self.temp_dir)

    def test_default_config(self):
        cfg = server.load_config()
        self.assertEqual(cfg["agent_transport"], "serial")
        self.assertEqual(cfg["agent_device"], "/dev/ttyACM0")
        self.assertEqual(cfg["agent_port"], "8888")
        self.assertEqual(cfg["agent_baud"], "921600")
        # Shipped blank on purpose: a pinned distro here would override
        # detect_ros_distro() on a fresh clone and put Ubuntu 26.04 on Jazzy,
        # which has no native packages for it.
        self.assertEqual(cfg.get("ros_distro"), "")
        self.assertIn(server.detect_ros_distro(), server.SUPPORTED_DISTROS)
        self.assertTrue(cfg.get("auto_bringup"))
        self.assertIn("workspace_path", cfg)

    def test_bringup_runner_and_distros(self):
        self.assertIsNotNone(server.bringup_runner)
        self.assertFalse(server.bringup_runner.is_busy())
        self.assertEqual(server.SUPPORTED_DISTROS, ["jazzy", "lyrical", "rolling"])
        self.assertNotIn("humble", server.SUPPORTED_DISTROS)  # dropped upstream

    def test_nav2_config_endpoints(self):
        for distro in server.SUPPORTED_DISTROS:
            cfg = server.get_nav2_config(distro)
            self.assertTrue(len(cfg) > 0, f"Empty config for {distro}")
            self.assertIn("ros__parameters", cfg, f"ros__parameters not in {distro} config")
            self.assertIn("behavior_server", cfg)

        orig_jazzy = server.get_nav2_config("jazzy")
        test_content = "# custom test nav2 parameters\nros__parameters:\n  footprint: '[[0.25, 0.25], [-0.25, 0.25]]'\n"
        path = server.save_nav2_config(test_content, "jazzy")
        self.assertTrue(os.path.exists(path))
        self.assertEqual(server.get_nav2_config("jazzy"), test_content)
        # restore original
        server.save_nav2_config(orig_jazzy, "jazzy")

    def test_save_and_load_config(self):
        new_cfg = {
            "workspace_path": "/tmp/custom_ws",
            "agent_transport": "udp4",
            "agent_device": "/dev/ttyUSB0",
            "agent_port": "9999",
            "agent_baud": "115200"
        }
        server.save_config(new_cfg)
        loaded = server.load_config()
        self.assertEqual(loaded["workspace_path"], "/tmp/custom_ws")
        self.assertEqual(loaded["agent_transport"], "udp4")
        self.assertEqual(loaded["agent_port"], "9999")
        self.assertEqual(loaded["agent_baud"], "115200")

    def test_laser_sensors_definitions(self):
        expected_keys = ["ydlidar", "xv11", "ldlidar", "sllidar"]
        for k in expected_keys:
            self.assertIn(k, server.LASER_SENSORS)
            sensor = server.LASER_SENSORS[k]
            self.assertIn("label", sensor)
            self.assertIn("install", sensor)
            self.assertIsInstance(sensor["install"], list)
            self.assertTrue(len(sensor["install"]) > 0)
            if sensor["udev"] is not None:
                self.assertIsInstance(sensor["udev"], list)

    def test_depth_sensors_definitions(self):
        expected_keys = ["realsense", "oakd", "astra"]
        for k in expected_keys:
            self.assertIn(k, server.DEPTH_SENSORS)
            sensor = server.DEPTH_SENSORS[k]
            self.assertIn("label", sensor)
            self.assertIn("install", sensor)
            self.assertIsInstance(sensor["install"], list)
            self.assertTrue(len(sensor["install"]) > 0)
            if sensor["udev"] is not None:
                self.assertIsInstance(sensor["udev"], list)

    def test_sensor_registry_is_single_source(self):
        """/api/sensors payload carries everything the frontend needs -- no client-side copies."""
        reg = server.sensor_registry()
        self.assertEqual(set(reg["laser"]), set(server.LASER_SENSORS))
        ld = reg["laser"]["ldlidar"]
        self.assertTrue(ld["serial"])
        self.assertEqual(ld["symlink"], "/dev/ldlidar")
        self.assertEqual(ld["docker_key"], "ldlidar")
        self.assertEqual([m["code"] for m in ld["models"]], ["ld06", "ld19", "stl27l"])
        self.assertTrue(all("product" in m for m in ld["models"]))
        # sllidar's one install covers seven bringup model codes
        self.assertEqual(len(reg["laser"]["sllidar"]["models"]), 7)
        # depth cameras are not serial-port devices
        self.assertFalse(reg["depth"]["realsense"]["serial"])

    def test_build_sensor_install_cmd(self):
        c = server.build_sensor_install_cmd("laser", "ldlidar", skip_udev=True, ws="/w")
        self.assertEqual(
            c,
            "cd /w && [ -d src/ldlidar_stl_ros2 ] || git clone "
            "https://github.com/hippo5329/ldlidar_stl_ros2.git src/ldlidar_stl_ros2 "
            "&& colcon build",
        )
        # udev appended when not skipped
        self.assertIn("udevadm control", server.build_sensor_install_cmd("laser", "ldlidar", ws="/w"))
        # udev_only drops the build steps
        only = server.build_sensor_install_cmd("laser", "sllidar", udev_only=True, ws="/w")
        self.assertIn("rplidar.rules", only)
        self.assertNotIn("colcon build", only)
        # {ws} substitution
        self.assertIn("/w/src/sllidar_ros2", only)
        # unknown / no-command sensor
        self.assertIsNone(server.build_sensor_install_cmd("depth", "zed"))
        self.assertIsNone(server.build_sensor_install_cmd("laser", "nope"))

    def test_to_by_path_is_idempotent_and_safe(self):
        # an already-stable path is returned unchanged
        p = "/dev/serial/by-path/pci-0000:00-usb-0:1:1.0-port0"
        self.assertEqual(server.to_by_path(p), p)
        self.assertEqual(server.to_by_path("/dev/serial/by-id/usb-Foo-if00"), "/dev/serial/by-id/usb-Foo-if00")
        # a device with no by-path mapping falls back to itself
        self.assertEqual(server.to_by_path("/dev/nonexistent-tty"), "/dev/nonexistent-tty")
        self.assertEqual(server.to_by_path(""), "")

    def test_list_serial_ports_shape(self):
        ports = server.list_serial_ports()
        self.assertIsInstance(ports, list)
        for p in ports:
            for k in ("preferred", "by_path", "by_id", "tty", "usb_id", "vendor", "model", "serial"):
                self.assertIn(k, p)
            self.assertTrue(p["preferred"])
            self.assertTrue(p["tty"].startswith("/dev/tty"))

    def test_costmap_depth_source_gating(self):
        import patcher
        sample = (
            "local_costmap:\n  local_costmap:\n    ros__parameters:\n"
            "      voxel_layer:\n        observation_sources: scan pointcloud\n"
            "        scan:\n          topic: /scan\n"
            "        pointcloud:\n          topic: /camera/depth/color/points\n"
            "collision_monitor:\n  ros__parameters:\n"
            "    observation_sources: [\"scan\"]\n"
        )
        self.assertTrue(patcher.costmap_depth_active(sample))
        off = patcher.patch_costmap_sources(sample, depth_enabled=False)
        self.assertIn("observation_sources: scan\n", off)
        self.assertNotIn("scan pointcloud", off)
        self.assertFalse(patcher.costmap_depth_active(off))
        # collision_monitor's list form is never touched
        self.assertIn('observation_sources: ["scan"]', off)
        # round-trips back on
        on = patcher.patch_costmap_sources(off, depth_enabled=True)
        self.assertEqual(on, sample)
        # idempotent
        self.assertEqual(patcher.patch_costmap_sources(on, depth_enabled=True), sample)

    def test_yaml_merge_preserves_comments_and_paths(self):
        import yaml_merge
        tgt = ("a:\n  ros__parameters:\n    max_vel_x: 0.5   # cruise\n"
               "    max_vel_theta: 2.5\n"
               "l:\n  l:\n    ros__parameters:\n      voxel_layer:\n"
               "        scan:\n          topic: /scan\n"
               "        pointcloud:\n          topic: /camera/depth/color/points\n")
        src = (tgt.replace("0.5   # cruise", "0.8")
                  .replace("topic: /scan\n", "topic: /scan_filtered\n")
                  .replace("    max_vel_theta: 2.5\n", "    max_vel_theta: 2.5\n    extra_key: 1\n"))
        merged, rep = yaml_merge.merge_yaml(tgt, src)
        self.assertIn("max_vel_x: 0.8   # cruise", merged)      # comment kept, value changed
        self.assertIn("topic: /scan_filtered", merged)
        self.assertIn("topic: /camera/depth/color/points", merged)  # sibling path untouched
        self.assertIn("a/ros__parameters/max_vel_x", rep["changed"])
        self.assertIn("l/l/ros__parameters/voxel_layer/scan/topic", rep["changed"])
        self.assertEqual(rep["source_only"], ["a/ros__parameters/extra_key"])  # reported, not written
        self.assertNotIn("extra_key", merged)
        self.assertEqual(yaml_merge.merge_yaml(merged, src)[0], merged)  # idempotent

    def test_params_export_bundle(self):
        import tempfile, importlib
        srv = importlib.import_module("server")
        with tempfile.TemporaryDirectory() as d:
            res = srv.export_params_bundle(d, distros=["jazzy", "lyrical", "rolling"], base="2wd")
            names = sorted(os.path.basename(f["path"]) for f in res["files"])
            self.assertIn("navigation_jazzy.yaml", names)
            self.assertIn("navigation_rolling.yaml", names)
            self.assertIn("ekf_mecanum.yaml", names)
            self.assertIn("slam.yaml", names)
            self.assertIn("nav2.launch.py", names)
            self.assertIn("README.md", names)
            launch = os.path.join(d, "launch", "nav2.launch.py")
            with open(launch) as fh:
                src = fh.read()
            self.assertIn('BUNDLE = os.path.dirname(os.path.dirname', src)   # config is a sibling of launch/
            self.assertIn('"distro": "jazzy"', src)
            compile(src, launch, "exec")   # exported launcher is valid python

    def test_params_path_resolution(self):
        import importlib
        srv = importlib.import_module("server")
        active, tpl, pkg = srv._params_paths("nav2", "jazzy")
        self.assertTrue(active.endswith("console_nav2_jazzy.yaml"))
        self.assertTrue(tpl.endswith("config/nav2_jazzy.yaml"))
        self.assertTrue(pkg.endswith("linorobot2_navigation/config/navigation_jazzy.yaml"))
        a2, t2, p2 = srv._params_paths("ekf", base="mecanum")
        self.assertTrue(t2.endswith("ekf_mecanum.yaml"))
        self.assertTrue(srv._params_paths("slam")[1].endswith("config/slam.yaml"))

    def test_list_dir_for_path_pickers(self):
        import importlib
        srv = importlib.import_module("server")
        here = os.path.dirname(os.path.abspath(__file__))

        d = srv.list_dir(here, only="dir")
        self.assertEqual(d["path"], here)
        self.assertTrue(d["parent"])
        self.assertTrue(all(e["is_dir"] for e in d["entries"]))
        self.assertIn("config", [e["name"] for e in d["entries"]])
        self.assertTrue(all(not e["name"].startswith(".") for e in d["entries"]))  # hidden dropped

        cfg = srv.list_dir(os.path.join(here, "config"), only="file", exts="yaml")
        files = [e["name"] for e in cfg["entries"] if not e["is_dir"]]
        self.assertIn("nav2_jazzy.yaml", files)
        self.assertTrue(all(f.endswith(".yaml") for f in files))
        # dirs still shown in file mode so you can navigate
        self.assertTrue(any(e["is_dir"] for e in cfg["entries"]) or not any(
            os.path.isdir(os.path.join(here, "config", n)) for n in os.listdir(os.path.join(here, "config"))))

        # blank / bad paths never raise; fall back to $HOME
        home = os.path.expanduser("~")
        self.assertEqual(srv.list_dir("")["path"], home)
        self.assertEqual(srv.list_dir("/no/such/dir/anywhere")["path"], home)
        self.assertEqual(srv.list_dir("~")["path"], home)

    def test_console_owns_its_docker_compose(self):
        """The console ships its own compose stack and never edits the repo's docker/."""
        d = os.path.join(os.path.dirname(__file__), "docker")
        with open(os.path.join(d, "docker-compose.yaml")) as fh:
            compose = fh.read()
        # nav/SLAM go through the console's own launchers, not linorobot2_navigation's
        self.assertIn("tools/console/launch_nav2.py", compose)
        self.assertIn("tools/console/launch_bringup.py", compose)
        self.assertNotIn("linorobot2_navigation slam.launch.py", compose)
        self.assertNotIn("linorobot2_navigation navigation.launch.py", compose)
        # generated files are gitignored, not the checked-in compose
        with open(os.path.join(d, ".gitignore")) as fh:
            ign = fh.read()
        self.assertIn(".env", ign)
        self.assertIn("devices.generated.yaml", ign)
        # upstream nav config left untouched (no console pointcloud edit)
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        with open(os.path.join(root, "linorobot2_navigation", "config", "navigation.yaml")) as fh:
            self.assertNotIn("scan pointcloud", fh.read())
        # containers read the repo-based per-robot config, not the legacy
        # ~/.config path baked into the image
        self.assertIn("config/${ROBOT_NAME:-linorobot2}_config.yaml", compose)
        self.assertIn("../config:/home/ros/linorobot2_ws/src/linorobot2/tools/console/config:rw", compose)
        self.assertNotIn(".config/linorobot2/robot_config.yaml", compose)

    def test_depth_costmap_gate_is_console_only(self):
        """The depth->costmap gate lives in the console's launch_nav2.py; upstream navigation.launch.py is untouched."""
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        with open(os.path.join(root, "linorobot2_navigation", "launch", "navigation.launch.py")) as fh:
            nav_src = fh.read()
        self.assertNotIn("depth_costmap", nav_src)
        self.assertNotIn("LINOROBOT2_DEPTH_SENSOR", nav_src)

        with open(os.path.join(os.path.dirname(__file__), "launch_nav2.py")) as fh:
            cons_src = fh.read()
        self.assertIn("name='depth_costmap'", cons_src)
        self.assertIn("LINOROBOT2_DEPTH_SENSOR", cons_src)
        self.assertIn("scan[ \\t]+pointcloud", cons_src)   # same gate regex as patcher
        self.assertIn("tempfile.NamedTemporaryFile", cons_src)
        # the arg is consumed here, not forwarded to navigation.launch.py
        self.assertNotIn("'depth_costmap':", cons_src)

    def test_shipped_nav_templates_have_gated_pointcloud(self):
        """Templates ship the depth pointcloud source + block for every supported distro."""
        cfg_dir = os.path.join(os.path.dirname(__file__), "config")
        for distro in ("jazzy", "lyrical", "rolling"):
            for suffix in ("", "_mecanum"):
                path = os.path.join(cfg_dir, f"nav2_{distro}{suffix}.yaml")
                with open(path) as fh:
                    text = fh.read()
                self.assertEqual(text.count("observation_sources: scan pointcloud"), 2, path)
                self.assertIn("topic: /camera/depth/color/points", text)
                self.assertIn('data_type: "PointCloud2"', text)
                # collision_monitor stays lidar-only
                self.assertIn('observation_sources: ["scan"]', text)

    def test_upstream_github_issues_diagnostics(self):
        """Test AI diagnosis and patches for common upstream GitHub issues (#113, #37, #76, #12)."""
        from server import analyze_robotics_ai

        # Upstream #113: Map continuously rotating during SLAM
        diag_113 = analyze_robotics_ai("Map continuously rotating with RPLidar A1 and vibrating IMU during SLAM")
        self.assertIn("Upstream #113/#67", diag_113["diagnosis"])
        self.assertFalse(diag_113["ekf_patch"]["fuse_imu_yaw"])
        self.assertEqual(diag_113["ekf_patch"]["frequency"], 50.0)
        self.assertEqual(diag_113["slam_patch"]["minimum_travel_heading"], 0.25)

        # Upstream #37: Obstacles can't clear in local costmap even after they move out
        diag_37 = analyze_robotics_ai("Obstacles cant clear in local costmap even after they move out ghost obstacle")
        self.assertIn("Upstream #37", diag_37["diagnosis"])
        self.assertEqual(diag_37["nav2_patch"]["raytrace_range"], 3.5)
        self.assertEqual(diag_37["nav2_patch"]["obstacle_max_range"], 3.0)

        # Upstream #76: Large robot jerks and runs slowly / fierce vibration
        diag_76 = analyze_robotics_ai("Large robot jerks and runs slowly with fierce vibration 50 kg")
        self.assertIn("Upstream #76", diag_76["diagnosis"])
        self.assertEqual(diag_76["nav2_patch"]["max_accel_x"], 1.0)
        self.assertEqual(diag_76["nav2_patch"]["max_decel_x"], 1.5)
        self.assertEqual(diag_76["nav2_patch"]["max_vel_theta"], 1.2)

    def test_ai_tune_rotation_modes(self):
        """Test AI diagnosis and patching for rotation, drift, overshoot, and unreachable destination."""
        from server import analyze_robotics_ai
        import patcher

        # 1. Rotation and spin stability
        diag_rot = analyze_robotics_ai("robot experiences rotational oscillation and spin slip during in-place turns", base="2wd")
        self.assertIn("Rotational instability or slip detected", diag_rot["diagnosis"])
        self.assertEqual(diag_rot["nav2_patch"]["rotate_to_heading_angular_vel"], 1.5)
        self.assertEqual(diag_rot["nav2_patch"]["angular_dist_threshold"], 0.785)
        self.assertEqual(diag_rot["nav2_patch"]["max_accel_theta"], 2.0)
        self.assertEqual(diag_rot["nav2_patch"]["yaw_goal_tolerance"], 0.12)
        self.assertFalse(diag_rot["ekf_patch"]["fuse_vy"])

        # 2. Drift
        diag_drift = analyze_robotics_ai("robot drifts sideways during in-place rotation", base="2wd")
        self.assertIn("State estimation drift detected", diag_drift["diagnosis"])
        self.assertFalse(diag_drift["ekf_patch"]["fuse_vy"])
        self.assertEqual(diag_drift["ekf_patch"]["frequency"], 50.0)

        # 3. Overshoot
        diag_over = analyze_robotics_ai("robot overshoots destination and blows past goal due to late braking", base="2wd")
        self.assertIn("Goal overshoot and late braking detected", diag_over["diagnosis"])
        self.assertEqual(diag_over["nav2_patch"]["max_decel_x"], 2.8)
        self.assertEqual(diag_over["nav2_patch"]["approach_velocity_scaling_dist"], 0.75)

        # 4. Unable to reach destination
        diag_reach = analyze_robotics_ai("robot unable to reach nav dest and times out near goal", base="2wd")
        self.assertIn("Robot unable to complete navigation to destination", diag_reach["diagnosis"])
        self.assertEqual(diag_reach["nav2_patch"]["xy_goal_tolerance"], 0.08)
        self.assertEqual(diag_reach["nav2_patch"]["movement_time_allowance"], 15.0)
        self.assertEqual(diag_reach["nav2_patch"]["inflation_radius"], 0.52)

    def test_patcher_all_presets(self):
        """Test that all presets including anti_drift, anti_overshoot, destination_guarantee, and smooth_rotation patch cleanly."""
        import patcher
        for p_name in ["standard_diff", "mecanum_omni", "cautious_indoor", "fast_open_space",
                       "anti_drift", "anti_overshoot", "destination_guarantee", "smooth_rotation"]:
            self.assertIn(p_name, patcher.PRESETS)

    # --- Sample Nav2 params, trimmed to the keys the patcher touches ---------
    NAV2_SAMPLE = (
        'amcl:\n  ros__parameters:\n'
        '    robot_model_type: "nav2_amcl::DifferentialMotionModel"\n'
        'controller_server:\n  ros__parameters:\n'
        '    min_y_velocity_threshold: 0.5\n'
        '    progress_checker:\n'
        '      required_movement_radius: 0.5\n'
        '      movement_time_allowance: 10.0\n'
        '    general_goal_checker:\n'
        '      xy_goal_tolerance: 0.35\n'
        '      yaw_goal_tolerance: 0.35\n'
        '    FollowPath:\n'
        '      angular_dist_threshold: 0.785\n'
        '      desired_linear_vel: 0.4  # cruise\n'
        '      lookahead_dist: 0.6\n'
        '      approach_velocity_scaling_dist: 0.6\n'
        '      max_vel_x: 0.5\n'
        '      acc_lim_x: 2.5\n'
        '      decel_lim_x: -2.5\n'
        'velocity_smoother:\n  ros__parameters:\n'
        '    max_velocity: [0.8, 0.0, 2.5]\n'
        '    min_velocity: [-0.8, 0.0, -2.5]\n'
        '    max_accel: [2.5, 0.0, 3.2]\n'
        '    max_decel: [-2.5, 0.0, -3.2]\n'
        'local_costmap:\n  local_costmap:\n    ros__parameters:\n'
        '      inflation_radius: 0.70\n'
        '      cost_scaling_factor: 3.0\n'
        '      raytrace_range: 3.0\n'
        '      obstacle_max_range: 2.5\n'
    )

    def test_patch_nav2_is_opt_in(self):
        """A targeted patch must not reset params it was not given (regression)."""
        import patcher
        # Only ask for a costmap-clearing fix (upstream #37 shape).
        out = patcher.patch_nav2_text(
            self.NAV2_SAMPLE,
            raytrace_range=3.5, obstacle_max_range=3.0, inflation_radius=0.55,
        )
        self.assertIn("raytrace_range: 3.5", out)
        self.assertIn("obstacle_max_range: 3.0", out)
        self.assertIn("inflation_radius: 0.55", out)
        # The tuned 0.8 m/s top speed and the AMCL model survive untouched.
        self.assertIn("max_velocity: [0.8, 0.0, 2.5]", out)
        self.assertIn("max_vel_x: 0.5", out)
        self.assertIn('robot_model_type: "nav2_amcl::DifferentialMotionModel"', out)
        self.assertIn("desired_linear_vel: 0.4  # cruise", out)

    def test_patch_nav2_applies_goal_and_rpp_params(self):
        """The goal-checker / RPP / progress-checker params must actually reach the YAML."""
        import patcher
        out = patcher.patch_nav2_text(
            self.NAV2_SAMPLE,
            xy_goal_tolerance=0.08, yaw_goal_tolerance=0.12,
            lookahead_dist=0.45, approach_velocity_scaling_dist=0.75,
            movement_time_allowance=15.0, required_movement_radius=0.15,
            angular_dist_threshold=0.6,
        )
        self.assertIn("xy_goal_tolerance: 0.08", out)
        self.assertIn("yaw_goal_tolerance: 0.12", out)
        self.assertIn("lookahead_dist: 0.45", out)
        self.assertIn("approach_velocity_scaling_dist: 0.75", out)
        self.assertIn("movement_time_allowance: 15.0", out)
        self.assertIn("required_movement_radius: 0.15", out)
        self.assertIn("angular_dist_threshold: 0.6", out)
        # Untouched velocity block stays put.
        self.assertIn("max_velocity: [0.8, 0.0, 2.5]", out)

    def test_patch_nav2_partial_velocity_reads_existing(self):
        """Supplying only max_vel_x keeps the file's y / theta components."""
        import patcher
        out = patcher.patch_nav2_text(self.NAV2_SAMPLE, max_vel_x=0.35)
        self.assertIn("max_velocity: [0.35, 0.0, 2.5]", out)
        self.assertIn("min_velocity: [-0.35, 0.0, -2.5]", out)

    def test_patch_ekf_is_opt_in(self):
        import patcher
        raw = (
            "ekf_filter_node:\n    ros__parameters:\n"
            "        frequency: 30.0\n        two_d_mode: true\n"
            "        odom0_config: [false, false, false,\n"
            "                       false, false, false,\n"
            "                       true, false, false,\n"
            "                       false, false, true,\n"
            "                       false, false, false]\n"
            "        imu0_config: [false, false, false,\n"
            "                      false, false, false,\n"
            "                      false, false, false,\n"
            "                      false, false, true,\n"
            "                      false, false, false]\n"
        )
        # Only disable IMU yaw fusion (upstream #113). Frequency 30.0 must remain.
        out = patcher.patch_ekf_text(raw, fuse_imu_yaw=False)
        self.assertIn("frequency: 30.0", out)
        self.assertIn("false, false, false,\n                      false, false, false", out)

    def test_kwargs_helpers_drop_absent_keys(self):
        import patcher
        kw = patcher.nav2_kwargs({"base": "mecanum", "inflation_radius": 0.6,
                                  "max_vel_x": None, "unrelated": 1})
        self.assertEqual(kw, {"base_type": "mecanum", "inflation_radius": 0.6})
        ekw = patcher.ekf_kwargs({"fuse_vy": True, "frequency": None}, base="4wd")
        self.assertEqual(ekw, {"base_type": "4wd", "fuse_vy": True})

    def test_deploy_robot_specs_shape(self):
        """generate_custom_robot_specs stays in the nested {design, tuning} shape the deploy path expects."""
        import patcher
        specs = server.generate_custom_robot_specs(
            "4WD Mecanum robot with 97mm wheels, 30cm track width, LD19 lidar")
        tuning = specs["tuning"]
        base = specs["design"]["base_type"]
        self.assertEqual(base, "mecanum")
        nav_kw = patcher.nav2_kwargs(dict(tuning["nav2"], base_type=base))
        self.assertEqual(nav_kw["base_type"], "mecanum")
        self.assertIn("inflation_radius", nav_kw)
        ekf_kw = patcher.ekf_kwargs(tuning["ekf"], base=base)
        self.assertTrue(ekf_kw["fuse_vy"])
        self.assertEqual(ekf_kw["frequency"], 50.0)

    def test_live_server_status_api(self):
        # Queries active console server on localhost:8090
        try:
            with urlopen("http://localhost:8090/api/status", timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode("utf-8"))
                self.assertIn("workspace_path", data)
                self.assertIn("host_ip", data)
                self.assertIn("os", data)
                self.assertIn("agent_busy_console", data)
                self.assertIn("bringup_busy_console", data)
                self.assertIn("supported_distros", data)
                self.assertIn("config", data)
        except Exception as e:
            self.skipTest(f"Console server not running on port 8090: {e}")

    def test_live_server_static_assets(self):
        # Verifies static assets are served properly
        for path, content_type in [
            ("/", "text/html"),
            ("/app.js", "application/javascript"),
            ("/style.css", "text/css")
        ]:
            try:
                with urlopen(f"http://localhost:8090{path}", timeout=5) as resp:
                    self.assertEqual(resp.status, 200)
                    ct = resp.headers.get("Content-Type", "")
                    self.assertIn(content_type, ct)
                    body = resp.read()
                    self.assertTrue(len(body) > 0)
            except Exception as e:
                self.skipTest(f"Static asset test skipped: {e}")

    def test_patcher_capabilities(self):
        import patcher
        self.assertIsNotNone(patcher)
        self.assertIn("standard_diff", patcher.PRESETS)
        self.assertIn("mecanum_omni", patcher.PRESETS)

        # Nav2 patching
        raw_nav = "amcl:\n  ros__parameters:\n    robot_model_type: \"nav2_amcl::DifferentialMotionModel\"\nvelocity_smoother:\n  ros__parameters:\n    max_velocity: [0.5, 0.0, 2.5]\n"
        patched_nav = patcher.patch_nav2_text(raw_nav, base_type="mecanum", max_vel_x=0.6, max_vel_theta=2.8)
        self.assertIn("nav2_amcl::OmniMotionModel", patched_nav)
        self.assertIn("[0.6, 0.6, 2.8]", patched_nav)

        # EKF patching
        raw_ekf = "ekf_filter_node:\n    ros__parameters:\n        frequency: 50.0\n        odom0_config: [false, false, false,\n                       false, false, false,\n                       true, false, false,\n                       false, false, true,\n                       false, false, false]\n"
        patched_ekf = patcher.patch_ekf_text(raw_ekf, base_type="mecanum", fuse_vy=True)
        self.assertIn("true, true, false", patched_ekf)

    def test_ai_robot_builder_logic(self):
        specs = server.generate_custom_robot_specs("4WD Mecanum delivery robot with 97mm wheels, 30cm track width, and LD19 lidar")
        self.assertIn("design", specs)
        self.assertIn("tuning", specs)
        self.assertEqual(specs["design"]["base_type"], "mecanum")
        self.assertEqual(specs["design"]["wheel_diameter_m"], 0.097)
        self.assertEqual(specs["design"]["track_width_m"], 0.30)
        self.assertEqual(specs["design"]["laser_sensor"], "ldlidar")
        self.assertTrue(specs["tuning"]["ekf"]["fuse_vy"])
        self.assertIn("OmniMotionModel", specs["tuning"]["nav2"]["robot_model_type"])

    def test_live_server_tuning_and_ai_apis(self):
        try:
            # Presets
            with urlopen("http://localhost:8090/api/presets", timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode())
                self.assertIn("presets", data)
                self.assertIn("mecanum_omni", data["presets"])

            # EKF
            with urlopen("http://localhost:8090/api/ekf_config?base=mecanum", timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode())
                self.assertIn("config", data)

            # SLAM
            with urlopen("http://localhost:8090/api/slam_config", timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode())
                self.assertIn("config", data)

            # AI tune
            req = Request("http://localhost:8090/api/ai/tune",
                          data=json.dumps({"prompt": "Mecanum strafe", "base": "mecanum"}).encode(),
                          headers={"Content-Type": "application/json"})
            with urlopen(req, timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode())
                self.assertIn("diagnosis", data)
                self.assertIn("nav2_patch", data)

            # AI Robot Builder
            req2 = Request("http://localhost:8090/api/ai/robot_builder",
                           data=json.dumps({"description": "Mecanum 97mm robot"}).encode(),
                           headers={"Content-Type": "application/json"})
            with urlopen(req2, timeout=5) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode())
                self.assertIn("design", data)
                self.assertIn("tuning", data)

        except Exception as e:
            self.skipTest(f"Live server test skipped: {e}")


    def test_robot_config_yaml_roundtrip_fidelity(self):
        cfg = {
            "base": "mecanum",
            "laser_sensor": "ld19",
            "depth_sensor": "realsense",
            "robot_name": "rover_mecanum",
            "ros_distro": "jazzy",
            "ros_domain_id": 42,
            "micro_ros_transport": "serial",
            "micro_ros_port": "/dev/ttyACM0",
            "micro_ros_baudrate": 1500000,
            "laser_serial_port": "/dev/serial/by-path/pci-0000:00-usb-0:1",
            "laser_baud": "230400",
            "madgwick": True,
        }
        sample_nav2 = "amcl:\n  ros__parameters:\n    use_sim_time: False\n    alpha1: 0.2"
        sample_ekf = "ekf_filter_node:\n  ros__parameters:\n    frequency: 50.0"
        sample_slam = "slam_toolbox:\n  ros__parameters:\n    resolution: 0.05"

        yaml_text = server.generate_unified_yaml(cfg, sample_nav2, sample_ekf, sample_slam)
        self.assertIn('linorobot2:', yaml_text)
        self.assertIn('base: "mecanum"', yaml_text)
        self.assertIn('laser_sensor: "ld19"', yaml_text)
        self.assertIn('depth_sensor: "realsense"', yaml_text)
        self.assertIn('nav2:', yaml_text)
        self.assertIn('ekf:', yaml_text)
        self.assertIn('slam:', yaml_text)

        parsed = server.parse_unified_yaml(yaml_text)
        lino = parsed["linorobot2"]
        self.assertEqual(lino["base"], "mecanum")
        self.assertEqual(lino["laser_sensor"], "ld19")
        self.assertEqual(lino["depth_sensor"], "realsense")
        self.assertEqual(lino["robot_name"], "rover_mecanum")
        self.assertEqual(lino["ros_domain_id"], 42)
        self.assertEqual(lino["micro_ros_port"], "/dev/ttyACM0")
        self.assertEqual(lino["micro_ros_baudrate"], 1500000)
        self.assertEqual(lino["madgwick"], True)
        self.assertIn("use_sim_time: False", parsed["nav2"])
        self.assertIn("frequency: 50.0", parsed["ekf"])
        self.assertIn("resolution: 0.05", parsed["slam"])

    def test_save_and_get_robot_config(self):
        server.set_active_robot_name("skid_steer_4wd")
        test_yaml_file = server.get_robot_config_path("skid_steer_4wd")
        cfg = {
            "base": "4wd",
            "laser_sensor": "ydlidar",
            "depth_sensor": "astra",
            "robot_name": "skid_steer_4wd",
            "ros_domain_id": 7,
            "micro_ros_transport": "serial",
            "micro_ros_port": "/dev/ttyUSB0",
            "micro_ros_baudrate": 921600,
            "madgwick": False,
        }
        res = server.save_unified_config(cfg, distro="jazzy", base="4wd")
        self.assertEqual(res["status"], "saved")
        self.assertTrue(os.path.exists(test_yaml_file))

        loaded = server.get_unified_config(distro="jazzy", base="4wd")
        self.assertEqual(loaded["base"], "4wd")
        self.assertEqual(loaded["linorobot2"]["laser_sensor"], "ydlidar")
        self.assertEqual(loaded["linorobot2"]["depth_sensor"], "astra")
        self.assertEqual(loaded["linorobot2"]["micro_ros_baudrate"], 921600)
        self.assertEqual(loaded["linorobot2"]["madgwick"], False)
        # config path is the repo-based per-robot file
        self.assertTrue(loaded["path"].endswith("skid_steer_4wd_config.yaml"))

    def test_launch_bringup_loader_reads_robot_config(self):
        import launch_bringup
        test_yaml = os.path.join(self.temp_dir, "custom_robot_config.yaml")
        sample_content = """linorobot2:
  base: "mecanum"
  laser_sensor: "ld19"
  depth_sensor: "realsense"
  micro_ros_transport: "serial"
  micro_ros_port: "/dev/ttyACM0"
  micro_ros_baudrate: 1500000
  madgwick: true
"""
        with open(test_yaml, "w") as f:
            f.write(sample_content)

        params = launch_bringup._load_robot_config_yaml(test_yaml)
        self.assertEqual(params["base"], "mecanum")
        self.assertEqual(params["laser_sensor"], "ld19")
        self.assertEqual(params["depth_sensor"], "realsense")
        self.assertEqual(params["micro_ros_baudrate"], "1500000")
        self.assertEqual(os.environ["LINOROBOT2_BASE"], "mecanum")
        self.assertEqual(os.environ["LINOROBOT2_LASER_SENSOR"], "ld19")
        self.assertEqual(os.environ["LINOROBOT2_DEPTH_SENSOR"], "realsense")


    def test_agent_port_check_available(self):
        res = server.check_agent_port_status("/dev/ttyNonExistent99", mode="serial")
        self.assertEqual(res["status"], "ok")
        self.assertFalse(res["in_use"])
        self.assertEqual(res["summary"], "Port is available")

    def test_parse_port_check_output_container(self):
        sample = (
            "---FUSER---\n"
            "---CONTAINERS---\n"
            "e409bcb5a438|uros-pico2-test|docker.io/microros/micro-ros-agent:jazzy|serial --dev /dev/ttyACM0 -b 921600\n"
            "---PROCESSES---\n"
        )
        res = {
            "status": "ok", "in_use": False, "mode": "serial",
            "target": "/dev/ttyACM0", "holder_type": "none",
            "pids": [], "process_names": [], "container_id": "",
            "container_name": "", "is_microros": False,
            "details": "", "summary": "Port is available"
        }
        parsed = server._parse_port_check_output(sample, "/dev/ttyACM0", "serial", 8888, res)
        self.assertTrue(parsed["in_use"])
        self.assertEqual(parsed["holder_type"], "container")
        self.assertEqual(parsed["container_name"], "uros-pico2-test")
        self.assertTrue(parsed["is_microros"])

    def test_port_check_container_matches_only_its_own_device(self):
        """An agent on one device must not mark every other port occupied.

        The container branch used to match on the image/name alone, so a single
        `microros/micro-ros-agent` container on /dev/ttyUSB0 reported ttyACM0 --
        and even a nonexistent port -- as in use. It needs `docker ps
        --no-trunc` for the device to be visible in the command at all.
        """
        sample = (
            "---FUSER---\n"
            "---CONTAINERS---\n"
            "9602d9dbd425|uros-gendrv|microros/micro-ros-agent:jazzy|"
            "/bin/sh /micro-ros_entrypoint.sh serial --dev /dev/ttyUSB0 -b 1500000\n"
            "---PROCESSES---\n"
        )

        def fresh(target):
            return {
                "status": "ok", "in_use": False, "mode": "serial",
                "target": target, "holder_type": "none",
                "pids": [], "process_names": [], "container_id": "",
                "container_name": "", "is_microros": False,
                "details": "", "summary": "Port is available",
            }

        # the device it really holds
        held = server._parse_port_check_output(sample, "/dev/ttyUSB0", "serial", 8888,
                                               fresh("/dev/ttyUSB0"))
        self.assertTrue(held["in_use"])
        self.assertEqual(held["container_name"], "uros-gendrv")

        # every other device must come back free
        for other in ("/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyNonExistent99"):
            r = server._parse_port_check_output(sample, other, "serial", 8888, fresh(other))
            self.assertFalse(r["in_use"], f"{other} wrongly reported as occupied")
            self.assertEqual(r["summary"], "Port is available")

        # A non-micro-ROS container holding a device is still detected.
        lidar = (
            "---FUSER---\n"
            "---CONTAINERS---\n"
            "fd5b17e8100a|ld19-run|ros:jazzy-ros-base|"
            "ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node -p port_name:=/dev/ttyUSB1\n"
            "---PROCESSES---\n"
        )
        r = server._parse_port_check_output(lidar, "/dev/ttyUSB1", "serial", 8888,
                                            fresh("/dev/ttyUSB1"))
        self.assertTrue(r["in_use"])
        self.assertEqual(r["container_name"], "ld19-run")

    def test_parse_port_check_output_fuser(self):
        sample = (
            "---FUSER---\n"
            "1355755\n"
            "---CONTAINERS---\n"
            "---PROCESSES---\n"
            "1355755 micro_ros_agent serial --dev /dev/ttyACM0 -b 1500000\n"
        )
        res = {
            "status": "ok", "in_use": False, "mode": "serial",
            "target": "/dev/ttyACM0", "holder_type": "none",
            "pids": [], "process_names": [], "container_id": "",
            "container_name": "", "is_microros": False,
            "details": "", "summary": "Port is available"
        }
        parsed = server._parse_port_check_output(sample, "/dev/ttyACM0", "serial", 8888, res)
        self.assertTrue(parsed["in_use"])
        self.assertEqual(parsed["holder_type"], "process")
        self.assertIn("1355755", parsed["pids"])
        self.assertTrue(parsed["is_microros"])

    def test_workflow_config_defaults(self):
        cfg = server.load_config()
        self.assertEqual(cfg.get("install_mode"), "native")
        # Blank, not "docker": shipping a container engine put a fresh native
        # install into a container workflow before the user chose anything.
        self.assertEqual(cfg.get("agent_engine"), "")

    def test_ros2_install_cmd(self):
        for distro in server.SUPPORTED_DISTROS:
            cmd = server.build_ros2_install_cmd(distro)
            self.assertIn(f"ros-{distro}-ros-base", cmd)
            self.assertIn("python3-colcon-common-extensions", cmd)
            self.assertIn("ros2-apt-source", cmd)
            # A bare `|| true` after the apt install would also swallow the
            # install's own failure and exit 0 with no ROS 2 on disk.
            self.assertIn("(sudo rosdep init 2>/dev/null || true)", cmd)
            rc = subprocess.run(["bash", "-n"], input=cmd, text=True,
                                capture_output=True).returncode
            self.assertEqual(rc, 0, f"generated install command is not valid bash for {distro}")

    def test_ros2_installed_flag(self):
        self.assertIsInstance(server.ros2_installed("jazzy"), bool)
        self.assertIsInstance(server.toolchain_ready(), bool)

    def test_ros2_install_cmd_includes_toolchain(self):
        # ros-<distro>-ros-base is runtime-only; without these the first colcon
        # build after the install fails on a missing C++ compiler.
        cmd = server.build_ros2_install_cmd("jazzy")
        for pkg in ("build-essential", "cmake"):
            self.assertIn(pkg, cmd)

    def test_sensor_udev_steps_are_best_effort(self):
        # Inside a container /etc/udev/rules.d does not exist; a bare
        # `sudo cp ... /etc/udev/rules.d` failed the whole driver install after
        # the driver had already built.
        cmd = server.build_sensor_install_cmd("laser", "ldlidar")
        self.assertIn("sudo mkdir -p /etc/udev/rules.d", cmd)
        self.assertIn("udev rule step skipped", cmd)
        rc = subprocess.run(["bash", "-n"], input=cmd, text=True,
                            capture_output=True).returncode
        self.assertEqual(rc, 0)

    def test_source_fallback_for_unpublished_packages(self):
        # nav2_bringup has no ros-lyrical-nav2-bringup binary; apt answers
        # "Unable to locate package" and the launch then fails on the same
        # package it just "installed".
        cmd = server.build_source_package_cmd("nav2_bringup", distro="lyrical",
                                              ws="/tmp/ws_test")
        self.assertIsNotNone(cmd)
        self.assertIn("navigation2", cmd)
        # nav2_bringup find_package()s the navigation2 metapackage, which has no
        # binary package either -- both have to be built, and rosdep has to be
        # told not to look for ros-<distro>-navigation2 in apt.
        self.assertIn("--packages-select nav2_bringup navigation2", cmd)
        # nav2_smac_planner is likewise unpublished on some distros (Lyrical
        # has navfn/planner/theta-star but no smac) and the navigation2
        # metapackage depends on it, so rosdep tried to apt-install it and
        # failed. Console configures NavfnPlanner, so it is skipped too.
        self.assertIn("--skip-keys 'microxrcedds_agent navigation2 nav2_smac_planner'", cmd)
        self.assertIn("COLCON_IGNORE", cmd)
        rc = subprocess.run(["bash", "-n"], input=cmd, text=True,
                            capture_output=True).returncode
        self.assertEqual(rc, 0)
        # A package with no fallback entry must still return None -- the map is
        # deliberate, not a catch-all that source-builds anything apt misses.
        # (slam_toolbox *does* have an entry now: Rolling on 26.04 publishes
        # neither it nor nav2_bringup.)
        self.assertIsNone(server.build_source_package_cmd("robot_localization"))

    def test_package_install_info_reports_source(self):
        info = server.get_package_install_info("nav2_bringup", distro="lyrical",
                                               ws="/tmp/ws_test")
        self.assertIn(info["source"], ("apt", "source"))
        self.assertTrue(info["install_cmd"])

    def test_nav2_stack_status_and_cmd(self):
        st = server.nav2_stack_status(distro="lyrical", ws="/tmp/ws_test")
        self.assertIn("installed", st)
        self.assertIsInstance(st["missing"], list)
        cmd = server.build_nav2_stack_cmd("lyrical")
        # Expanded from apt rather than a hand-kept list, which drifts with
        # every nav2 release.
        self.assertIn("^ros-lyrical-(nav2|opennav)-", cmd)
        self.assertIn("-dbgsym", cmd)
        rc = subprocess.run(["bash", "-n"], input=cmd, text=True,
                            capture_output=True).returncode
        self.assertEqual(rc, 0)

    def test_rotation_shim_primary_controller_namespace(self):
        # Kilted namespaced the shim's primary controller parameters. Flat, as
        # Jazzy wants them, controller_server aborts configure with "Failed to
        # get 'primary_controller.plugin' parameter" and lifecycle_manager
        # gives up on the whole nav2 bringup.
        import yaml as _yaml
        for distro in ("lyrical", "rolling"):
            for suffix in ("", "_mecanum"):
                path = os.path.join(server.CONFIG_DIR, f"nav2_{distro}{suffix}.yaml")
                with open(path) as f:
                    cfg = _yaml.safe_load(f)
                fp = cfg["controller_server"]["ros__parameters"]["FollowPath"]
                pc = fp.get("primary_controller")
                self.assertIsInstance(pc, dict, f"{path}: primary_controller must be a namespace")
                self.assertIn("plugin", pc, path)
                self.assertIn("desired_linear_vel", pc, path)
        # Kilted also renamed bt_navigator's error_code_names. Left alone,
        # bt_navigator throws while creating navigate_to_pose and the composed
        # component_container dies with SIGSEGV, taking the rest of nav2 down.
        for distro in ("lyrical", "rolling"):
            for suffix in ("", "_mecanum"):
                path = os.path.join(server.CONFIG_DIR, f"nav2_{distro}{suffix}.yaml")
                with open(path) as f:
                    cfg = _yaml.safe_load(f)
                bt = cfg["bt_navigator"]["ros__parameters"]
                self.assertNotIn("error_code_names", bt, path)
                self.assertIn("error_code_name_prefixes", bt, path)

        # Jazzy predates the change and must keep the flat form.
        with open(os.path.join(server.CONFIG_DIR, "nav2_jazzy.yaml")) as f:
            jazzy = _yaml.safe_load(f)
        self.assertIsInstance(
            jazzy["controller_server"]["ros__parameters"]["FollowPath"]["primary_controller"], str)

    def test_container_status_api(self):
        status = server.check_container_status()
        self.assertEqual(status["status"], "ok")
        self.assertIn("has_docker", status)
        self.assertIn("has_podman", status)
        self.assertIn("is_rootless_docker", status)
        self.assertIn("platform_system", status)

    def test_rootless_info_api(self):
        info = server.get_rootless_info()
        self.assertEqual(info["status"], "ok")
        self.assertIn("commands", info)
        self.assertIn("ubuntu_debian", info["commands"])
        self.assertIn("podman_alternative", info["commands"])
        self.assertIn("user", info)
        self.assertIn("uid", info)

    def test_autostart_status_and_lifecycle(self):
        status = server.get_autostart_status()
        self.assertEqual(status["status"], "ok")
        self.assertIn("has_service", status)
        self.assertIn("enabled", status)
        self.assertIn("lingering", status)
        self.assertEqual(status["service_name"], "linorobot2-autostart.service")

        # Test enable autostart logic (mock / generation)
        res = server.enable_autostart({
            "stack": "full_nav2",
            "mode": "native",
            "distro": "jazzy"
        })
        self.assertEqual(res["status"], "ok")
        self.assertTrue(res["enabled"])
        self.assertTrue(os.path.exists(res["script_path"]))
        self.assertTrue(os.path.exists(res["service_path"]))

        # Verify generated script content
        with open(res["script_path"]) as sf:
            s_content = sf.read()
            self.assertIn("linorobot2_bringup", s_content)
            self.assertIn("linorobot2_navigation", s_content)

        # Test disable autostart
        dis_res = server.disable_autostart()
        self.assertEqual(dis_res["status"], "ok")
        self.assertFalse(dis_res["enabled"])

    def test_setup_rootless_docker(self):
        res = server.setup_rootless_docker()
        self.assertIn("status", res)
        self.assertIn("success", res)
        self.assertIn("is_rootless", res)
        self.assertIn("message", res)

    def test_install_container_engine(self):
        res_p = server.install_container_engine("podman")
        self.assertIn("status", res_p)
        self.assertIn("installed", res_p)
        self.assertEqual(res_p["engine"], "podman")

        res_d = server.install_container_engine("docker")
        self.assertIn("status", res_d)
        self.assertIn("installed", res_d)
        self.assertEqual(res_d["engine"], "docker")

    # ------------------------------------------------------------------
    # Repo-based robot config: Robot Name + Branch header
    # ------------------------------------------------------------------
    def test_active_robot_name_get_set_default(self):
        self.assertEqual(server.get_active_robot_name(), "linorobot2")
        server.set_active_robot_name("scout")
        self.assertEqual(server.get_active_robot_name(), "scout")
        self.assertTrue(os.path.exists(server.ACTIVE_ROBOT_FILE))
        with self.assertRaises(ValueError):
            server.set_active_robot_name("Bad Name!")

    def test_get_robot_config_path_layout(self):
        server.set_active_robot_name("gendrv")
        p = server.get_robot_config_path()
        self.assertEqual(p, os.path.join(server.ROBOT_CONFIGS_DIR, "gendrv_config.yaml"))
        self.assertEqual(
            server.get_robot_config_path("rover_x"),
            os.path.join(server.ROBOT_CONFIGS_DIR, "rover_x_config.yaml"),
        )

    def test_load_save_config_roundtrip_via_yaml_console_section(self):
        cfg = server.load_config()
        cfg["ros_distro"] = "rolling"
        cfg["install_mode"] = "podman"
        cfg["agent_port"] = "7777"
        cfg["ros_domain_id"] = 15
        cfg["auto_bringup"] = False
        server.save_config(cfg)
        # persisted as a console: section in the active robot's yaml
        with open(server.get_robot_config_path()) as _f:
            text = _f.read()
        self.assertIn("console:", text)
        self.assertIn("ros_distro:", text)
        loaded = server.load_config()
        self.assertEqual(loaded["ros_distro"], "rolling")
        self.assertEqual(loaded["install_mode"], "podman")
        self.assertEqual(loaded["agent_port"], "7777")   # stays a str
        self.assertEqual(loaded["ros_domain_id"], 15)     # stays an int
        self.assertIs(loaded["auto_bringup"], False)      # stays a bool

    def test_list_robot_configs(self):
        server.save_config(server.load_config(), robot_name="linorobot2")
        server.save_config(server.load_config(), robot_name="scout")
        server.set_active_robot_name("scout")
        robots = server.list_robot_configs()
        names = {r["name"] for r in robots}
        self.assertEqual(names, {"linorobot2", "scout"})
        active = [r for r in robots if r["active"]]
        self.assertEqual(len(active), 1)
        self.assertEqual(active[0]["name"], "scout")

    def test_migrate_legacy_console_config_json(self):
        # No repo config yet; a legacy console_config.json exists.
        with open(server.CONFIG_PATH, "w") as f:
            json.dump({"robot_name": "legacybot", "ros_distro": "lyrical",
                       "install_mode": "docker", "agent_baud": "500000"}, f)
        res = server.migrate_legacy_config()
        self.assertIsNotNone(res)
        self.assertEqual(res["robot"], "legacybot")
        self.assertTrue(os.path.exists(server.get_robot_config_path("legacybot")))
        self.assertEqual(server.get_active_robot_name(), "legacybot")
        loaded = server.load_config()
        self.assertEqual(loaded["ros_distro"], "lyrical")
        self.assertEqual(loaded["agent_baud"], "500000")
        # idempotent
        self.assertIsNone(server.migrate_legacy_config())

    def test_collect_git_info_shape(self):
        gi = server.collect_git_info()
        for key in ("version", "branch", "branches", "dirty", "commits"):
            self.assertIn(key, gi)
        self.assertIsInstance(gi["branches"], list)
        self.assertIsInstance(gi["commits"], list)

    def test_commit_robot_config_if_dirty_noop_when_clean(self):
        # Not a git repo path -> commit helper must never raise, returns ''.
        server.save_config(server.load_config())
        out = server.commit_robot_config_if_dirty(action_label="unit-test")
        self.assertIsInstance(out, str)

    def test_generate_unified_yaml_includes_console_section(self):
        console_cfg = dict(server.DEFAULT_CONFIG)
        console_cfg["ros_distro"] = "rolling"
        text = server.generate_unified_yaml(
            {"base": "2wd"}, "amcl:\n  ros__parameters:\n    x: 1",
            "ekf_filter_node:\n  ros__parameters:\n    frequency: 50.0",
            "slam_toolbox:\n  ros__parameters:\n    resolution: 0.05",
            console_cfg=console_cfg,
        )
        self.assertIn("console:", text)
        self.assertIn('ros_distro: "rolling"', text)
        parsed = server.parse_unified_yaml(text)
        self.assertEqual(parsed["console"].get("ros_distro"), "rolling")
        self.assertEqual(parsed["linorobot2"].get("base"), "2wd")

    # ------------------------------------------------------------------
    # Bringup health: topic- + TF-level readiness, not just pgrep
    # ------------------------------------------------------------------
    def test_deindent_preserves_comments_below_section_indent(self):
        """A banner comment at column 0 must keep its '#'.

        deindent() used to slice min_indent characters off EVERY line, so any
        line indented less than the section minimum lost real content -- a
        '# ----' banner became '----', turning the extracted section into
        invalid YAML. That silently corrupted console_ekf.yaml on save.
        """
        text = (
            "ekf:\n"
            "  ekf_filter_node:\n"
            "    ros__parameters:\n"
            "      frequency: 50.0\n"
            "# ------------------------------------------------------------\n"
            "# SLAM Toolbox Parameters\n"
            "# ------------------------------------------------------------\n"
            "slam:\n"
            "  slam_toolbox:\n"
            "    ros__parameters:\n"
            "      resolution: 0.05\n"
        )
        parsed = server.parse_unified_yaml(text)
        for section in ("ekf", "slam"):
            body = parsed[section]
            for line in body.splitlines():
                stripped = line.strip()
                if stripped.startswith("---") and "-----" in stripped:
                    self.fail(f"{section}: banner comment lost its '#': {line!r}")
            if yaml is not None:
                # the whole point: it must still parse as YAML
                self.assertTrue(yaml.safe_load(body), f"{section} did not parse")
        self.assertIn("# SLAM Toolbox Parameters", parsed["ekf"])
        self.assertIn("frequency: 50.0", parsed["ekf"])
        self.assertIn("resolution: 0.05", parsed["slam"])

    def test_parse_topic_hz(self):
        # `ros2 topic hz` prints a running average; the LAST one has seen the
        # most samples, so that is the one we report.
        out = (
            "average rate: 48.921\n"
            "\tmin: 0.019s max: 0.022s std dev: 0.00051s window: 50\n"
            "average rate: 50.004\n"
            "\tmin: 0.019s max: 0.021s std dev: 0.00043s window: 100\n"
        )
        self.assertAlmostEqual(server._parse_topic_hz(out), 50.004)
        # a topic with a publisher but no messages prints no average at all
        self.assertIsNone(server._parse_topic_hz(
            "WARNING: topic [/scan] does not appear to be published yet\n"))
        self.assertIsNone(server._parse_topic_hz(""))

    def test_bringup_health_topic_and_tf_expectations(self):
        keys = [k for k, _t, _w, _h in server.BRINGUP_HEALTH_TOPICS]
        self.assertEqual(keys, ["odom_raw", "odom", "imu", "scan"])
        topics = {t for _k, t, _w, _h in server.BRINGUP_HEALTH_TOPICS}
        # /odom/unfiltered comes straight off the microcontroller, /odom from EKF
        self.assertIn("/odom/unfiltered", topics)
        self.assertIn("/odom", topics)
        self.assertIn("/scan", topics)
        # the TF chain SLAM/Nav2 need before they do anything useful
        self.assertEqual(server.BRINGUP_TF_CHAIN,
                         [("odom", "base_footprint"), ("base_footprint", "laser")])

    def test_bringup_health_shape_and_no_graph(self):
        h = server.check_bringup_health(timeout=1.0)
        for key in ("status", "ready", "ros_available", "topics", "tf", "summary"):
            self.assertIn(key, h)
        self.assertEqual(set(h["topics"]), {"odom_raw", "odom", "imu", "scan"})
        self.assertEqual(len(h["tf"]), len(server.BRINGUP_TF_CHAIN))
        self.assertTrue(h["summary"])
        for entry in h["topics"].values():
            for key in ("topic", "what", "min_hz", "advertised", "hz", "ok"):
                self.assertIn(key, entry)
        # With no ROS graph reachable nothing may be reported as ready, and the
        # summary must name the two things that actually cause it.
        if not h["ros_available"]:
            self.assertEqual(h["status"], "no_graph")
            self.assertFalse(h["ready"])
            self.assertIn("ROS_DOMAIN_ID", h["summary"])
            self.assertFalse(any(t["ok"] for t in h["topics"].values()))
            self.assertFalse(any(l["ok"] for l in h["tf"]))

    def test_splice_yaml_section_replaces_and_inserts(self):
        base = 'linorobot2:\n  base: "2wd"\n'
        spliced = server.splice_yaml_section(base, "console", "console:\n  ros_distro: \"jazzy\"")
        self.assertIn("console:", spliced)
        self.assertIn('base: "2wd"', spliced)
        # replace, not duplicate
        again = server.splice_yaml_section(spliced, "console", "console:\n  ros_distro: \"rolling\"")
        self.assertEqual(again.count("console:"), 1)
        self.assertIn("rolling", again)

    def test_git_info_shape_and_badge(self):
        info = server.collect_git_info()
        self.assertIn("version", info)
        self.assertIn("version_at_start", info)
        self.assertIn("branch", info)
        self.assertIn("branches", info)
        self.assertIn("remotes", info)
        self.assertIn("commits", info)
        self.assertEqual(len(info["version"]), 7)
        self.assertEqual(len(info["version_at_start"]), 7)

        html_path = os.path.join(server.WEB_DIR, "index.html")
        with open(html_path, "r", encoding="utf-8") as f:
            html = f.read()
        self.assertIn('id="git-version-badge"', html)
        self.assertIn('id="git-version-text"', html)
        self.assertIn('id="git-version-popover"', html)

        js_path = os.path.join(server.WEB_DIR, "app.js")
        with open(js_path, "r", encoding="utf-8") as f:
            js = f.read()
        self.assertIn("initGitVersionBadge", js)

    def test_container_registry_configurable(self):
        # 1. Config defaults and YAML rendering
        self.assertIn("container_registry", server.DEFAULT_CONFIG)
        self.assertEqual(server.DEFAULT_CONFIG["container_registry"], "auto")
        self.assertIn("custom_registry", server.DEFAULT_CONFIG)
        yaml_sec = server.render_console_section({"container_registry": "cluster", "custom_registry": "reg.local:5000"})
        self.assertIn('container_registry: "cluster"', yaml_sec)
        self.assertIn('custom_registry: "reg.local:5000"', yaml_sec)

        # 2. HTML elements
        html_path = os.path.join(server.WEB_DIR, "index.html")
        with open(html_path, "r", encoding="utf-8") as f:
            html = f.read()
        self.assertIn('id="hdr-container-registry"', html)
        self.assertIn('id="hdr-custom-registry"', html)
        self.assertIn('id="cfg-container-registry"', html)
        self.assertIn('id="cfg-custom-registry"', html)

        # 3. JS helper
        js_path = os.path.join(server.WEB_DIR, "app.js")
        with open(js_path, "r", encoding="utf-8") as f:
            js = f.read()
        self.assertIn("getContainerRegistry", js)
        self.assertIn("syncRegistryState", js)


    def test_process_runner_rolling_buffer_and_subscribers(self):
        runner = server.ProcessRunner("test_runner", max_history=10)
        self.assertEqual(runner.get_history(), [])
        
        # Test broadcast and history
        runner._broadcast("output", {"line": "line 1"})
        runner._broadcast("output", {"line": "line 2"})
        self.assertEqual(runner.get_history(), ["line 1", "line 2"])
        
        # Test subscriber queue
        import queue
        q = queue.Queue()
        runner.subscribe(q)
        runner._broadcast("output", {"line": "line 3"})
        self.assertFalse(q.empty())
        ev, payload = q.get_nowait()
        self.assertEqual(ev, "output")
        self.assertEqual(payload["line"], "line 3")
        
        runner.unsubscribe(q)
        runner._broadcast("output", {"line": "line 4"})
        self.assertTrue(q.empty())

    def test_sensor_driver_status_resolution(self):
        # 1. ldlidar -> ldlidar_stl_ros2
        key, entry, pkg = server.find_laser_driver_info("ld19")
        self.assertEqual(key, "ldlidar")
        self.assertEqual(pkg, "ldlidar_stl_ros2")
        
        # 2. rplidar / sllidar -> sllidar_ros2
        key, entry, pkg = server.find_laser_driver_info("a1")
        self.assertEqual(key, "sllidar")
        self.assertEqual(pkg, "sllidar_ros2")
        
        # 3. ydlidar -> ydlidar_ros2_driver
        key, entry, pkg = server.find_laser_driver_info("ydlidar")
        self.assertEqual(key, "ydlidar")
        self.assertEqual(pkg, "ydlidar_ros2_driver")
        
        # 4. Status dictionary format
        status = server.get_sensor_driver_status("ld19", ws="/tmp/dummy_ws")
        self.assertEqual(status["sensor"], "ld19")
        self.assertEqual(status["package"], "ldlidar_stl_ros2")
        self.assertIn("installed", status)
        self.assertIn("reason", status)
        self.assertIn("install_cmd", status)
        if not status["installed"]:
            self.assertIn("git clone", status["install_cmd"])
        else:
            self.assertIsNone(status["install_cmd"])

    def test_bringup_stream_endpoint_idle(self):
        # Verify /api/bringup/stream endpoint is registered and responds
        srv = server.ThreadingHTTPServer(("127.0.0.1", 0), server.ConsoleHandler)
        port = srv.server_port
        t = threading.Thread(target=srv.serve_forever)
        t.daemon = True
        t.start()
        try:
            url = f"http://127.0.0.1:{port}/api/bringup/stream"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=3) as resp:
                self.assertEqual(resp.status, 200)
                self.assertIn("text/event-stream", resp.headers.get("Content-Type", ""))
                first_chunk = resp.read(100).decode("utf-8")
                self.assertTrue("event: idle" in first_chunk or "event: init" in first_chunk)
        finally:
            srv.shutdown()

    def test_sensors_driver_status_endpoint(self):
        srv = server.ThreadingHTTPServer(("127.0.0.1", 0), server.ConsoleHandler)
        port = srv.server_port
        t = threading.Thread(target=srv.serve_forever)
        t.daemon = True
        t.start()
        try:
            url = f"http://127.0.0.1:{port}/api/sensors/driver_status?sensor=ld19"
            with urllib.request.urlopen(url, timeout=3) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode("utf-8"))
                self.assertEqual(data["package"], "ldlidar_stl_ros2")
                self.assertIn("installed", data)
        finally:
            srv.shutdown()


    def test_build_base_install_cmd(self):
        cmd = server.build_base_install_cmd(ws="/tmp/test_ws", distro="jazzy")
        self.assertIn("mkdir -p /tmp/test_ws/src", cmd)
        self.assertIn("colcon build --symlink-install", cmd)
        self.assertIn("rosdep", cmd)
        self.assertIn("linorobot2", cmd)

    def test_get_package_install_info(self):
        info = server.get_package_install_info("nav2_bringup", distro="jazzy")
        self.assertEqual(info["package"], "nav2_bringup")
        self.assertIn("installed", info)
        if not info["installed"]:
            self.assertIn("apt-get install -y ros-jazzy-nav2-bringup", info["install_cmd"])

    def test_workspace_build_cmd_endpoint(self):
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), server.ConsoleHandler)
        port = httpd.server_address[1]
        t = threading.Thread(target=httpd.serve_forever)
        t.daemon = True
        t.start()
        try:
            url = f"http://127.0.0.1:{port}/api/workspace/build_cmd?ws=/tmp/test_ws&distro=jazzy"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode("utf-8"))
                self.assertIn("command", data)
                self.assertIn("colcon build", data["command"])
                self.assertEqual(data["workspace"], "/tmp/test_ws")
        finally:
            httpd.shutdown()
            httpd.server_close()

    def test_package_check_endpoint(self):
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), server.ConsoleHandler)
        port = httpd.server_address[1]
        t = threading.Thread(target=httpd.serve_forever)
        t.daemon = True
        t.start()
        try:
            url = f"http://127.0.0.1:{port}/api/package/check?pkg=nav2_bringup&distro=jazzy"
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                self.assertEqual(resp.status, 200)
                data = json.loads(resp.read().decode("utf-8"))
                self.assertEqual(data["package"], "nav2_bringup")
                self.assertIn("installed", data)
        finally:
            httpd.shutdown()
            httpd.server_close()

    def test_rviz_teleop_config_exists(self):
        root = os.path.dirname(os.path.abspath(__file__))
        rviz_file = os.path.join(root, "rviz", "teleop.rviz")
        self.assertTrue(os.path.isfile(rviz_file))
        with open(rviz_file) as f:
            content = f.read()
        self.assertIn("Fixed Frame: odom", content)

    def test_gamepad_runner_and_endpoints(self):
        socketserver.ThreadingTCPServer.allow_reuse_address = True
        httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), server.ConsoleHandler)
        port = httpd.server_address[1]
        t = threading.Thread(target=httpd.serve_forever)
        t.daemon = True
        t.start()
        try:
            url = f"http://127.0.0.1:{port}/api/status"
            with urllib.request.urlopen(url, timeout=3.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                self.assertIn("gamepad_running", data)
                self.assertFalse(data["gamepad_running"])

            req = urllib.request.Request(
                f"http://127.0.0.1:{port}/api/gamepad/cmd",
                data=json.dumps({"linear_x": 0.2, "linear_y": 0.0, "angular_z": 0.5}).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                self.assertIn("sent", data)
                self.assertIn("running", data)
                self.assertFalse(data["sent"])

            req = urllib.request.Request(
                f"http://127.0.0.1:{port}/api/gamepad/kill",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST"
            )
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                data = json.loads(resp.read().decode("utf-8"))
                self.assertIn("killed", data)
                self.assertFalse(data["killed"])
        finally:
            httpd.shutdown()
            httpd.server_close()


class TestOrchestrationHelpersAreDefined(unittest.TestCase):
    """The 1-click flow calls a handful of helpers that live in app.js. A missing
    one is a ReferenceError that only surfaces when a user clicks the button, and
    it takes teleop, SLAM and navigation down together -- which is exactly how
    `isDockerMode` shipped: called from all three paths, defined nowhere.

    Deliberately a plain substring check. Regex-parsing JavaScript to find every
    undefined call was tried and abandoned: a stray "/*" inside a string literal
    makes a block-comment strip swallow real code, and the false failures that
    produces would just get the test disabled.
    """

    HELPERS = [
        "isDockerMode",
        "isAgentAlive",
        "isAutoBringupEnabled",
        "ensureAgentRunning",
        "ensureBringupRunning",
        "ensureNav2Prerequisites",
    ]

    def test_helpers_called_by_the_one_click_flow_exist(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js")) as f:
            src = f.read()
        undefined = [h for h in self.HELPERS if f"function {h}" not in src]
        self.assertEqual(undefined, [],
                         f"called by the orchestration but defined nowhere: {undefined}")


class TestImportConfigPersists(unittest.TestCase):
    """Importing a <robot>_config.h has to land in robot_config.yaml. The parsed
    values used to be returned to the browser and thrown away, so the header
    showed the imported robot while bringup still launched with the previous
    port and baud. robot_config.yaml is the single source of truth: the
    launchers read it, and nothing depends on a shell export."""

    def test_header_fields_map_onto_console_config_keys(self):
        mapped = server.imported_config_to_console_config({
            "base": "2wd", "agent_baud": "1500000", "transport": "serial",
            "has_imu": True,
        })
        self.assertEqual(mapped["base_type"], "2wd")
        self.assertEqual(mapped["agent_baud"], "1500000")
        self.assertEqual(mapped["agent_transport"], "serial")
        self.assertTrue(mapped["madgwick"])

    def test_unknown_fields_are_not_written_into_the_config(self):
        mapped = server.imported_config_to_console_config({
            "base": "2wd", "agent_ip": "192.168.1.100", "mag_bias": ["1", "2", "3"],
        })
        self.assertNotIn("agent_ip", mapped)
        self.assertNotIn("mag_bias", mapped)

    def test_lidar_over_udp_maps_to_the_bridge_route(self):
        """USE_LIDAR_UDP means the MCU relays the scan as datagrams: no pin, no
        cable, and Console has to bridge a socket rather than open a device."""
        result = server.parse_robot_config_header(
            "#define USE_FAKE_LD19\n"
            "#define USE_LIDAR_UDP\n"
            "// #define LIDAR_RXD 4\n"
            "#define LIDAR_BAUDRATE 230400\n"
            "#define LIDAR_PORT 8889\n"
            "#define AGENT_PORT 8888\n")
        self.assertEqual(result["lidar_transport"], "udp_bridge")
        mapped = server.imported_config_to_console_config(result)
        self.assertEqual(mapped["laser_transport"], "udp_bridge")
        self.assertEqual(mapped["laser_udp_port"], "8889")
        self.assertEqual(mapped["laser_baud"], "230400")
        self.assertEqual(mapped["agent_port"], "8888")

    def test_lidar_rxd_maps_to_the_serial_route_without_a_udp_port(self):
        """LIDAR_PORT sits in every header. It only means something on the UDP
        route, so a serial robot must not come back carrying one."""
        result = server.parse_robot_config_header(
            "#define USE_FAKE_LD19\n"
            "// #define USE_LIDAR_UDP\n"
            "#define LIDAR_RXD 4\n"
            "#define LIDAR_BAUDRATE 230400\n"
            "#define LIDAR_PORT 8889\n")
        self.assertEqual(result["lidar_transport"], "serial")
        self.assertEqual(result["lidar_rxd"], "4")
        mapped = server.imported_config_to_console_config(result)
        self.assertEqual(mapped["laser_transport"], "serial")
        self.assertNotIn("laser_udp_port", mapped)

    def test_fake_or_real_does_not_change_the_route(self):
        """Console reads bytes; it does not care what produced them. The same
        two headers minus USE_FAKE_LD19 must resolve to the same routes."""
        for extra, expected in (("#define USE_LIDAR_UDP\n", "udp_bridge"),
                                ("#define LIDAR_RXD 4\n", "serial")):
            result = server.parse_robot_config_header(extra + "#define LIDAR_BAUDRATE 230400\n")
            self.assertEqual(result["lidar_transport"], expected)
            self.assertFalse(result["fake_ld19"])
            mapped = server.imported_config_to_console_config(result)
            self.assertEqual(mapped["laser_transport"], expected)
            # A relayed *real* LiDAR could be any LDROBOT model, so the model is
            # only assumed for the emulator, which is known to speak LD19.
            self.assertNotIn("laser_model", mapped)

    def test_the_emulator_pins_the_driver_family_and_model(self):
        result = server.parse_robot_config_header(
            "#define USE_FAKE_LD19\n#define LIDAR_RXD 4\n")
        mapped = server.imported_config_to_console_config(result)
        self.assertEqual(mapped["laser_sensor"], "ldlidar")
        self.assertEqual(mapped["laser_model"], "ld19")

    def test_a_fake_ld19_with_no_route_is_warned_about(self):
        """The one misconfiguration with no symptom on the board: begin() with
        no pin opens no UART, so the firmware runs and publishes nothing."""
        result = server.parse_robot_config_header(
            "#define USE_FAKE_LD19\n"
            "// #define USE_LIDAR_UDP\n"
            "#define LIDAR_BAUDRATE 230400\n")
        self.assertNotIn("lidar_transport", result)
        self.assertTrue(any("publishes no scan" in w for w in result.get("warnings", [])))
        mapped = server.imported_config_to_console_config(result)
        self.assertNotIn("laser_transport", mapped)

    def test_commented_out_defines_are_not_read_as_active(self):
        result = server.parse_robot_config_header(
            "// #define USE_LIDAR_UDP\n"
            "// #define USE_FAKE_LD19\n"
            "// #define AGENT_PORT 8888\n")
        self.assertFalse(result["fake_ld19"])
        self.assertNotIn("lidar_transport", result)
        self.assertNotIn("agent_port", result)

    def test_laser_route_keys_round_trip_through_the_config(self):
        """The keys have to exist in DEFAULT_CONFIG or the mapping silently
        drops them -- imported_config_to_console_config filters on it."""
        for key in ("laser_transport", "laser_udp_port"):
            self.assertIn(key, server.DEFAULT_CONFIG)

    def test_a_fake_imu_turns_madgwick_off(self):
        mapped = server.imported_config_to_console_config({"base": "2wd", "has_imu": False})
        self.assertFalse(mapped["madgwick"])

    def test_persist_writes_through_and_leaves_other_keys_alone(self):
        saved_to = {}
        original_load, original_save = server.load_config, server.save_config
        server.load_config = lambda: {"workspace_path": "/keep/me", "base_type": "4wd"}
        server.save_config = lambda cfg: saved_to.update(cfg)
        try:
            written = server.persist_imported_config(
                {"base": "2wd", "agent_baud": "1500000", "has_imu": True})
        finally:
            server.load_config, server.save_config = original_load, original_save

        self.assertEqual(written["base_type"], "2wd")
        self.assertEqual(saved_to["base_type"], "2wd")          # overwritten
        self.assertEqual(saved_to["workspace_path"], "/keep/me")  # untouched

    def test_nothing_parsed_means_nothing_written(self):
        called = []
        original_save = server.save_config
        server.save_config = lambda cfg: called.append(cfg)
        try:
            self.assertEqual(server.persist_imported_config({}), {})
        finally:
            server.save_config = original_save
        self.assertEqual(called, [])

class TestConsolePaneIsBounded(unittest.TestCase):
    """The console pane must not slow down as output accumulates.

    It used to do `consolePane.textContent += line` plus a scrollHeight read per
    line: a whole-buffer copy and a forced synchronous layout, every line. The
    micro-ROS agent emits a few hundred lines a second against a 50 Hz board, and
    on a fresh Jazzy box that pegged the renderer at 100% CPU for 23 of its 24
    minutes. Timers stopped firing, so the 1-Click chain froze between "starting
    the LiDAR driver" and the nav2 prerequisites -- no error, no failed request,
    nothing in any log to point at it.

    So this is a behavioural test, not a substring check: the ring-buffer block
    is pulled out of app.js and executed in node against a stub pane.
    """

    @staticmethod
    def _console_block():
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js"), encoding="utf-8") as f:
            src = f.read()
        start = src.index("const CONSOLE_MAX_LINES")
        end = src.index("\n}\n", src.index("function clearConsole", start)) + 3
        return src, src[start:end]

    def test_appending_is_not_quadratic(self):
        src, _ = self._console_block()
        self.assertFalse("consolePane.textContent +=" in src,
                         "app.js appends onto the pane's textContent again -- "
                         "that is a whole-buffer copy per line")

    @unittest.skipUnless(shutil.which("node"), "node is required to run app.js")
    def test_ring_buffer_bounds_the_pane_and_batches_repaints(self):
        _, block = self._console_block()
        harness = """
let flushes = 0, layouts = 0;
const consolePane = {
  _text: "",
  scrollTop: 0,
  get scrollHeight() { layouts++; return 1; },
  get textContent() { return this._text; },
  set textContent(v) { flushes++; this._text = v; },
};
%s
for (let i = 0; i < 50000; i++) logLine("line " + i);
const pending = consoleFlushTimer !== null;
flushConsole();
const lines = consolePane.textContent.split("\\n").filter((l) => l.length);
console.log(JSON.stringify({
  flushes, layouts, pending,
  lines: lines.length,
  first: lines[0],
  last: lines[lines.length - 1],
  cleared: (clearConsole(), flushConsole(), consolePane.textContent),
}));
""" % block
        out = subprocess.run(["node", "-e", harness], capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        res = json.loads(out.stdout)

        # Bounded: 50k lines in, at most CONSOLE_MAX_LINES kept, newest ones.
        self.assertLessEqual(res["lines"], 2000)
        self.assertEqual(res["last"], "line 49999")
        self.assertNotEqual(res["first"], "line 0")
        # Batched: one timer covers the whole burst, so 50k lines cost one
        # repaint and one layout, not 50k of each.
        self.assertTrue(res["pending"], "flush must be deferred, not per line")
        self.assertEqual(res["flushes"], 1)
        self.assertEqual(res["layouts"], 1)
        self.assertEqual(res["cleared"], "")


class TestLaserPortPairsWithTheAgent(unittest.TestCase):
    """The LiDAR's tty can be derived instead of left blank: the MCU's is
    already configured, and the two boards never share a device. Whichever one
    the agent took, the LiDAR is on the other -- ttyUSB1 beside an agent on
    ttyUSB0, ttyUSB0 when the agent is a native-CDC board on ttyACM0."""

    @staticmethod
    def _port_block():
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js"), encoding="utf-8") as f:
            src = f.read()
        start = src.index("function laserPortPairedWithAgent")
        end = src.index("\n}\n", src.index("function laserDefaultPort", start)) + 3
        return src, src[start:end]

    def test_a_saved_port_still_outranks_the_guess(self):
        """Precedence is load-bearing: a port the user or an import actually
        recorded must not be replaced by a derived one."""
        _, block = self._port_block()
        self.assertIn("c.laser_serial_port || laserPortPairedWithAgent(c) || meta.symlink", block)

    @unittest.skipUnless(shutil.which("node"), "node is required to run app.js")
    def test_the_pairing_resolves_the_way_the_rig_is_wired(self):
        _, block = self._port_block()
        harness = """
%s
const state = {};
const run = (config, meta) => { state.config = config; return laserDefaultPort(meta); };
const ld = { symlink: "/dev/ldlidar" };
console.log(JSON.stringify({
  beside_usb0: run({ agent_device: "/dev/ttyUSB0" }, ld),
  beside_acm0: run({ agent_device: "/dev/ttyACM0" }, ld),
  beside_usb1: run({ agent_device: "/dev/ttyUSB1" }, ld),
  saved_wins: run({ agent_device: "/dev/ttyUSB0",
                    laser_serial_port: "/dev/serial/by-id/keep-me" }, ld),
  over_wifi: run({ agent_transport: "udp4", agent_device: "/dev/ttyUSB0" }, ld),
  unknown_device: run({ agent_device: "/dev/rfcomm0" }, ld),
}));
""" % block
        out = subprocess.run(["node", "-e", harness], capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        res = json.loads(out.stdout)

        self.assertEqual(res["beside_usb0"], "/dev/ttyUSB1")
        self.assertEqual(res["beside_acm0"], "/dev/ttyUSB0")
        self.assertEqual(res["beside_usb1"], "/dev/ttyUSB2")
        self.assertEqual(res["saved_wins"], "/dev/serial/by-id/keep-me")
        # No serial port is held over WiFi, so there is nothing to pair against
        # and a stale agent_device must not push the LiDAR a slot along.
        self.assertEqual(res["over_wifi"], "/dev/ldlidar")
        self.assertEqual(res["unknown_device"], "/dev/ldlidar")


class TestRos2InstallFallsBackToTesting(unittest.TestCase):
    """A distro is not in the `ros2` repo for every Ubuntu that supports it.

    On 26.04 (resolute) the ros2 repo carries 2563 ros-lyrical-* packages and
    zero ros-rolling-*; the 2205 rolling ones live in ros2-testing while the
    new Ubuntu is being brought up. 1-Click SLAM on a fresh Rolling box died at
    "E: Unable to locate package ros-rolling-ros-base", which reads like a
    broken box rather than a repo that does not carry it yet.
    """

    def test_install_cmd_enables_ros2_testing_when_package_is_absent(self):
        cmd = server.build_ros2_install_cmd("rolling")
        self.assertIn("ros2-testing", cmd)
        # Guarded: the fallback must not fire when the package is present, or
        # every distro silently starts installing from a testing repo.
        self.assertIn("apt-cache policy ros-rolling-ros-base", cmd)
        self.assertIn("Candidate: [0-9]", cmd)
        # Derived from the file ros2-apt-source installs, so the signing key
        # and suite come along instead of being hand-written.
        self.assertIn("/etc/apt/sources.list.d/ros2.sources", cmd)

    def test_install_cmd_is_valid_shell(self):
        for distro in server.SUPPORTED_DISTROS:
            cmd = server.build_ros2_install_cmd(distro)
            res = subprocess.run(["bash", "-n", "-c", cmd],
                                 capture_output=True, text=True)
            self.assertEqual(res.returncode, 0,
                             f"{distro}: {res.stderr}")


class TestSourceBuildsHaveARosEnvironment(unittest.TestCase):
    """Source fallbacks must source ROS 2 before rosdep and colcon.

    Console runs commands through `bash -lc`, and a fresh box has nothing
    sourced in its profile. On the first Rolling box the nav2_bringup source
    build therefore ran with no ROS environment: rosdep reported "ROS distro is
    not set ... Cannot locate rosdep definition for [ament_cmake]" and colcon
    died on "Could not find a package configuration file provided by
    ament_cmake". It had only ever worked on boxes where something else had
    already sourced setup.bash.
    """

    def test_source_build_sources_ros_and_names_the_distro(self):
        for pkg in server.SOURCE_FALLBACK_REPOS:
            cmd = server.build_source_package_cmd(pkg, distro="rolling", ws="/tmp/ws")
            self.assertIn("source /opt/ros/rolling/setup.bash", cmd, pkg)
            self.assertIn("export ROS_DISTRO=rolling", cmd, pkg)
            # rosdep cannot resolve ament_cmake without being told the distro.
            self.assertIn("--rosdistro rolling", cmd, pkg)
            # Packages built earlier in the same chain must be visible.
            self.assertIn("/tmp/ws/install/setup.bash", cmd, pkg)
            res = subprocess.run(["bash", "-n", "-c", cmd], capture_output=True, text=True)
            self.assertEqual(res.returncode, 0, f"{pkg}: {res.stderr}")

    def test_slam_toolbox_can_be_built_from_source(self):
        """Rolling on 26.04 publishes the nav2 components and
        robot_localization but neither slam_toolbox nor nav2_bringup. Without a
        source fallback, ensureRosPackages fell through to apt, hit "E: Unable
        to locate package ros-rolling-slam-toolbox", warned, and launched SLAM
        anyway -- a stack with no mapper in it."""
        self.assertIn("slam_toolbox", server.SOURCE_FALLBACK_REPOS)
        cmd = server.build_source_package_cmd("slam_toolbox", distro="rolling", ws="/tmp/ws")
        self.assertIn("slam_toolbox", cmd)
        self.assertIn("colcon build", cmd)


class TestSlamDoesNotLaunchWithoutItsPackages(unittest.TestCase):
    """1-Click must not bring up SLAM when the mapper does not exist.

    Rolling on Ubuntu 26.04 publishes 2205 packages of which exactly three are
    nav2-* (all TurtleBot sim assets); slam_toolbox and nav2_bringup are not
    published at all, and slam_toolbox cannot be source-built there either
    because its own dependencies (nav2_map_server) are missing too. Console
    used to warn "Launching anyway..." and start the stack regardless, which
    came up with no mapper and nothing to explain why.
    """

    def setUp(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "web", "app.js"), encoding="utf-8") as f:
            self.js = f.read()

    def test_prerequisites_are_re_checked_not_assumed_from_exit_codes(self):
        # apt can exit non-zero having installed most of the set, and a source
        # build can exit 0 having built nothing ("Summary: 0 packages
        # finished"), so the outcome has to be read back from disk.
        self.assertIn("stillMissing", self.js)
        self.assertFalse("Launching anyway..." in self.js,
                         "a code path still promises to launch after a failed "
                         "prerequisite install")

    def test_missing_slam_packages_abort_the_chain(self):
        self.assertIn("Cannot start ${label}", self.js)
        self.assertIn("prerequisites unavailable for", self.js)


class TestUnbuiltSourceTreeIsNotInstalled(unittest.TestCase):
    """A clone in <ws>/src is not a usable package.

    A failed source build leaves its clone behind. Counting that as installed
    made the next 1-Click run report the package satisfied and launch without
    it -- exactly the false green a fresh-box test exists to catch. Seen on
    Rolling, where slam_toolbox failed on its own missing dependencies and the
    leftover clone masked it on the following run.
    """

    def test_src_only_package_is_reported_missing(self):
        with tempfile.TemporaryDirectory() as ws:
            os.makedirs(os.path.join(ws, "src", "slam_toolbox"))
            installed, reason = server.check_sensor_driver_installed("slam_toolbox", ws=ws)
            self.assertFalse(installed, reason)

    def test_built_package_is_reported_installed(self):
        with tempfile.TemporaryDirectory() as ws:
            os.makedirs(os.path.join(ws, "install", "slam_toolbox"))
            installed, _ = server.check_sensor_driver_installed("slam_toolbox", ws=ws)
            self.assertTrue(installed)


class TestLifecycleBondTimeout(unittest.TestCase):
    """The lifecycle managers must tolerate a loaded machine.

    nav2 defaults bond_timeout to 4 s. On a fresh Lyrical box the SLAM group
    aborted with "Server map_saver was unable to be reached after 4.00s by
    bond" while that box was still finishing a source build and starting
    bringup, nav2 and slam_toolbox at once. Mapping itself was fine -- /scan,
    /odom at 50 Hz, /map and TF were all healthy -- but map_saver was left
    unmanaged, so Save Map could not work. 1-Click runs on exactly that kind of
    busy machine, so every shipped params file raises the timeout.
    """

    def test_every_nav2_params_file_waits_long_enough_for_services(self):
        """bt_navigator builds its tree at activation and waits for the services
        that tree calls. is_path_valid comes from planner_server, which on a
        fresh Lyrical box activated 2.3 s before bt_navigator gave up after its
        1.0 s default -- aborting the whole navigation group with "Error loading
        BT". Composition had hidden it by putting every node in one process."""
        import glob
        here = os.path.dirname(os.path.abspath(__file__))
        for path in sorted(glob.glob(os.path.join(here, "config", "nav2_*.yaml"))):
            with open(path) as f:
                cfg = yaml.safe_load(f)
            timeout = (cfg.get("bt_navigator") or {}).get("ros__parameters", {}).get(
                "wait_for_service_timeout")
            self.assertIsNotNone(timeout, f"{os.path.basename(path)}: no wait_for_service_timeout")
            self.assertGreaterEqual(
                timeout, 5000,
                f"{os.path.basename(path)}: {timeout} ms is too tight for an uncomposed start")

    def test_every_nav2_params_file_raises_the_bond_timeout(self):
        import glob
        here = os.path.dirname(os.path.abspath(__file__))
        files = sorted(glob.glob(os.path.join(here, "config", "nav2_*.yaml")))
        self.assertTrue(files, "no nav2 params files found")
        for path in files:
            with open(path) as f:
                cfg = yaml.safe_load(f)
            for mgr in ("lifecycle_manager_slam", "lifecycle_manager_navigation"):
                params = (cfg.get(mgr) or {}).get("ros__parameters") or {}
                timeout = params.get("bond_timeout")
                self.assertIsNotNone(timeout, f"{os.path.basename(path)}: {mgr} has no bond_timeout")
                self.assertGreaterEqual(
                    timeout, 10.0,
                    f"{os.path.basename(path)}: {mgr} bond_timeout {timeout} is too tight for a loaded box")


class TestNav2RunsUncomposed(unittest.TestCase):
    """nav2 must not be brought up inside a single composed container.

    Composed bringup loads nodes via service calls. On a fresh Lyrical box --
    one that had just installed ROS 2, built a workspace and cloned drivers --
    those loads timed out: controller_server and the costmap filter *mask*
    servers came up, five load_node requests failed, and planner_server,
    bt_navigator, the filter *info* servers and lifecycle_manager_navigation
    never existed at all, leaving the keepout/speed managers waiting forever.
    Separate processes also stop one bad parameter from segfaulting the
    container and taking every nav2 node down with it.
    """

    def test_launcher_disables_composition(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "launch_nav2.py"), encoding="utf-8") as f:
            src = f.read()
        self.assertIn("'use_composition': 'False'", src)


if __name__ == "__main__":
    unittest.main()

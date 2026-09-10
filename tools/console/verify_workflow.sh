#!/usr/bin/env bash
# Console workflow verification for one ROS 2 distro environment.
# Usage: verify_console.sh <distro>   (run inside a ros:<distro>-ros-base container/box)
set -o pipefail
DISTRO="${1:-jazzy}"
REPO="${REPO:-/repo}"
PASS=0; FAIL=0
ok(){ echo "  PASS  $*"; PASS=$((PASS+1)); }
no(){ echo "  FAIL  $*"; FAIL=$((FAIL+1)); }
step(){ echo; echo "== $* =="; }

export ROS_DISTRO="$DISTRO"
if [ -f "/opt/ros/$DISTRO/setup.bash" ]; then set +u; source "/opt/ros/$DISTRO/setup.bash" >/dev/null 2>&1 || true; set -o pipefail; fi

step "unit suites"
( cd "$REPO/tools/console" && python3 test_console.py 2>&1 | tail -3 | grep -q "^OK" ) \
  && ok "linorobot2_console test_console.py" || no "test_console.py"
if [ -d "$REPO/../linorobot2_hardware/tools/robot_config_engine" ]; then
  ( cd "$REPO/../linorobot2_hardware/tools/robot_config_engine" && python3 test_config_engine.py 2>&1 | tail -3 | grep -q "^OK" ) \
    && ok "robot_config_engine test_config_engine.py" || no "test_config_engine.py"
fi

step "console server + REST smoke ($DISTRO)"
cd "$REPO/tools/console/web"
rm -f console_nav2_*.yaml console_ekf.yaml console_slam.yaml console_config.json
python3 server.py 8199 >/tmp/console_srv.log 2>&1 & SRV=$!
for i in $(seq 1 40); do curl -s -o /dev/null http://127.0.0.1:8199/api/status && break; sleep 0.25; done
G(){ curl -s "http://127.0.0.1:8199$1"; }
P(){ curl -s -XPOST "http://127.0.0.1:8199$1" -H 'Content-Type: application/json' -d "$2"; }

G /api/status | grep -q '"ros_distro"' && ok "/api/status" || no "/api/status"
G /api/sensors | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["laser"]["ldlidar"]["models"];assert d["depth"]["realsense"]' \
  && ok "/api/sensors registry" || no "/api/sensors"
G /api/serial_ports | python3 -c 'import sys,json;json.load(sys.stdin)["ports"]' && ok "/api/serial_ports" || no "/api/serial_ports"
G "/api/nav2_config?distro=$DISTRO" | python3 -c "import sys,json;d=json.load(sys.stdin);assert d['depth_pointcloud_active'] in (True,False,None);assert 'config' in d" \
  && ok "/api/nav2_config?distro=$DISTRO" || no "/api/nav2_config"
P /api/nav2_config/costmap_sources "{\"distro\":\"$DISTRO\",\"depth_enabled\":false}" | grep -q '"depth_pointcloud_active": false' \
  && ok "costmap_sources gate off" || no "costmap_sources gate off"
P /api/nav2_config/costmap_sources "{\"distro\":\"$DISTRO\",\"depth_enabled\":true}" | grep -q '"depth_pointcloud_active": true' \
  && ok "costmap_sources gate on" || no "costmap_sources gate on"
P /api/ai/tune '{"prompt":"robot overshoots the goal and blows past","base":"2wd"}' | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["nav2_patch"].get("max_decel_x")==2.8' \
  && ok "AI tune overshoot" || no "AI tune"
P /api/params/export "{\"dest_dir\":\"/tmp/exp_$DISTRO\",\"distros\":[\"$DISTRO\"]}" | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["count"]>=6;assert any("nav2.launch.py" in f["path"] for f in d["files"])' \
  && ok "/api/params/export bundle" || no "/api/params/export"
python3 -c "compile(open('/tmp/exp_$DISTRO/launch/nav2.launch.py').read(),'x','exec')" && ok "exported launcher compiles" || no "exported launcher"
P /api/params/merge "{\"kind\":\"nav2\",\"distro\":\"$DISTRO\",\"target\":\"template\",\"dry_run\":true}" | grep -q '"status": "dry-run"' \
  && ok "/api/params/merge dry-run" || no "/api/params/merge"
P /api/sensor_install_cmd '{"kind":"laser","key":"ldlidar","skip_udev":true}' | grep -q ldlidar_stl_ros2 \
  && ok "/api/sensor_install_cmd" || no "/api/sensor_install_cmd"

# Repo-based robot config: config/<robot>_config.yaml is the single source of truth.
G /api/status | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["robot_name"];assert d["robot_config_path"].endswith("_config.yaml");assert "/config/" in d["robot_config_path"]' \
  && ok "/api/status reports repo-based robot config" || no "/api/status robot_config_path"
G /api/robots | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["active"];assert any(r["active"] for r in d["robots"])' \
  && ok "/api/robots lists the active robot" || no "/api/robots"
G /api/gitinfo | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["branch"];assert isinstance(d["branches"],list);assert isinstance(d["commits"],list)' \
  && ok "/api/gitinfo branch + branches" || no "/api/gitinfo"
P /api/robot/select '{"name":"verify_bot"}' | python3 -c 'import sys,json;d=json.load(sys.stdin);assert d["active"]=="verify_bot";assert d["robot_config_path"].endswith("verify_bot_config.yaml")' \
  && ok "/api/robot/select creates + activates a robot" || no "/api/robot/select"
P /api/robot/select '{"name":"Bad Name"}' | grep -q '"error"' \
  && ok "/api/robot/select rejects invalid names" || no "/api/robot/select validation"
P /api/gitinfo/branch '{"branch":"bad branch~!"}' | grep -q '"error"' \
  && ok "/api/gitinfo/branch rejects invalid names" || no "/api/gitinfo/branch validation"
P /api/robot/select '{"name":"linorobot2"}' >/dev/null
rm -f "$REPO/config/verify_bot_config.yaml" "$REPO/config/.active_robot"
kill "$SRV" 2>/dev/null || true; wait "$SRV" 2>/dev/null || true

step "console docker-compose ($DISTRO)"
CD="$REPO/tools/console/docker"
if command -v docker >/dev/null 2>&1; then
  ( cd "$CD" && printf 'BASE_IMAGE=jazzy\nROBOT_BASE=2wd\nLASER_SENSOR=\nDEPTH_SENSOR=\n' > .env \
    && printf 'services:\n  bringup:\n    devices: ["/dev/ttyACM0:/dev/ttyACM0"]\n' > devices.generated.yaml \
    && docker compose --env-file .env -f docker-compose.yaml -f devices.generated.yaml config >/dev/null 2>&1 \
    && grep -q "tools/console/launch_nav2.py" docker-compose.yaml ; rc=$? ; rm -f .env devices.generated.yaml ; exit $rc ) \
    && ok "console compose validates + uses launch_nav2.py" || no "console compose"
  grep -q "scan pointcloud" "$REPO/linorobot2_navigation/config/navigation_jazzy.yaml" \
    && no "upstream navigation_jazzy.yaml must NOT have the console pointcloud edit" \
    || ok "upstream nav configs untouched"
else
  echo "  SKIP  no docker in this env"
fi

step "launch-file introspection ($DISTRO)"
if command -v ros2 >/dev/null && python3 -c "import launch,launch_ros" 2>/dev/null; then
  for lf in "$REPO/tools/console/launch_nav2.py" "$REPO/linorobot2_navigation/launch/navigation.launch.py" "/tmp/exp_$DISTRO/launch/nav2.launch.py"; do
    python3 - "$lf" <<'PY' && ok "generate_launch_description $(basename "$lf")" || no "launch parse $(basename "$lf")"
import importlib.util,sys
s=importlib.util.spec_from_file_location("m",sys.argv[1]);m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
m.generate_launch_description()
PY
  done
  grep -q depth_costmap "$REPO/linorobot2_navigation/launch/navigation.launch.py" && no "upstream navigation.launch.py must NOT have depth_costmap" || ok "upstream launcher untouched (no depth_costmap)"
  grep -qE "FindPackageShare\(.linorobot2_navigation.\), .launch." "$REPO/tools/console/launch_nav2.py" \
    && no "launch_nav2.py still includes the linorobot2_navigation launch file" \
    || ok "launch_nav2.py decoupled (no linorobot2_navigation launch include)"
  grep -q "nav2_bringup.), .launch., .bringup_launch.py" "$REPO/tools/console/launch_nav2.py" \
    && ok "launch_nav2.py includes nav2_bringup/bringup_launch.py directly" \
    || no "launch_nav2.py should include nav2_bringup/bringup_launch.py"
else
  echo "  SKIP  no launch/launch_ros in this env"
fi

step "RESULT ($DISTRO):  $PASS passed, $FAIL failed"
exit $((FAIL>0))

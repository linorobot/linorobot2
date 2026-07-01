#!/bin/bash
# Lidar + IMU SLAM without wheel encoders -- NATIVE (no Docker) version.
#
# Pipeline:
#   lidar ---------------------------------> /scan
#   rf2o_laser_odometry --(odom->base_footprint)-> translation+yaw from scan match
#   mpu6050_imu ---------------------------> /imu/data (available for later EKF)
#   slam_toolbox --(map->odom)-------------> builds map, corrects drift
#
# WHY rf2o: slam_toolbox only adds a scan to the map once *odometry* says the
# robot moved minimum_travel_distance (0.5 m). With no wheel encoders there is
# no such odometry, so the map never grows. rf2o derives real x/y/yaw motion by
# matching consecutive lidar scans and publishes the odom->base_footprint TF --
# giving slam_toolbox the motion it needs. The IMU just publishes /imu/data and
# does NOT own the odom TF (rf2o does), so the two don't fight.
#
# Do NOT run this alongside the stock linorobot2 bringup -- its EKF also
# publishes odom->base_footprint and the two will fight.
#
# When the Nucleo + AK10-9 wheel odometry arrives, replace rf2o with wheel
# odometry (odom/unfiltered) and let the stock EKF fuse it with /imu/data.

set -e
source /opt/ros/jazzy/setup.bash
source /home/jetson1/linorobot2_ws/install/setup.bash
set +e

# Sanity check: the three packages this script drives must be built.
for pkg in sllidar_ros2 mpu6050_imu rf2o_laser_odometry; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
        echo "[slam_imu] ERROR: package '$pkg' not found in the workspace." >&2
        echo "[slam_imu] Build it in ~/linorobot2_ws then re-source install/setup.bash." >&2
        exit 1
    fi
done

# Prefer a stable udev symlink for the lidar if one exists.
LIDAR_PORT="${LIDAR_PORT:-}"
if [ -z "$LIDAR_PORT" ]; then
    if [ -e /dev/rplidar ]; then
        LIDAR_PORT=/dev/rplidar
    else
        LIDAR_PORT=/dev/ttyUSB0
    fi
fi
echo "[slam_imu] lidar port: $LIDAR_PORT"

cleanup() {
    kill $(jobs -p) 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT

# RPLIDAR A3 @ 256000 baud, scans framed as 'laser'.
ros2 launch sllidar_ros2 sllidar_a3_launch.py \
    serial_port:="$LIDAR_PORT" \
    serial_baudrate:=256000 \
    frame_id:=laser &

# IMU: publishes /imu/data only. rf2o owns odom->base_footprint, so the IMU
# must NOT publish its own odom TF (they would conflict).
ros2 launch mpu6050_imu imu.launch.py publish_odom_tf:=false &

# rf2o laser odometry: matches consecutive scans -> odom->base_footprint TF.
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
    -p laser_scan_topic:=/scan \
    -p odom_topic:=/odom_rf2o \
    -p publish_tf:=true \
    -p base_frame_id:=base_footprint \
    -p odom_frame_id:=odom \
    -p init_pose_from_topic:='""' \
    -p freq:=10.0 &

# Mounting offsets (x y z yaw pitch roll, meters/radians). Replace the zeros
# with the real lidar/IMU positions on the robot. Identity is fine for testing.
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --frame-id base_footprint --child-frame-id laser &
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --frame-id base_footprint --child-frame-id imu_link &

sleep 3

ros2 launch slam_toolbox online_async_launch.py &

# Visualization.
#
# Over SSH there is usually no display ($DISPLAY empty), so rviz2 cannot open a
# window and would crash this script. In that case run headless: keep the SLAM
# nodes alive in the foreground and view the map from your laptop instead (see
# below). With a real display (local screen, or ssh -X), rviz runs as before --
# closing it tears everything down.
RVIZ_CFG="$(dirname "$0")/slam_imu.rviz"

if [ -n "$DISPLAY" ] && command -v rviz2 >/dev/null 2>&1; then
    # Local/forwarded display available: rviz in the foreground.
    if [ -f "$RVIZ_CFG" ]; then
        rviz2 -d "$RVIZ_CFG"
    else
        rviz2
    fi
else
    echo
    echo "[slam_imu] No display detected (\$DISPLAY is empty) -- running headless."
    echo "[slam_imu] SLAM is mapping. To watch the map live from your LAPTOP:"
    echo "[slam_imu]   1. Same network as the robot, and ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    echo "[slam_imu]   2. source /opt/ros/jazzy/setup.bash"
    echo "[slam_imu]   3. rviz2 -d slam_imu.rviz   (copy slam_imu.rviz from this repo)"
    echo "[slam_imu] Save the map when done, then Ctrl-C here to stop SLAM."
    echo
    # Block in the foreground so the trap tears everything down on Ctrl-C.
    wait
fi

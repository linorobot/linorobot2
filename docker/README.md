## Hardware

### 1. Configure for hardware

Edit `docker/.env` and set:

```
BASE_IMAGE=hardware
ROBOT_BASE=<robot_type>      # 2wd, 4wd, or mecanum
LASER_SENSOR=<laser_sensor>  # e.g. ld06, a1, ydlidar  (leave blank if not used)
DEPTH_SENSOR=<depth_sensor>  # e.g. realsense, oakd     (leave blank if not used)
```

### 2. Build the image

```bash
cd linorobot2/docker
docker compose build
```

### 3. Install udev rules on the host

The Docker image already contains the sensor drivers (installed during build). To create
the `/dev/<sensor>` symlinks on the **host** machine, run `install.bash` with `--udev-only`:

```bash
# From the linorobot2 repo root on the host:
bash install.bash --laser <laser_sensor>
# and/or
bash install.bash --depth <depth_sensor>
```

Use `--udev-only` if you only want the udev rules without reinstalling drivers:

```bash
bash install.bash --laser <laser_sensor> --udev-only
```

### 4. Verify the device symlink

Plug in the sensor, then confirm the udev symlink exists on the host:

```bash
ls /dev/<sensor_name>
# Example:
ls /dev/ldlidar
```

### 5. Update the bringup service devices

Edit `docker/docker-compose.yaml` and update the `bringup` service's `devices` section
to map the correct host device into the container:

```yaml
  bringup:
    ...
    devices:
      - /dev/ldlidar:/dev/ldlidar
```

### 6. Run the robot

```bash
docker compose up bringup
```

---

## Simulation

### 1. Install Tmuxinator

Follow the installation instructions for Tmuxinator [here](https://github.com/tmuxinator/tmuxinator?tab=readme-ov-file#installation)

### 2. Running the demos using Tmuxinator and Docker

#### 2.1. First, export the tmuxinator project path:

```
cd linorobot2/docker/demos
export TMUXINATOR_CONFIG=$PWD
```

#### 2.2. Running the Nav2 demo in Gazebo::

```
tmuxinator start sim
```

To stop the simulation, stop any process by pressing Ctrl + C and run:
```
tmuxinator stop sim
```

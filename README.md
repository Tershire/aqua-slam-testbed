# aqua-slam-testbed

Stonefish simulation environment for AQUA-SLAM baseline testing.
Runs as a Docker container and communicates with the AQUA-SLAM container via ROS2 DDS (`network_mode: host`, same `ROS_DOMAIN_ID`).

---

## Overview

```
aqua-slam-testbed/                ← this repo (simulator)
  docker/
    Dockerfile                    ROS2 Jazzy + Stonefish 1.6
    docker-compose.yml
  scenarios/
    simple_tank.scn               20×8×4 m tank, Girona500 + stereo/IMU/DVL/pressure
  launch/
    sim.launch.py

AQUA-SLAM (simulation/stonefish branch)
  docker/ros2_jazzy/
    docker-compose.yml            image: aqua-slam:ros2-jazzy-sim-dev / container: aqua_slam_ros2_sim_dev
  launch/
    stonefish_sim.launch.py       SLAM launch for simulation
  data/
    stonefish_sim.yaml            sensor config matched to simple_tank.scn
  scripts/
    sim_dvl_converter.py          stonefish_ros2/DVL → nav_msgs/Odometry
```

---

## Prerequisites

| Item | Notes |
|------|-------|
| NVIDIA GPU | Required for `stonefish_simulator` (GPU rendering) |
| [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) | Enables GPU passthrough to Docker |
| Docker + Docker Compose v2 | `docker compose` (not `docker-compose`) |

### Allow X11 from Docker (once per session)
```bash
xhost +local:docker
```

---

## Build

```bash
git clone https://github.com/Tershire/aqua-slam-testbed.git
cd aqua-slam-testbed/docker
docker compose build
```

Build takes ~10–20 min (compiles Stonefish from source).

---

## Run

### 1. Start the simulator

```bash
cd aqua-slam-testbed/docker
docker compose up -d
docker exec -it aqua_slam_testbed bash

# Inside container:
ros2 launch /ros2_ws/launch/sim.launch.py
```

Optional launch arguments:
```bash
ros2 launch /ros2_ws/launch/sim.launch.py \
  simulation_rate:=1000.0 \
  window_width:=1280 \
  window_height:=720 \
  quality:=medium
```

### 2. Start AQUA-SLAM (separate terminal)

```bash
cd aqua_slam_ws/src/AQUA-SLAM/docker/ros2_jazzy
docker compose up -d
docker exec -it aqua_slam_ros2_sim_dev bash

# Inside container:
ros2 launch aqua_slam stonefish_sim.launch.py
```

### 3. Verify topic flow

```bash
# In either container:
ros2 topic list
ros2 topic hz /girona500/camera_left/image_color
ros2 topic hz /girona500/imu/data
ros2 topic hz /bluerov2/dvl
```

---

## Sensor Topics

| Sensor | Topic | Type | Rate |
|--------|-------|------|------|
| Left camera | `/girona500/camera_left/image_color` | `sensor_msgs/Image` | 20 Hz |
| Right camera | `/girona500/camera_right/image_color` | `sensor_msgs/Image` | 20 Hz |
| IMU | `/girona500/imu/data` | `sensor_msgs/Imu` | 200 Hz |
| DVL (raw) | `/girona500/dvl` | `stonefish_ros2/DVL` | 5 Hz |
| DVL (SLAM) | `/bluerov2/dvl` | `nav_msgs/Odometry` | 5 Hz |
| Pressure | `/girona500/pressure` | `sensor_msgs/FluidPressure` | 10 Hz |

`/bluerov2/dvl` is published by `sim_dvl_converter` running inside the AQUA-SLAM container.

---

## Scenario: simple_tank.scn

- **Tank**: 20 m (x/forward) × 8 m (y) × 4 m (z/depth)
- **Robot**: Girona500 simplified cylinder, starts at (2, 0, 2) facing forward
  - TODO: replace with BlueROV2 model
- **Stereo baseline**: 0.12 m (matches `Camera.bf / Camera.fx` in `stonefish_sim.yaml`)
- **Sonar**: removed pending FLS vs MSIS decision; Water Linked 3D-15 may need a custom sensor class

---

## Known Issues / TODO

- [ ] Sonar sensor spec format for Stonefish 1.6 not yet resolved → sensor omitted
- [ ] `T_dvl_c` sign convention needs verification against `stonefish_ros2` DVL output frame
- [ ] Wall colors are plain — consider adding decals/textures for richer ORB features
- [ ] Replace Girona500 with BlueROV2 model (team task)
- [ ] `sim_dvl_converter.py` message field names should be verified with `ros2 interface show stonefish_ros2/msg/DVL`

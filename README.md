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

## SLAM Initialization Test (forward straight-line)

### Test protocol

```bash
# Terminal 1 — simulator + DVL converter
docker exec -it aqua_slam_testbed bash
ros2 launch /ros2_ws/launch/sim.launch.py

# Terminal 2 — SLAM
docker exec -it aqua_slam_ros2_sim_dev bash
ros2 launch aqua_slam stonefish_sim.launch.py

# Terminal 3 — record ground truth + SLAM output
docker exec -it aqua_slam_ros2_sim_dev bash
ros2 bag record /tf /bluerov2/dvl \
  /girona500/camera_left/image_color \
  /girona500/imu/data \
  -o ~/slam_test_$(date +%Y%m%d_%H%M%S)

# Terminal 4 — run forward test (wait ~5 s after SLAM launches)
docker exec -it aqua_slam_testbed bash
python3 /ros2_ws/scripts/forward_test.py --thrust 0.3 --cruise 20 --ramp 3
```

### Thruster setpoint

| Parameter | Default | Notes |
|-----------|---------|-------|
| `--thrust` | 0.3 | Normalized thrust 0–1. Start low; adjust if velocity is too slow/fast. |
| `--cruise` | 20 s | At ~0.3 thrust the robot should travel ~5–8 m (enough for IMU init at 3 m). |
| `--ramp`   | 3 s  | Smooth trapezoidal ramp; reduces IMU integration error at start/stop. |

Profile shape:
```
thrust
  0.3 |      ___________
      |     /           \
    0 |____/             \____
         |ramp| cruise  |ramp|
```

### Ground truth comparison

Stonefish publishes the robot's true pose on `/tf` (frame: `world` → `girona500/Vehicle`).
After the bag is recorded, extract and compare with the SLAM odometry output.

---

## Known Issues / TODO

- [ ] Sonar sensor spec format for Stonefish 1.6 not yet resolved → sensor omitted
- [ ] `T_dvl_c` sign convention needs verification against `stonefish_ros2` DVL output frame
- [ ] Wall colors are plain — consider adding decals/textures for richer ORB features
- [ ] Replace Girona500 with BlueROV2 model (team task)
- [ ] `sim_dvl_converter.py` message field names should be verified with `ros2 interface show stonefish_ros2/msg/DVL`

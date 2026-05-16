# aqua-slam-testbed

Stonefish simulation environment for AQUA-SLAM baseline testing.

This repository provides the simulator container (`aqua_slam_testbed`).
Full setup and usage documentation is in the AQUA-SLAM repository:

- [Installation guide](https://github.com/Tershire/AQUA-SLAM/blob/simulation/stonefish/documents/installation_guide.md)
- [Simulation guide](https://github.com/Tershire/AQUA-SLAM/blob/simulation/stonefish/documents/simulation_guide.md)

---

## Quick start

```bash
# Allow X11 (once per session)
xhost +local:docker

# Build (first time only, ~10–20 min)
cd docker
docker compose build

# Run
docker compose up -d
docker exec -it aqua_slam_testbed bash
ros2 launch /ros2_ws/launch/sim.launch.py
```

---

## Sensors

| Sensor | Stonefish type | Topic | Message type | Rate |
|--------|---------------|-------|--------------|------|
| Camera left | camera | `/girona500/camera_left/image_color` | `sensor_msgs/Image` | 20 Hz |
| Camera right | camera | `/girona500/camera_right/image_color` | `sensor_msgs/Image` | 20 Hz |
| IMU | imu | `/girona500/imu/data` | `sensor_msgs/Imu` | 200 Hz |
| DVL (raw) | dvl | `/girona500/dvl` | `stonefish_ros2/DVL` | 5 Hz |
| DVL (SLAM) | — | `/bluerov2/dvl` | `nav_msgs/Odometry` | 5 Hz |
| Pressure | pressure | `/girona500/pressure` | `sensor_msgs/FluidPressure` | 10 Hz |
| Sonar 3D-15 (raw) | depthcamera | `/girona500/sonar_3d15/raw/image_depth` | `sensor_msgs/Image` (32FC1) | 5 Hz |
| Sonar 3D-15 (SLAM) | — | `/sonar_3d15/points` | `sensor_msgs/PointCloud2` | 5 Hz |

`/bluerov2/dvl` and `/sonar_3d15/points` are published by converter nodes inside the testbed
container (`sim_dvl_converter.py`, `sim_sonar_3d15_converter.py`).

### Sonar 3D-15 implementation note

The sonar is currently approximated with a Stonefish `depthcamera` (90° H × 40° V, 256×94 px,
0.2–15 m) rather than `multibeam2d`. The `depthcamera` outputs a 32FC1 z-depth image which
`sim_sonar_3d15_converter.py` backprojects to a 3-D point cloud in the sensor RDF frame,
applies the sensor→body FLU rotation, looks up the `aqua_slam → bluerov/base_link` TF, and
publishes the result in the `aqua_slam` world frame.

Acoustic effects added by the converter:
- **Dropout** — range-dependent Rayleigh, P(survive) = exp(−0.02 r)
- **Angular jitter** — Gaussian beam uncertainty: 0.425° H / 0.80° V (1-σ, proportional to range)
- **Intensity** — Rayleigh speckle field, σ(r) = max(0.05, 0.8 − 0.04 r)

Simulated specs match the 3D-15 low-frequency datasheet:
90° H × 40° V FOV, 0.2–15 m range, 5 Hz.

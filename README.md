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
| Sonar 3D-15 (raw) | multibeam2d | `/girona500/sonar_3d15/raw` | `sensor_msgs/PointCloud2` | 5 Hz |
| Sonar 3D-15 (SLAM) | — | `/sonar_3d15/points` | `sensor_msgs/PointCloud2` | 5 Hz |

`/bluerov2/dvl` and `/sonar_3d15/points` are published by converter nodes inside the testbed
container (`sim_dvl_converter.py`, `sim_sonar_3d15_converter.py`).

### Sonar 3D-15 coordinate frame

The `multibeam2d` sensor publishes in its own sensor frame (`frame_id: sonar_3d15`).
Stonefish provides the TF chain `sonar_3d15 → Vehicle → world`.

Sensor frame axes (after `rpy="1.5708 0.0 1.5708"` — same convention as stereo cameras):

```
z  →  forward  (NED x, boresight)
x  →  right    (NED y, horizontal sweep)
y  →  down     (NED z, vertical extent)
```

The `/sonar_3d15/points` cloud adds an `intensity` field (Rayleigh acoustic speckle,
range-attenuated) on top of the raw geometry.
Simulated specs match the 3D-15 low-frequency datasheet:
90° H × 40° V FOV, 256 × 67 beams, 0.2–15 m range.

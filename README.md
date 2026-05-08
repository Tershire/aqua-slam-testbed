# aqua-slam-testbed

Stonefish simulation environment for AQUA-SLAM baseline testing.

This repository provides the simulator container (`aqua_slam_testbed`).
Full setup and usage documentation is in the AQUA-SLAM repository:

- [Installation guide](https://github.com/Tershire/AQUA-SLAM/blob/simulation/stonefish/documents/installation_guide.md)
- [Simulation guide](https://github.com/Tershire/AQUA-SLAM/blob/simulation/stonefish/documents/simulation_guide.md)

---

## Quick start

```bash
# Allow X11
xhost +local:docker

# Build (first time only, ~10–20 min)
cd docker
docker compose build

# Run
docker compose up -d
docker exec -it aqua_slam_testbed bash
ros2 launch /ros2_ws/launch/sim.launch.py
```

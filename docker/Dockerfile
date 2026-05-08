FROM osrf/ros:jazzy-desktop

# Stonefish build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libglm-dev libsdl2-dev libfreetype-dev \
    libopengl-dev libgl1-mesa-dev libgles2-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# Build Stonefish library
RUN git clone --depth 1 https://github.com/patrykcieslak/stonefish.git /opt/stonefish \
    && cmake -S /opt/stonefish -B /opt/stonefish/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/stonefish/build --parallel $(nproc) \
    && cmake --install /opt/stonefish/build \
    && rm -rf /opt/stonefish/build

# Build stonefish_ros2 workspace
RUN mkdir -p /ros2_ws/src \
    && git clone --depth 1 https://github.com/patrykcieslak/stonefish_ros2.git /ros2_ws/src/stonefish_ros2

WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# ROS 2 Humble host-side stack for the Yahboom MicroROS V2 robot.
# Builds natively for arm64 on Apple Silicon.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Runtime packages: localization, SLAM, map saving, robot model, viewer bridge.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential python3-colcon-common-extensions python3-pip \
        ros-humble-robot-localization \
        ros-humble-slam-toolbox \
        ros-humble-nav2-map-server \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-tf2-ros ros-humble-tf2-tools \
        ros-humble-cv-bridge python3-opencv \
        ros-humble-teleop-twist-keyboard \
        ros-humble-foxglove-bridge \
        ros-humble-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# micro-ROS agent, built from source (same repos micro_ros_setup uses).
WORKDIR /opt/uros_ws
RUN mkdir -p src \
    && git clone --depth 1 -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent \
    && git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs \
    && source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Yahboom packages (bringup, robot model, interfaces, demos).
WORKDIR /root/microros_ws
COPY vendor/yahboom/src ./src
RUN source /opt/ros/humble/setup.bash \
    && source /opt/uros_ws/install/setup.bash \
    && colcon build \
    && rm -rf build log

# Use the project's tuned slam_toolbox params (compose mounts ./config at /work/config).
# A symlink, not a single-file bind mount, so edits on the Mac are always picked up.
RUN ln -sf /work/config/slam_toolbox.yaml \
      /root/microros_ws/install/microros_v2_bringup/share/microros_v2_bringup/config/mapper_params_online_async.yaml \
    && ln -sf /work/config/ekf.yaml \
      /root/microros_ws/install/microros_v2_bringup/share/microros_v2_bringup/config/ekf.yaml

# Unit tests for the tools (./ros python3 -m pytest /work/tests).
RUN apt-get update && apt-get install -y --no-install-recommends python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# robot_tool.py reads camera_ip from this file; compose mounts the real one.
RUN mkdir -p /root/microros_ws/.system_cfg /root/microros_ws/cache /root/map

RUN printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      'source /opt/uros_ws/install/setup.bash' \
      'source /root/microros_ws/install/setup.bash' \
      >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]

# ROS 2 host-side stack for the Yahboom MicroROS V2 robot.
# Builds natively for arm64 on Apple Silicon.
# ROS_DISTRO picks the release: humble (default, Ubuntu 22.04) or jazzy (Ubuntu 24.04).
# The official ros images also set ROS_DISTRO in the environment, so RUN steps use it.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Runtime packages: localization, SLAM, map saving, robot model, viewer bridge.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential python3-colcon-common-extensions python3-pip \
        ros-${ROS_DISTRO}-robot-localization \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-tools \
        ros-${ROS_DISTRO}-cv-bridge python3-opencv \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

# micro-ROS agent, built from source (same repos micro_ros_setup uses).
WORKDIR /opt/uros_ws
RUN mkdir -p src \
    && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent \
    && git clone --depth 1 -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Yahboom packages (bringup, robot model, interfaces, demos).
WORKDIR /root/microros_ws
COPY vendor/yahboom/src ./src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
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

# Nav2 (navigation on a saved map: AMCL, planners, controllers, behavior trees).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# robot_tool.py reads camera_ip from this file; compose mounts the real one.
RUN mkdir -p /root/microros_ws/.system_cfg /root/microros_ws/cache /root/map

RUN printf '%s\n' \
      'source /opt/ros/$ROS_DISTRO/setup.bash' \
      'source /opt/uros_ws/install/setup.bash' \
      'source /root/microros_ws/install/setup.bash' \
      >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]

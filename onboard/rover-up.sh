#!/bin/bash
# Start the rover's ROS stack on the Orange Pi (see onboard/launch/rover_pi.launch.py).
#   onboard/rover-up.sh [launch args...]     e.g. agent_transport:=tcp4 agent_port:=8888 nav:=true
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source "$HOME/uros_ws/install/setup.bash"          # micro_ros_agent
export ROS_DOMAIN_ID=77                             # the rover firmware's domain
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST      # keep DDS off Wi-Fi; Foxglove is the remote view
exec ros2 launch onboard/launch/rover_pi.launch.py urdf:="$HOME/lidarbot/private/robot.urdf" "$@"

#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/uros_ws/install/setup.bash
source /root/microros_ws/install/setup.bash
exec "$@"

#!/bin/bash
# Save as ros_entrypoint.sh in the same directory as your Dockerfile

set -e

# Source ROS 2 Rolling
source /opt/ros/rolling/setup.bash

# Source the workspace if it exists
if [ -f "/yolo_ws/install/setup.bash" ]; then
  source /yolo_ws/install/setup.bash
fi

exec "$@"

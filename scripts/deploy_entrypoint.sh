#!/bin/bash
# deploy_entrypoint.sh - Custom entrypoint for ORB-SLAM3 deploy container.
# Sets LD_LIBRARY_PATH for ORB-SLAM3/Pangolin and sources the ROS workspace.

# Source ROS2
source ${ROS_ROOT:?}/setup.bash

# Source ROS workspace if exists
if [[ ! -z "${ROS_WS}" && -d "${ROS_WS}/install" ]]; then
    source ${ROS_WS}/install/setup.bash
fi

# ORB-SLAM3 and Pangolin libraries
export LD_LIBRARY_PATH="/opt/orb_slam3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

# Register shared libraries
ldconfig 2>/dev/null || true

set -x
exec "$@"

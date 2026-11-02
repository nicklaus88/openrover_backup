#!/bin/bash

set -e


# ============================================================
# Shared ROS 2 environment
#
# Sets ROS_HOST_IP / ROS_DISCOVERY_SERVER / ROS_SUPER_CLIENT and defines
# the ros_discovery_server_* helpers. The identical file is sourced from
# ~/.bashrc so `docker compose exec ros2 bash` lands in the same
# environment this entrypoint builds.
# ============================================================

source /ros_env.sh


# ============================================================
# Discovery server
#
# Started here so that `docker compose up -d` on its own is enough -- the
# server is up before any node, launch file or exec'd shell exists. It
# listens on all interfaces, so it is reachable both on loopback (our own
# nodes) and on the host's wifi address (the TurtleBot 4, laptops).
#
# Not fatal if it fails: coming up with a shell and a clear warning beats
# a container that refuses to start.
# ============================================================

if ! ros_discovery_server_start; then
    echo "WARNING: discovery server (id 0) failed to start. Log follows:" >&2
    cat /tmp/discovery_server_0.log >&2 2>/dev/null || true
fi


# ============================================================
# Default working directory
# ============================================================

cd /workspace/ROS2-WorkSpace


# ============================================================
# Information
# ============================================================

ros_status


exec "$@"

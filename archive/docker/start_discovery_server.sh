#!/bin/bash
set -e

# Restarts the Fast DDS Discovery Server inside the running ros2 container.
#
# DISABLED BY DEFAULT (2026-09-21). ROS_DISCOVERY_SERVER_ENABLE=0 in .env,
# so this script is a no-op as things stand: ros_discovery_server_restart
# stops anything listening and then returns without starting a replacement,
# and ros_status below prints the server as "disabled". To actually get a
# server, set ROS_DISCOVERY_SERVER_ENABLE=1 in .env and recreate the
# container (`docker compose up -d ros2`) so the new value is in its
# environment -- then this script restarts it in place.
#
# With the flag on, /entrypoint.sh starts the server when the container
# launches and every interactive shell re-checks it, so reach for this only
# after the server has been killed, or to pick up a changed .env.
#
# The server listens on 0.0.0.0:${ROS_DISCOVERY_PORT}, so it serves both
# this container's own nodes (via loopback) and remote peers on the wifi.

cd "$(dirname "$0")"

if [ -z "$(docker compose ps -q --status running ros2 2>/dev/null)" ]; then
    echo "ROS2 container is not running -- start it first (./enter_ros.sh)."
    exit 1
fi

# `exec ... bash -c` is non-interactive, so it does not read ~/.bashrc --
# source the shared environment explicitly to get the helper functions.
docker compose exec ros2 bash -c '
    source /ros_env.sh
    ros_discovery_server_restart
    ros_status
'

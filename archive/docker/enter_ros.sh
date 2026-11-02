#!/bin/bash
set -e

cd ~/ros2_humble_docker || exit 1

# Start Docker daemon if it is not running
if ! docker info >/dev/null 2>&1; then
    echo "Docker is not running. Starting Docker..."
    sudo systemctl start docker
fi

# Resolve DISPLAY if the calling shell has none.
#
# compose.yaml passes DISPLAY straight through, so a container created from
# a shell without one -- `docker compose up -d` over ssh, from a service, or
# from an agent -- bakes DISPLAY= into its config. Container environment is
# fixed at creation, so every later `exec` inherits that empty value, and
# that is exactly what makes rqt_image_view / rviz2 die with
# `could not connect to display` followed by `no Qt platform plugin could be
# initialized`. Nothing is wrong with the X11 mount or the image when that
# happens -- only the variable is missing.
#
# The display is discovered, not hardcoded, because this Jetson's GNOME
# session runs Xwayland and Xwayland numbers its socket per session (:1
# today, but gdm's own servers also sit in that directory as :1024/:1025).
# Pick the X<n> socket in /tmp/.X11-unix that this user owns: that is our
# session's server, and being uid 1000 it is also the one the container user
# `ros` (uid 1000) is allowed to open.
if [ -z "$DISPLAY" ]; then
    for sock in /tmp/.X11-unix/X*; do
        [ -S "$sock" ] && [ -O "$sock" ] || continue
        export DISPLAY=":${sock##*/X}"
        echo "DISPLAY was unset; using $DISPLAY"
        break
    done
fi

# Allow the container to connect to this host's X server, for GUI apps
# (rqt, rviz2, rqt_image_view, ...). Harmless if there's no X server / DISPLAY.
if [ -n "$DISPLAY" ]; then
    xhost +local: >/dev/null 2>&1 || true
fi

# Start ROS2 container if it is not running
if [ -z "$(docker compose ps -q --status running ros2)" ]; then
    echo "ROS2 container is not running. Starting it..."
    docker compose up -d ros2
fi

echo "Entering ROS2 Humble container..."

# -e DISPLAY on the exec, not just in compose.yaml: it overrides whatever the
# container was created with, so a container already running with an empty
# DISPLAY gets working GUI apps without a `docker compose up --force-recreate`
# that would kill the nodes currently running inside it.
docker compose exec -e DISPLAY="$DISPLAY" ros2 bash

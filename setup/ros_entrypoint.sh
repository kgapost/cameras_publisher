#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/camera_capture_ws/install/setup.bash
exec "$@"

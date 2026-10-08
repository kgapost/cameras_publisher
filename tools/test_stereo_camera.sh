#!/usr/bin/env bash
# Runs test_stereo_camera.py inside the cameras-publisher docker image (Jetson,
# CSI/nvarguscamerasrc): a raw check of the two CSI cameras in an OpenCV window,
# no ROS. Mounts this repo at /host so the working copy of test_stereo_camera.py
# is used. See Dockerfile Step 2 for the GStreamer-enabled OpenCV.
#
# Requires, beyond the image itself:
#   --runtime nvidia            bind-mounts the host's nvarguscamerasrc/
#                                nvvidconv GStreamer plugins and libargus into
#                                the container (already this host's Docker
#                                default, passed explicitly for portability).
#   --device .../nvhost-ctrl-vi0, -vi1, -isp, -nvcsi, capture-vi-channel*,
#   capture-isp-channel*        the per-camera VI/ISP/CSI hardware control
#                                nodes. NOT covered by nvidia-container-
#                                runtime's own device auto-mounts (those only
#                                cover GPU compute + display) - without them,
#                                two simultaneous nvarguscamerasrc sessions are
#                                flaky (confirmed by testing: one camera opens
#                                fine, the other intermittently fails with
#                                "Failed to create CaptureSession").
#   -v /tmp/argus_socket:/tmp/argus_socket   nvargus-daemon's socket - the
#                                daemon itself runs on the host, not in the
#                                container.
#   DISPLAY / X11 socket         only needed because the script calls
#                                cv2.imshow(); run `xhost +local:docker` once
#                                per host boot first.

set -e

cd "$(dirname "${BASH_SOURCE[0]}")/.."

IMAGE=cameras-publisher:latest

# Only pass device nodes that actually exist - docker run aborts on a --device
# path that's missing.
shopt -s nullglob
DEVICE_FLAGS=()
for f in /dev/nvhost-ctrl-isp /dev/nvhost-ctrl-isp-thi /dev/nvhost-ctrl-nvcsi \
         /dev/nvhost-ctrl-vi0 /dev/nvhost-ctrl-vi0-thi \
         /dev/nvhost-ctrl-vi1 /dev/nvhost-ctrl-vi1-thi \
         /dev/video0 /dev/video1 \
         /dev/capture-vi-channel* /dev/capture-isp-channel*; do
    [ -e "$f" ] && DEVICE_FLAGS+=(--device "$f")
done
shopt -u nullglob

xhost +local:docker >/dev/null

echo "Starting test_stereo_camera.py in $IMAGE (press 'q' in the window, or Ctrl+C, to stop)..."
docker run --rm -it \
    --runtime nvidia \
    "${DEVICE_FLAGS[@]}" \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)":/host \
    "$IMAGE" \
    python3 /host/test_stereo_camera.py

# --------------------------------------------------------------------------
# cameras_publisher image - Jetson (aarch64); also builds on x86 for testing.
# Base: official ROS 2 Jazzy (Ubuntu 24.04). No CUDA and no PyTorch: the camera
# server needs neither.
#   - OpenCV is Ubuntu's python3-opencv, which is built with GStreamer and V4L2,
#     so the CSI cameras open through nvarguscamerasrc - no OpenCV build.
#   - The NVIDIA GStreamer plugins (nvarguscamerasrc, nvvidconv) and their
#     libraries are NOT in the image: `docker run --runtime nvidia` mounts them
#     from the Jetson (JetPack 6: /etc/nvidia-container-runtime/
#     host-files-for-container.d/drivers.csv). From the image they only need
#     GStreamer, GLib, libEGL and libGLESv2, installed below.
# --------------------------------------------------------------------------
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-pip \
        python3-opencv \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        libegl1 libgles2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Python packages from requirements.txt (all but the [dev-pc] lines).
COPY requirements.txt ./
COPY setup/install_requirements.sh setup/
RUN setup/install_requirements.sh --docker

# Application code. Logs go to /workspace/logs: mount a host folder there.
COPY config.py utils_*.py publish_cameras.py write_calibration_yaml.py test_stereo_camera.py ./
COPY patch/ ./patch/

CMD ["bash"]

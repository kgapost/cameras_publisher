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
        ros-jazzy-std-msgs \
        ros-jazzy-rosidl-default-generators \
        build-essential python3-colcon-common-extensions \
        python3-pip \
        python3-opencv \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        libegl1 libgles2 \
    && rm -rf /var/lib/apt/lists/*

# Keep generated ROS interfaces outside /workspace, which can be bind-mounted.
COPY ros_interfaces/ /opt/camera_capture_ws/src/
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    colcon --log-base /opt/camera_capture_ws/log build \
      --base-paths /opt/camera_capture_ws/src \
      --build-base /opt/camera_capture_ws/build \
      --install-base /opt/camera_capture_ws/install --merge-install \
      --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3'
COPY setup/ros_entrypoint.sh /camera_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/camera_entrypoint.sh"]

WORKDIR /workspace

# Python packages from requirements.txt (all but the [dev-pc] lines).
COPY requirements.txt ./
COPY setup/install_requirements.sh setup/
RUN setup/install_requirements.sh --docker

# Application code. Logs go to /workspace/logs: mount a host folder there.
COPY config.py utils_*.py publish_cameras.py write_calibration_yaml.py test_stereo_camera.py ./
COPY patch/ ./patch/

RUN mkdir -p /workspace/captures
VOLUME ["/workspace/captures"]

CMD ["bash"]

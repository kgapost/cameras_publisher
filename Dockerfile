# --------------------------------------------------------------------------
# cameras_publisher - Jetson (aarch64) image.
# Base: dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 (ROS 2 Jazzy + CUDA).
#   - OpenCV is rebuilt from source with GStreamer (Step 2): needed to open the
#     CSI cameras through nvarguscamerasrc.
#   - Steps 1-3 are kept IDENTICAL to the visual_odometry / obstacle_detection
#     Dockerfiles, so on a Jetson that already built one of those images Docker
#     reuses the cached layers and this build takes a minute instead of ~40.
#     Keep them in sync when changing either file.
#   - MUST be built ON aarch64 (the Jetson itself).
# --------------------------------------------------------------------------

FROM dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================
# Step 1: System dependencies, ROS Jazzy extras, and Cyclone DDS RMW
# ============================================================
# The base image bakes in its own (by-then-stale) ROS apt keyring, so those
# entries must be deleted BEFORE the first apt-get update - a stale/expired
# ROS signing key makes apt-get update itself fail otherwise.
RUN rm -f /etc/apt/sources.list.d/ros2.list \
          /etc/apt/sources.list.d/ros2-latest.list \
          /etc/apt/sources.list.d/ros2.sources

# Remove the base image's own OpenCV so Step 2 below can rebuild it from
# source with GStreamer support (see that step's comment) without conflicts.
RUN apt-get purge -y opencv-dev opencv-main opencv-licenses

RUN apt-get update && apt-get install -y --no-install-recommends \
        wget curl gnupg2 lsb-release \
        build-essential cmake git \
        iputils-ping \
        python3-pip python3-dev \
        libgl1 libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1 \
        liboctomap-dev \
    && mkdir -p /usr/share/keyrings/ \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-cv-bridge \
        ros-jazzy-octomap-msgs \
        ros-jazzy-rmw-cyclonedds-cpp \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Step 2: Build and install OpenCV (with contrib + GStreamer support)
# ============================================================
# The base image leaves its own OpenCV sources in /tmp/opencv, so clear that
# first (git clone refuses a non-empty destination).
# Neither a pip opencv wheel (no GStreamer) nor a jammy-only nvidia-opencv apt
# package (fails dependency resolution on noble/24.04) gives a GStreamer-enabled
# cv2 here, so build against noble's own generic GStreamer instead.
# nvarguscamerasrc/nvvidconv themselves come from the host at runtime via
# nvidia-container-runtime's auto bind-mounts (--runtime nvidia): the host-built
# (jammy, GStreamer 1.20.3) plugins load fine into this image's newer GStreamer
# core because the 1.0 plugin ABI is stable across minor releases.
#
# RUNTIME REQUIREMENTS (not Dockerfile-buildable - pass at `docker run`):
#   --runtime nvidia            (mounts the GStreamer plugins above)
#   --device /dev/nvhost-ctrl-vi0, -vi1, -isp, -nvcsi, and every
#   /dev/capture-vi-channel*, /dev/capture-isp-channel*   the per-camera VI/
#                                 ISP/CSI hardware control nodes. Without them,
#                                 two simultaneous nvarguscamerasrc sessions are
#                                 flaky (one camera intermittently fails with
#                                 "Failed to create CaptureSession").
#   -v /tmp/argus_socket:/tmp/argus_socket   (nvargus-daemon's socket - the
#                                 daemon runs on the host, not the container)
#   -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix   (only for cv2.imshow
#                                 windows; run `xhost +local:docker` on the host)
#
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config unzip \
        libgtk-3-dev \
        libjpeg-dev libpng-dev libtiff-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
        gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/opencv /tmp/opencv_contrib \
    && git clone --branch 4.9.0 --depth 1 https://github.com/opencv/opencv.git /tmp/opencv \
    && git clone --branch 4.9.0 --depth 1 https://github.com/opencv/opencv_contrib.git /tmp/opencv_contrib \
    && mkdir /tmp/opencv/build && cd /tmp/opencv/build \
    && cmake -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
             -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,calib3d,features2d,flann,objdetect,dnn,ml,photo,stitching,video,python3,ximgproc,xfeatures2d,optflow \
             -DWITH_GSTREAMER=ON \
             -DWITH_V4L=ON \
             -DWITH_FFMPEG=OFF \
             -DWITH_GTK=ON \
             -DBUILD_opencv_python3=ON \
             -DPYTHON3_EXECUTABLE=$(which python3) \
             -DINSTALL_PYTHON_EXAMPLES=OFF \
             -DBUILD_EXAMPLES=OFF \
             -DBUILD_TESTS=OFF \
             -DBUILD_PERF_TESTS=OFF \
             -DBUILD_opencv_apps=OFF \
             -DOPENCV_GENERATE_PKGCONFIG=ON \
             .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/opencv /tmp/opencv_contrib

# ============================================================
# Step 3: Python packages
# ============================================================
# The base image points pip at pypi.jetson-ai-lab.dev, which can be down or
# empty ("No matching distribution found for numpy"), so use PyPI explicitly.
# Installed straight into the base image's Python environment (no fresh venv)
# so the base's own prebuilt PyTorch is left untouched. opencv-contrib-python
# and torch/torchvision are deliberately NOT installed here - they'd shadow
# the from-source GStreamer OpenCV (Step 2) and the base image's Jetson-built
# PyTorch, respectively.
RUN python3 -m pip install --break-system-packages --ignore-installed --no-cache-dir \
        --index-url https://pypi.org/simple \
        numpy==1.26.4 \
        scipy==1.11.4 \
    && rm -rf /root/.cache/pip

# ============================================================
# Environment setup
# ============================================================
ENV PYTHONPATH="/opt/venv/lib/python3.12/site-packages:/usr/lib/python3/dist-packages"
ENV LD_LIBRARY_PATH=/usr/local/lib:/usr/local/cuda/lib64:$LD_LIBRARY_PATH
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace

# Application code. Logs go to /workspace/logs: mount a host folder there.
COPY config.py utils_*.py publish_cameras.py write_calibration_yaml.py test_stereo_camera.py ./
COPY patch/ ./patch/

CMD ["bash"]

"""Configuration of the cameras publisher (publish_cameras.py).

Every parameter declared as NAME = _env_<type>('NAME', <default>) can be
overridden without editing this file: export OD_NAME=<value> before starting
(the OD_ prefix is shared with the other SWARMER modules). Topic names and
image sizes must match the consumers (visual_odometry, obstacle_detection).
"""
import os

# === Force RMW Implementation for ROS 2 (must be set before rclpy import) ===
# This ensures all nodes use the same middleware,
# preventing "sequence size exceeds buffer"
# and improving large message handling (images, pointclouds).
# Options:
#   - rmw_fastrtps_cpp   :  Default, uses Fast DDS
#                           can have buffer issues with large msgs.
#   - rmw_cyclonedds_cpp :  Recommended; handles large data well, robust, open-source.
#   - rmw_connextdds     :  Commercial, robust but not free for commercial use.
#   - rmw_zenoh_cpp      :  New lightweight non-DDS, still maturing.

if os.environ.get("RMW_IMPLEMENTATION") is None:
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    print(f"[config] Set RMW_IMPLEMENTATION to {os.environ['RMW_IMPLEMENTATION']}")
else:
    print(f"[config] RMW_IMPLEMENTATION already set to {os.environ['RMW_IMPLEMENTATION']}")

# === Force ROS_DOMAIN_ID (must be set before rclpy import) ===
# A mismatch here means the dev PC (publish_airsim.py) and the Jetson
# (the consumer nodes) are on different DDS domains and never discover each
# other at all - not just slow/lossy data. Driving it from this shared config
# (instead of each machine's own shell/Dockerfile env) keeps both sides in sync.
# Override per-machine without editing this file via the env var:
#   export ROS_DOMAIN_ID=42
# (This one keeps its standard ROS name - no OD_ prefix - because rclpy and every
# other ROS tool read it directly.)
ROS_DOMAIN_ID = 42
if os.environ.get("ROS_DOMAIN_ID") is None:
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    print(f"[config] Set ROS_DOMAIN_ID to {os.environ['ROS_DOMAIN_ID']}")
else:
    print(f"[config] ROS_DOMAIN_ID already set to {os.environ['ROS_DOMAIN_ID']}")


# =========================================================================
# ===== Environment-variable overrides =====
# =========================================================================
# The field-tunable parameters in this file are declared as
#
#   NAME = _env_<type>('NAME', <default>)
#
# which reads the environment variable OD_NAME and falls back to <default> - the
# literal written right here - when it is unset or empty. So config.py stays the
# single source of truth for every default, while a field session can change any
# of them with `export OD_NAME=...` before launching: no edit to a file that is
# mounted read-only into a container on the Jetson, and no rebuild.
#
# Only the ENVIRONMENT VARIABLE carries the OD_ prefix. The config parameter keeps
# its own name, so every `config.NAME` reference across the codebase is unchanged.
#
# Parameters NOT declared through an _env_* helper are deliberately fixed - either
# design choices (or debug
# switches that must stay off in the field (see the Debug section below).

_ENV_PREFIX = 'OD_'

_ENV_LEGACY = {}   # old env var names still honoured (none here)

# (param_name, env_var, default, parsed_value) for every override actually applied.
# Printed by _env_report() at the end of this file.
_ENV_OVERRIDES = []


def _env_lookup(name):
    """(env_var, raw_string) for config parameter `name`, or (None, None) when no
    environment variable is set for it. An empty / whitespace-only value counts as
    unset, so a `export OD_FOO=` left in a sourced script means "use the default"
    rather than "set it to the empty string"."""
    candidates = [_ENV_PREFIX + name]
    if name in _ENV_LEGACY:
        candidates.append(_ENV_LEGACY[name])
    for var in candidates:
        raw = os.environ.get(var)
        if raw is not None and raw.strip() != '':
            return var, raw.strip()
    return None, None


def _env_get(name, default, parse):
    """Core override: return parse(OD_<name>) if set, else `default`.

    A value that fails to parse is a HARD error rather than a silent fallback: on
    a field laptop a typo'd export that is quietly ignored means the whole flight
    runs on a parameter nobody chose, and the log looks completely normal."""
    var, raw = _env_lookup(name)
    if raw is None:
        return default
    try:
        value = parse(raw)
    except Exception as exc:
        raise SystemExit(f"[config] {var}={raw!r} is not a valid value for "
                         f"{name} (default {default!r}): {exc}")
    _ENV_OVERRIDES.append((name, var, default, value))
    return value


def _env_str(name, default, choices=None):
    """String parameter, optionally restricted to `choices` (validated, so a
    misspelled mode fails at import instead of silently taking some other path)."""
    def parse(raw):
        if choices is not None and raw not in choices:
            raise ValueError(f"expected one of {sorted(choices)}")
        return raw
    return _env_get(name, default, parse)


_ENV_TRUE = ('1', 'true', 'yes', 'on', 'y', 't')
_ENV_FALSE = ('0', 'false', 'no', 'off', 'n', 'f')


def _env_bool(name, default):
    """Boolean parameter. Accepts 1/0, true/false, yes/no, on/off (any case)."""
    def parse(raw):
        low = raw.lower()
        if low in _ENV_TRUE:
            return True
        if low in _ENV_FALSE:
            return False
        raise ValueError("expected one of 1/0, true/false, yes/no, on/off")
    return _env_get(name, default, parse)


def _env_int(name, default, choices=None):
    """Integer parameter, optionally restricted to `choices`."""
    def parse(raw):
        value = int(raw)
        if choices is not None and value not in choices:
            raise ValueError(f"expected one of {sorted(choices)}")
        return value
    return _env_get(name, default, parse)


def _env_float(name, default):
    """Float parameter. Accepts anything float() takes, so '5', '5.0' and '5e-1'
    all work - handy when a period is exported as a plain number."""
    return _env_get(name, default, float)


def _env_device(name, default):
    """v4l2 camera device: either an OpenCV device index (0, 1, ...) or a device
    path ('/dev/video0'). A bare integer becomes an int; anything else is kept as
    the string the driver expects."""
    def parse(raw):
        return int(raw) if raw.lstrip('+-').isdigit() else raw
    return _env_get(name, default, parse)


def _env_report():
    """Print one line per applied override. Called once, after the last parameter
    below, so a field-test log records exactly which values this run did NOT take
    from the defaults in this file."""
    if not _ENV_OVERRIDES:
        print(f"[config] no {_ENV_PREFIX}* environment overrides - all defaults")
        return
    print(f"[config] {len(_ENV_OVERRIDES)} environment override(s):")
    for param, var, default, value in _ENV_OVERRIDES:
        print(f"[config]   {param}: {default!r} -> {value!r}  (via {var})")




# =========================================================================
# ===== Stereo pair (front, for obstacle detection) =====
# =========================================================================
# Published size of the stereo images: frames are resized to this before they go
# out, so the consumers always get the same size whatever the capture mode.
IMG_W_PROCESS = _env_int('IMG_W_PROCESS', 640)
IMG_H_PROCESS = _env_int('IMG_H_PROCESS', 400)
# Horizontal field of view (degrees) used for the stereo CameraInfo. 83 is the
# headline FOV of the Waveshare/Seeed IMX219-83 module (its datasheet lists
# 83/73/50 deg as diagonal/horizontal/vertical), worth checking on site.
FOV_D = _env_float('FOV_D', 83)
# Physical distance (cm) between the two cameras' optical centres - goes into the
# camera_left -> camera_right tf. Set it to the REAL MEASURED baseline of the rig.
STEREO_CAMERA_BASELINE_CMS = _env_float('STEREO_CAMERA_BASELINE_CMS', 6.0)
                                  # Waveshare/Seeed IMX219-83 datasheet: Baseline Length 60mm

# Publish the stereo pair (--no-stereo turns it off per run).
STEREO_CAMERA_ENABLED = _env_bool('STEREO_CAMERA_ENABLED', True)

# Capture backend for the stereo pair:
#   'gstreamer' : GStreamer pipelines below - REQUIRED for the Jetson CSI cameras
#                 (nvarguscamerasrc debayers the IMX219's raw 'RG10' in hardware;
#                 plain v4l2 only gets an unusable raw Bayer buffer).
#   'v4l2'      : OpenCV device indices / paths, for a USB / UVC stereo pair.
STEREO_CAMERA_BACKEND = _env_str('STEREO_CAMERA_BACKEND', 'gstreamer',
                                 choices=('v4l2', 'gstreamer'))

# v4l2 device indices (or paths like '/dev/video0') for the two cameras.
STEREO_CAMERA_LEFT_DEVICE = _env_device('STEREO_CAMERA_LEFT_DEVICE', 0)
STEREO_CAMERA_RIGHT_DEVICE = _env_device('STEREO_CAMERA_RIGHT_DEVICE', 1)

# GStreamer pipelines (STEREO_CAMERA_BACKEND='gstreamer'). Output must be BGR.
# Override from the shell with single quotes:
#   export OD_STEREO_CAMERA_LEFT_PIPELINE='nvarguscamerasrc ...'
STEREO_CAMERA_LEFT_PIPELINE = _env_str('STEREO_CAMERA_LEFT_PIPELINE', (
    "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=25/1 "
    "! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"))
STEREO_CAMERA_RIGHT_PIPELINE = _env_str('STEREO_CAMERA_RIGHT_PIPELINE', (
    "nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=640,height=480,framerate=25/1 "
    "! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"))

# Downscale to IMG_W/H_PROCESS inside the GStreamer pipeline (Jetson hardware
# scaler, nvvidconv) instead of with cv2 on the CPU. 'gstreamer' backend only.
STEREO_CAMERA_GSTREAMER_HW_SCALE = _env_bool('STEREO_CAMERA_GSTREAMER_HW_SCALE', False)

# Capture mode requested from a v4l2 stereo pair (the GStreamer pipelines carry
# their own size/rate). Frames are resized to IMG_W/H_PROCESS before publishing.
STEREO_CAMERA_CAPTURE_WIDTH = _env_int('STEREO_CAMERA_CAPTURE_WIDTH', 640)
STEREO_CAMERA_CAPTURE_HEIGHT = _env_int('STEREO_CAMERA_CAPTURE_HEIGHT', 480)
STEREO_CAMERA_FPS = _env_int('STEREO_CAMERA_FPS', 25)

# Idle delay (s) of the capture loop between polls, and the silence (s) after
# which a camera is re-opened (unplugged / hung device; 0 = never).
STEREO_CAMERA_POLL_DELAY = _env_float('STEREO_CAMERA_POLL_DELAY', 0.002)
STEREO_CAMERA_REOPEN_AFTER_S = _env_float('STEREO_CAMERA_REOPEN_AFTER_S', 5.0)


# =========================================================================
# ===== Belly camera (down-looking, for visual odometry) =====
# =========================================================================
# Arducam B0495: 2.3MP AR0234 colour GLOBAL SHUTTER, USB 3.0 (UVC), 1920x1200
# (16:10), YUY2 only, M12 lens 3.6 mm, FOV 95(D) x 82(H) x 66(V) deg, fixed
# focus 2 m - infinity. See the README for calibration.
# Publish the belly camera (--no-belly turns it off per run).
BELLY_CAMERA_ENABLED = _env_bool('BELLY_CAMERA_ENABLED', True)
# Published size. MUST keep the sensor's 16:10 shape (square pixels).
BELLY_CAMERA_WIDTH = _env_int('BELLY_CAMERA_WIDTH', 640)
BELLY_CAMERA_HEIGHT = _env_int('BELLY_CAMERA_HEIGHT', 400)
# Horizontal FOV (degrees): only used for the intrinsics while the lens is not
# calibrated. The datasheet is not self-consistent (82 deg vs 3.6 mm / 3 um ->
# an ~8 % scale error for the VO), so calibrate the real lens.
BELLY_CAMERA_FOV_DEG = _env_float('BELLY_CAMERA_FOV_DEG', 82.0)
# Calibrated intrinsics of the real lens (pixels, at BELLY_CAMERA_WIDTH x
# HEIGHT), e.g. from cv2.calibrateCamera. 0 = centred pinhole from the FOV.
# They go into the belly CameraInfo, which the visual odometry reads.
BELLY_CAMERA_FX = _env_float('BELLY_CAMERA_FX', 0.0)        # TODO: CALIBRATE
BELLY_CAMERA_FY = _env_float('BELLY_CAMERA_FY', 0.0)        # TODO: CALIBRATE
BELLY_CAMERA_CX = _env_float('BELLY_CAMERA_CX', 0.0)        # TODO: CALIBRATE
BELLY_CAMERA_CY = _env_float('BELLY_CAMERA_CY', 0.0)        # TODO: CALIBRATE
# plumb_bob k1, k2, p1, p2, k3 of the real lens (~3 % distortion per datasheet).
BELLY_CAMERA_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)   # TODO: CALIBRATE
# Mount orientation relative to the body (degrees, FRD; pitch -90 = looking
# down with the image top towards the nose) and offset from the IMU (metres,
# body FRD). Published as the base_link -> camera_belly tf.
BELLY_CAMERA_MOUNT_ROLL_DEG = _env_float('BELLY_CAMERA_MOUNT_ROLL_DEG', 0.0)
BELLY_CAMERA_MOUNT_PITCH_DEG = _env_float('BELLY_CAMERA_MOUNT_PITCH_DEG', -90.0)
BELLY_CAMERA_MOUNT_YAW_DEG = _env_float('BELLY_CAMERA_MOUNT_YAW_DEG', 0.0)
BELLY_CAMERA_TX_M = _env_float('BELLY_CAMERA_TX_M', 0.0)   # TODO: MEASURE
BELLY_CAMERA_TY_M = _env_float('BELLY_CAMERA_TY_M', 0.0)   # TODO: MEASURE
BELLY_CAMERA_TZ_M = _env_float('BELLY_CAMERA_TZ_M', 0.1)   # TODO: MEASURE
# Capture. The Arducam is USB (v4l2, its own backend - the CSI ports are taken
# by the stereo pair); use a USB 3 port (USB 2 gives only 10 fps).
BELLY_CAMERA_BACKEND = _env_str('BELLY_CAMERA_BACKEND', 'v4l2',
                                       choices=('v4l2', 'gstreamer'))
BELLY_CAMERA_DEVICE = _env_device('BELLY_CAMERA_DEVICE', 0)
# Capture mode: 960x600 is the AR0234's full-FOV half-resolution mode; YUYV is
# its only pixel format. Capture faster than the publish rate so the newest
# frame is always fresh.
BELLY_CAMERA_CAPTURE_WIDTH = _env_int('BELLY_CAMERA_CAPTURE_WIDTH', 960)
BELLY_CAMERA_CAPTURE_HEIGHT = _env_int('BELLY_CAMERA_CAPTURE_HEIGHT', 600)
BELLY_CAMERA_FPS = _env_int('BELLY_CAMERA_FPS', 60)
BELLY_CAMERA_FOURCC = _env_str('BELLY_CAMERA_FOURCC', 'YUYV')
# Manual exposure in UVC units of 100 us (e.g. 30 = 3 ms); -1 = auto. A short
# fixed exposure limits motion blur and stops auto-exposure flicker.
BELLY_CAMERA_EXPOSURE = _env_int('BELLY_CAMERA_EXPOSURE', -1)
BELLY_CAMERA_GAIN = _env_int('BELLY_CAMERA_GAIN', -1)   # -1 = leave as is
BELLY_CAMERA_POLL_DELAY = _env_float('BELLY_CAMERA_POLL_DELAY', 0.002)
BELLY_CAMERA_REOPEN_AFTER_S = _env_float('BELLY_CAMERA_REOPEN_AFTER_S', 5.0)
# Only used with BELLY_CAMERA_BACKEND='gstreamer' (e.g. a CSI belly camera instead).
BELLY_CAMERA_PIPELINE = _env_str('BELLY_CAMERA_PIPELINE', (
    "v4l2src device=/dev/video2 "
    "! video/x-raw,format=YUY2,width=960,height=600,framerate=30/1 "
    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"))


# =========================================================================
# ===== Stereo calibration (CameraInfo / tf_static, write_calibration_yaml.py) =====
# =========================================================================
# The stereo pair is published as an undistorted pinhole from FOV_D and
# IMG_W/H_PROCESS with zero distortion - what the obstacle detection uses.
CALIBRATION_DISTORTION_MODEL = 'plumb_bob'
CALIBRATION_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)   # k1, k2, p1, p2, k3

# IMU -> LEFT camera transform (translation in metres, body FRD; rotation xyzw).
# Never measured: the values below are a PLACEHOLDER. Set them from calipers/CAD
# and flip CALIBRATION_IMU_TO_CAMERA_MEASURED before sending a calibration out.
CALIBRATION_IMU_TO_CAMERA_TX_M = _env_float('CALIBRATION_IMU_TO_CAMERA_TX_M', 0.0)   # TODO: MEASURE
CALIBRATION_IMU_TO_CAMERA_TY_M = _env_float('CALIBRATION_IMU_TO_CAMERA_TY_M', 0.0)   # TODO: MEASURE
CALIBRATION_IMU_TO_CAMERA_TZ_M = _env_float('CALIBRATION_IMU_TO_CAMERA_TZ_M', 0.0)   # TODO: MEASURE
CALIBRATION_IMU_TO_CAMERA_QX = _env_float('CALIBRATION_IMU_TO_CAMERA_QX', 0.0)       # TODO: MEASURE if camera is not boresight-aligned with the body
CALIBRATION_IMU_TO_CAMERA_QY = _env_float('CALIBRATION_IMU_TO_CAMERA_QY', 0.0)
CALIBRATION_IMU_TO_CAMERA_QZ = _env_float('CALIBRATION_IMU_TO_CAMERA_QZ', 0.0)
CALIBRATION_IMU_TO_CAMERA_QW = _env_float('CALIBRATION_IMU_TO_CAMERA_QW', 1.0)
CALIBRATION_IMU_TO_CAMERA_MEASURED = _env_bool('CALIBRATION_IMU_TO_CAMERA_MEASURED', False)
# IMU-camera time offset (s, camera_stamp - imu_stamp); None = not measured.
CALIBRATION_IMU_CAMERA_TIME_OFFSET_SEC = None


# =========================================================================
# ===== Topics and rates =====
# =========================================================================
# Publish periods (seconds) of the two streams.
TOPIC_PUBLISHER_TIMER_STEREO_CAMERA = _env_float('TOPIC_PUBLISHER_TIMER_STEREO_CAMERA', 0.040)     # ~25 Hz
TOPIC_PUBLISHER_TIMER_BELLY_CAMERA = _env_float('TOPIC_PUBLISHER_TIMER_BELLY_CAMERA', 0.0333)  # ~30 Hz
# A GStreamer camera that has not opened after this many seconds counts as missing
# (OpenCV can otherwise block forever, e.g. nvargus-daemon down or a CSI cable out).
CAMERA_OPEN_TIMEOUT_S = _env_float('CAMERA_OPEN_TIMEOUT_S', 10.0)
# Per-stream fps / stats line in the log every N seconds (minimum 5).
MEASURE_CAMERA_LATENCY_PRINT_SECS = _env_float('MEASURE_CAMERA_LATENCY_PRINT_SECS', 5.0)

# True = JPEG CompressedImage topics, False = raw Image (--compressed / --raw per
# run). Raw and compressed are DIFFERENT topic names: the consumers must agree.
USE_COMPRESSED_IMAGE_TOPICS = _env_bool('USE_COMPRESSED_IMAGE_TOPICS', True)
TOPIC_STEREO_CAMERA_LEFT = _env_str('TOPIC_STEREO_CAMERA_LEFT', '/camera/left/image_raw')
TOPIC_STEREO_CAMERA_RIGHT = _env_str('TOPIC_STEREO_CAMERA_RIGHT', '/camera/right/image_raw')
TOPIC_STEREO_CAMERA_LEFT_COMPRESSED = _env_str('TOPIC_STEREO_CAMERA_LEFT_COMPRESSED', '/camera/left/image_compressed')
TOPIC_STEREO_CAMERA_RIGHT_COMPRESSED = _env_str('TOPIC_STEREO_CAMERA_RIGHT_COMPRESSED', '/camera/right/image_compressed')
TOPIC_STEREO_CAMERA_LEFT_INFO = _env_str('TOPIC_STEREO_CAMERA_LEFT_INFO', '/camera/left/camera_info')
TOPIC_STEREO_CAMERA_RIGHT_INFO = _env_str('TOPIC_STEREO_CAMERA_RIGHT_INFO', '/camera/right/camera_info')
TOPIC_CAMERA_BELLY = _env_str('TOPIC_CAMERA_BELLY', '/camera/belly/image_raw')
TOPIC_CAMERA_BELLY_COMPRESSED = _env_str('TOPIC_CAMERA_BELLY_COMPRESSED', '/camera/belly/image_compressed')
TOPIC_CAMERA_BELLY_INFO = _env_str('TOPIC_CAMERA_BELLY_INFO', '/camera/belly/camera_info')


# Last environment-overridable parameter is above: report what this run changed.
_env_report()

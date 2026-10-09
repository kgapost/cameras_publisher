"""Camera helpers for publish_cameras.py, the only code that opens a physical
camera: capture setup (v4l2 / GStreamer), finding the belly camera, and the belly
intrinsics. Everything else receives the camera feed from ROS topics (or AirSim).
See config.py for the backend / device / pipeline options.
"""

import glob
import math
import os
import re
import threading
import time

import config

import cv2


def _open_gstreamer(pipeline):
    """cv2.VideoCapture of a GStreamer pipeline, or None if opening takes longer than
    config.CAMERA_OPEN_TIMEOUT_S. OpenCV can block forever when a pipeline cannot
    start (missing plugin, nvargus-daemon down, CSI camera unplugged); the open runs
    in a helper thread so that camera is reported as missing instead of hanging the
    whole server. A thread that never returns is left behind (daemon)."""
    result = {}
    t = threading.Thread(target=lambda: result.setdefault('cap', cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)),
                         name='gst-open', daemon=True)
    t.start()
    t.join(config.CAMERA_OPEN_TIMEOUT_S)
    if t.is_alive():
        print(f"[camera] GStreamer pipeline did not open within {config.CAMERA_OPEN_TIMEOUT_S:.0f}s "
              f"- treated as missing: {pipeline}", flush=True)
        return None
    return result.get('cap')


def _build_capture(backend, device, pipeline):
    """Open a single cv2.VideoCapture for the requested backend (None if a GStreamer
    pipeline does not open in time)."""
    if backend == 'gstreamer':
        return _open_gstreamer(pipeline)
    cap = cv2.VideoCapture(device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.STEREO_CAMERA_CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.STEREO_CAMERA_CAPTURE_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.STEREO_CAMERA_FPS)
    return cap


# ----------------------------------------------------------------------
# Belly camera - Arducam B0495 AR0234 USB 3.0 (see README.md)
# ----------------------------------------------------------------------
def belly_camera_intrinsics(width=None, height=None):
    """(K 3x3, distortion (5,)) of the belly camera at width x height (default
    BELLY_CAMERA_WIDTH x HEIGHT): the calibrated BELLY_CAMERA_FX/FY/CX/CY when
    set, else a centred pinhole from BELLY_CAMERA_FOV_DEG; scaled linearly when
    a different image size is asked for."""
    import numpy as np
    w0, h0 = config.BELLY_CAMERA_WIDTH, config.BELLY_CAMERA_HEIGHT
    f_fov = w0 / (2.0 * math.tan(math.radians(config.BELLY_CAMERA_FOV_DEG) / 2.0))
    fx = config.BELLY_CAMERA_FX or f_fov
    fy = config.BELLY_CAMERA_FY or fx
    cx = config.BELLY_CAMERA_CX or w0 / 2.0
    cy = config.BELLY_CAMERA_CY or h0 / 2.0
    sx = (width or w0) / w0
    sy = (height or h0) / h0
    K = np.array([[fx * sx, 0.0, cx * sx], [0.0, fy * sy, cy * sy], [0.0, 0.0, 1.0]])
    return K, np.array(config.BELLY_CAMERA_DISTORTION_COEFFS, dtype=float)


def belly_render_fov_deg():
    """Horizontal FOV AirSim must render the belly camera with so that its ideal
    pinhole has the same focal length as the configured / calibrated one."""
    K, _ = belly_camera_intrinsics()
    return math.degrees(2.0 * math.atan(config.BELLY_CAMERA_WIDTH / (2.0 * K[0, 0])))


def build_belly_capture(device=None):
    """Open the belly camera: v4l2 with its OWN capture mode
    (BELLY_CAMERA_CAPTURE_*, FOURCC - the AR0234 only offers YUYV),
    single-frame buffer and optional manual exposure / gain; or a GStreamer
    pipeline when BELLY_CAMERA_BACKEND='gstreamer'. `device` overrides
    config.BELLY_CAMERA_DEVICE (used by find_belly_capture)."""
    if config.BELLY_CAMERA_BACKEND == 'gstreamer':
        return _open_gstreamer(config.BELLY_CAMERA_PIPELINE)
    cap = cv2.VideoCapture(config.BELLY_CAMERA_DEVICE if device is None else device,
                           cv2.CAP_V4L2)
    if not cap.isOpened():
        return cap
    fourcc = config.BELLY_CAMERA_FOURCC
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4].ljust(4)))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.BELLY_CAMERA_CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.BELLY_CAMERA_CAPTURE_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, config.BELLY_CAMERA_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if config.BELLY_CAMERA_EXPOSURE >= 0:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)          # V4L2_EXPOSURE_MANUAL
        cap.set(cv2.CAP_PROP_EXPOSURE, config.BELLY_CAMERA_EXPOSURE)
    if config.BELLY_CAMERA_GAIN >= 0:
        cap.set(cv2.CAP_PROP_GAIN, config.BELLY_CAMERA_GAIN)
    return cap


# ----------------------------------------------------------------------
# Finding the cameras that are actually plugged in
# ----------------------------------------------------------------------
# V4L2 names of the Jetson's CSI sensor nodes (the stereo pair): never the belly camera.
_CSI_V4L2_NAMES = ('vi-output', 'imx219', 'nvcsi', 'tegra')


def device_path(device):
    """'/dev/videoN' for an index N, or the path unchanged."""
    return f'/dev/video{device}' if isinstance(device, int) else str(device)


def v4l2_name(device):
    """Kernel name of a v4l2 device ('' if unknown), from sysfs."""
    node = os.path.basename(os.path.realpath(device_path(device)))
    try:
        with open(f'/sys/class/video4linux/{node}/name') as f:
            return f.read().strip()
    except OSError:
        return ''


def grab_ok(cap, tries=10, delay=0.1):
    """True if `cap` delivers a frame within `tries` reads: tells a real capture
    node from one that merely opens (a UVC metadata node, a dead device)."""
    for _ in range(tries):
        ok, frame = cap.read()
        if ok and frame is not None:
            return True
        time.sleep(delay)
    return False


def find_belly_capture(exclude=()):
    """Find the belly camera. Returns (cap, device) with the capture open and
    delivering frames, or (None, None) when no belly camera is plugged in.

    The configured config.BELLY_CAMERA_DEVICE (default /dev/video0) is
    tried first, then every other /dev/video* node. A node is skipped when it is
    a CSI sensor (the stereo pair), is in `exclude`, or does not deliver a frame
    (e.g. the UVC metadata node next to the real capture node)."""
    if config.BELLY_CAMERA_BACKEND == 'gstreamer':
        cap = build_belly_capture()
        if cap is not None and cap.isOpened() and grab_ok(cap):
            return cap, 'gstreamer'
        if cap is not None:
            cap.release()
        return None, None
    skip = {os.path.realpath(device_path(d)) for d in exclude}
    configured = device_path(config.BELLY_CAMERA_DEVICE)
    others = sorted(glob.glob('/dev/video*'),
                    key=lambda p: int(re.sub(r'\D', '', p) or 0))
    for dev in [configured] + [p for p in others if p != configured]:
        if os.path.realpath(dev) in skip or not os.path.exists(dev):
            continue
        name = v4l2_name(dev)
        if any(k in name.lower() for k in _CSI_V4L2_NAMES):
            continue
        cap = build_belly_capture(dev)
        if cap.isOpened() and grab_ok(cap):
            print(f"[BellyCamera] Found at {dev}" + (f" ('{name}')." if name else "."))
            return cap, dev
        cap.release()
    return None, None


def fill_camera_info(msg, frame_id, width, height, K, D=(0.0, 0.0, 0.0, 0.0, 0.0)):
    """Populate a sensor_msgs/CameraInfo (duck-typed - this module imports no
    ROS) from a 3x3 K and plumb_bob distortion D."""
    fx, fy, cx, cy = float(K[0][0]), float(K[1][1]), float(K[0][2]), float(K[1][2])
    msg.header.frame_id = frame_id
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = 'plumb_bob'
    msg.d = [float(v) for v in D]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg

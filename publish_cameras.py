#!/usr/bin/env python3
"""Cameras publisher (Jetson): stereo pair + belly camera as ROS 2 topics.

Two independent streams, each with its own capture thread, rate, CameraInfo and
tf_static frame:
  * stereo : config.TOPIC_STEREO_CAMERA_LEFT/RIGHT(_COMPRESSED) at IMG_W/H_PROCESS,
             every config.TOPIC_PUBLISHER_TIMER_STEREO_CAMERA s
  * belly  : config.TOPIC_CAMERA_BELLY(_COMPRESSED) at BELLY_CAMERA_WIDTH/HEIGHT,
             every config.TOPIC_PUBLISHER_TIMER_BELLY_CAMERA s

Each stream is switched by config.STEREO_CAMERA_ENABLED / BELLY_CAMERA_ENABLED
(both on by default; --no-stereo / --no-belly per run). Every enabled camera is
found at start-up and only what is found is published (belly only, stereo only,
left only, right only, or everything). If nothing is found the node exits with an
error, so a docker restart policy retries it.

Robustness: a camera that stops delivering frames is re-opened automatically
(config.STEREO/BELLY_CAMERA_REOPEN_AFTER_S), a stalled stream is reported by a
watchdog, exceptions in a capture thread or a publish callback are logged and
survived, and SIGTERM (docker stop) shuts down cleanly.

Logs: terminal (INFO) and logs/publish_cameras.log next to this file (DEBUG,
rotating) - start-up config, devices, every open/re-open, stalls, errors and
per-stream stats.

Stereo configurations (config.py, overridable per run):
  * Two USB / UVC cameras (v4l2)     : --backend v4l2 --left /dev/video0 --right /dev/video2
  * CSI stereo camera(s) (GStreamer) : --backend gstreamer (STEREO_CAMERA_*_PIPELINE)
  * Single camera only               : right absent -> left-only
Belly camera: own backend (config.BELLY_CAMERA_BACKEND, default v4l2; the
Orin Nano's two CSI ports are taken by the stereo pair), --belly-backend / --belly.

Run inside the Jetson docker, CSI stereo + USB belly camera (see README.md):

    docker run --rm --network host --runtime nvidia \
        -v /tmp/argus_socket:/tmp/argus_socket --device /dev/video2 \
        cameras-publisher:latest python3 publish_cameras.py --backend gstreamer --belly /dev/video2

/dev/videoN numbering can change across reboots; use /dev/v4l/by-path/... for a fixed rig.
"""

import argparse
import glob
import math
import os
import platform
import signal
import sys
import threading
import time

import config

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from patch.cv_bridge import CvBridge

from builtin_interfaces.msg import Time as TimeMsg
from utils_camera_source import (_build_capture, belly_camera_intrinsics, device_path,
                                 find_belly_capture, fill_camera_info, grab_ok, v4l2_name)
from utils_log import setup_logger, LOG_DIR

log = setup_logger('publish_cameras')

STALL_AFTER_S = 3.0        # watchdog: no new frame for this long = stream stalled
WATCHDOG_PERIOD_S = 1.0
MAX_ERROR_LOGS_PER_MIN = 6  # repeated identical errors are rate-limited in the log


def _resize_to(frame, width, height):
    """Resize to width x height; skipped when the frame already has that size
    (matching capture size, GStreamer hardware-scaled frame)."""
    if frame is None:
        return None
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_NEAREST)


def _configure_gstreamer_scaling():
    """If config.STEREO_CAMERA_GSTREAMER_HW_SCALE is on, inject each stream's output
    size into the nvvidconv caps of its GStreamer pipeline so the Jetson hardware
    scaler does the downscale. Must run after the CLI overrides, before opening."""
    if not config.STEREO_CAMERA_GSTREAMER_HW_SCALE:
        return
    targets = []
    if config.STEREO_CAMERA_ENABLED and config.STEREO_CAMERA_BACKEND == 'gstreamer':
        targets += [('STEREO_CAMERA_LEFT_PIPELINE', config.IMG_W_PROCESS, config.IMG_H_PROCESS),
                    ('STEREO_CAMERA_RIGHT_PIPELINE', config.IMG_W_PROCESS, config.IMG_H_PROCESS)]
    if config.BELLY_CAMERA_ENABLED and config.BELLY_CAMERA_BACKEND == 'gstreamer':
        targets.append(('BELLY_CAMERA_PIPELINE',
                        config.BELLY_CAMERA_WIDTH, config.BELLY_CAMERA_HEIGHT))
    marker = "nvvidconv ! video/x-raw,format="
    for attr, w, h in targets:
        pipeline = getattr(config, attr)
        if marker in pipeline:
            setattr(config, attr, pipeline.replace(
                marker, f"nvvidconv ! video/x-raw,width={w},height={h},format=", 1))
            log.info(f"GStreamer hardware scaling of {attr} to {w}x{h} (nvvidconv) enabled.")
        else:
            log.warning(f"Could not enable hardware scaling for {attr} "
                        f"(no '{marker}' stage found); the CPU resize will handle it.")


def _build_camera_info(frame_id, width, height, fov_deg):
    """sensor_msgs/CameraInfo from the pinhole model the consumers use (focal from
    the horizontal FOV, centred principal point, config.py's distortion)."""
    focal = width / (2.0 * math.tan(math.radians(fov_deg / 2.0)))
    cx, cy = width / 2.0, height / 2.0
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = config.CALIBRATION_DISTORTION_MODEL
    msg.d = list(config.CALIBRATION_DISTORTION_COEFFS)
    msg.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def _static_transform(parent, child, xyz, quat_xyzw):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = quat_xyzw
    return t


def _broadcast_calibration_tf(node, with_left, with_right, with_belly):
    """Static, latched transforms: base_link->camera_left (config.CALIBRATION_IMU_TO_CAMERA_*),
    camera_left->camera_right (stereo baseline) and base_link->camera_belly (config.BELLY_CAMERA_*
    mount). Cameras use the body (FRD) axis convention. Returns the broadcaster (the
    caller must keep a reference), or None if it could not be created."""
    baseline_m = config.STEREO_CAMERA_BASELINE_CMS / 100.0
    stamp = node.get_clock().now().to_msg()
    transforms = []
    if with_left:
        transforms.append(_static_transform(
            'base_link', 'camera_left',
            (config.CALIBRATION_IMU_TO_CAMERA_TX_M,
             config.CALIBRATION_IMU_TO_CAMERA_TY_M,
             config.CALIBRATION_IMU_TO_CAMERA_TZ_M),
            (config.CALIBRATION_IMU_TO_CAMERA_QX,
             config.CALIBRATION_IMU_TO_CAMERA_QY,
             config.CALIBRATION_IMU_TO_CAMERA_QZ,
             config.CALIBRATION_IMU_TO_CAMERA_QW)))
        if with_right:
            transforms.append(_static_transform(
                'camera_left', 'camera_right', (baseline_m, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    elif with_right:
        # Left camera missing: the right one hangs off base_link directly, one
        # baseline along the (rotated) camera x axis from where the left one would be.
        q = (config.CALIBRATION_IMU_TO_CAMERA_QX, config.CALIBRATION_IMU_TO_CAMERA_QY,
             config.CALIBRATION_IMU_TO_CAMERA_QZ, config.CALIBRATION_IMU_TO_CAMERA_QW)
        offset = Rotation.from_quat(q).apply([baseline_m, 0.0, 0.0])
        transforms.append(_static_transform(
            'base_link', 'camera_right',
            (config.CALIBRATION_IMU_TO_CAMERA_TX_M + offset[0],
             config.CALIBRATION_IMU_TO_CAMERA_TY_M + offset[1],
             config.CALIBRATION_IMU_TO_CAMERA_TZ_M + offset[2]), q))
    if with_belly:
        mount = Rotation.from_euler('ZYX', [config.BELLY_CAMERA_MOUNT_YAW_DEG,
                                            config.BELLY_CAMERA_MOUNT_PITCH_DEG,
                                            config.BELLY_CAMERA_MOUNT_ROLL_DEG], degrees=True)
        transforms.append(_static_transform(
            'base_link', 'camera_belly',
            (config.BELLY_CAMERA_TX_M, config.BELLY_CAMERA_TY_M, config.BELLY_CAMERA_TZ_M),
            tuple(mount.as_quat())))
    for t in transforms:
        t.header.stamp = stamp
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transforms)
    log.info("tf_static broadcast: " + ", ".join(
        f"{t.header.frame_id}->{t.child_frame_id}" for t in transforms))
    if (with_left or with_right) and not config.CALIBRATION_IMU_TO_CAMERA_MEASURED:
        log.warning("base_link->camera_left is still the identity/zero PLACEHOLDER "
                    "(config.CALIBRATION_IMU_TO_CAMERA_MEASURED is False), not a real "
                    "measurement - see config.py's Calibration section.")
    return broadcaster


class _ErrorLimiter:
    """Logs an exception with traceback at most MAX_ERROR_LOGS_PER_MIN times per
    minute per key; the rest are counted and summarised at the next logged one."""

    def __init__(self):
        self.window_start = {}
        self.count = {}
        self.suppressed = {}

    def error(self, key, msg):
        now = time.monotonic()
        if now - self.window_start.get(key, 0.0) > 60.0:
            self.window_start[key] = now
            self.count[key] = 0
        self.count[key] += 1
        if self.count[key] <= MAX_ERROR_LOGS_PER_MIN:
            skipped = self.suppressed.pop(key, 0)
            extra = f" ({skipped} similar errors suppressed)" if skipped else ""
            log.exception(f"{msg}{extra}")
        else:
            self.suppressed[key] = self.suppressed.get(key, 0) + 1


_errors = _ErrorLimiter()


def _release(cap):
    if cap is not None:
        try:
            cap.release()
        except Exception:
            log.debug("cap.release() failed", exc_info=True)


class _CaptureSource:
    """Base capture thread. Subclasses implement _grab() (read one set of frames
    and store it, return True on success) and _reopen() (re-open the camera(s)).
    The loop survives exceptions, and when no frame has arrived for
    its REOPEN_AFTER_S it calls _reopen() (with a back-off)."""

    name = 'Camera'
    cfg = None                    # config prefix: STEREO_CAMERA / BELLY_CAMERA

    def __init__(self):
        self.lock = threading.Lock()
        self.frame_id = 0
        self.running = False
        self.thread = None
        self.last_frame_t = None      # time.monotonic() of the last good frame
        self.read_failures = 0
        self.reopens = 0
        self.captured = 0

    def _grab(self):
        raise NotImplementedError

    def _reopen(self):
        raise NotImplementedError

    def _release_all(self):
        raise NotImplementedError

    def _loop(self):
        poll = getattr(config, f'{self.cfg}_POLL_DELAY')
        reopen_after = getattr(config, f'{self.cfg}_REOPEN_AFTER_S')
        last_ok = time.monotonic()
        next_reopen = 0.0
        backoff = 1.0
        while self.running:
            try:
                if self._grab():
                    now = time.monotonic()
                    if now - last_ok > reopen_after:
                        log.info(f"[{self.name}] Frames are flowing again.")
                    last_ok = self.last_frame_t = now
                    self.captured += 1
                    backoff = 1.0
                    time.sleep(poll)
                    continue
                self.read_failures += 1
            except Exception:
                self.read_failures += 1
                _errors.error(f"{self.name}-grab", f"[{self.name}] Capture error")
            now = time.monotonic()
            if reopen_after > 0 and now - last_ok > reopen_after and now >= next_reopen:
                self.reopens += 1
                log.warning(f"[{self.name}] No frame for {now - last_ok:.1f}s "
                            f"({self.read_failures} failed reads so far) - re-opening "
                            f"(attempt {self.reopens}).")
                try:
                    self._reopen()
                except Exception:
                    _errors.error(f"{self.name}-reopen", f"[{self.name}] Re-open error")
                backoff = min(backoff * 2.0, 30.0)
                next_reopen = time.monotonic() + backoff
            time.sleep(max(poll, 0.01))

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, name=f"{self.name}-capture", daemon=True)
        self.thread.start()

    def ensure_running(self):
        """Watchdog hook: restart the capture thread if it has died."""
        if self.running and (self.thread is None or not self.thread.is_alive()):
            log.error(f"[{self.name}] Capture thread died - restarting it.")
            self.start()

    def stop(self):
        self.running = False
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self._release_all()


class StereoCameraSource(_CaptureSource):
    """Stereo pair. Left and right are opened on their own and whichever is missing
    is left out. Newest frames (resized to IMG_W/H_PROCESS) live in self.left /
    self.right (None for an absent camera)."""

    name = 'StereoCamera'
    cfg = 'STEREO_CAMERA'

    def __init__(self, exclude=()):
        super().__init__()
        self.exclude = {os.path.realpath(device_path(d)) for d in exclude}
        self.left = None
        self.right = None
        self.cap_left = None
        self.cap_right = None
        self.has_left = False       # found at start-up; fixed for the run
        self.has_right = False
        self._logged_first = False

    def _try_open(self, label, device, pipeline):
        """One camera, or None (with a logged reason) when it is not there."""
        backend = config.STEREO_CAMERA_BACKEND
        try:
            if backend == 'v4l2' and os.path.realpath(device_path(device)) in self.exclude:
                log.info(f"[StereoCamera] {label} device {device} is the belly camera - skipping it.")
                return None
            cap = _build_capture(backend, device, pipeline)
            if cap is None or not cap.isOpened():
                log.warning(f"[StereoCamera] {label} camera not found (backend={backend}, device={device}).")
                _release(cap)
                return None
            if backend == 'v4l2':
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # newest frame only
                if not grab_ok(cap):
                    log.warning(f"[StereoCamera] {label} camera ({device}) opens but delivers no frames.")
                    _release(cap)
                    return None
            log.info(f"[StereoCamera] {label} camera opened (backend={backend}, device={device}).")
            return cap
        except Exception:
            log.exception(f"[StereoCamera] Opening {label} camera failed")
            return None

    def _open_left(self):
        return self._try_open('LEFT', config.STEREO_CAMERA_LEFT_DEVICE, config.STEREO_CAMERA_LEFT_PIPELINE)

    def _open_right(self):
        return self._try_open('RIGHT', config.STEREO_CAMERA_RIGHT_DEVICE, config.STEREO_CAMERA_RIGHT_PIPELINE)

    def open(self):
        """Probe the cameras (blocking). True if at least one was found."""
        self.cap_left = self._open_left()
        self.cap_right = self._open_right()
        self.has_left = self.cap_left is not None
        self.has_right = self.cap_right is not None
        return self.has_left or self.has_right

    def _reopen(self):
        _release(self.cap_left)
        _release(self.cap_right)
        self.cap_left = self.cap_right = None
        if self.has_left:
            self.cap_left = self._open_left()
        if self.has_right:
            self.cap_right = self._open_right()

    def _release_all(self):
        _release(self.cap_left)
        _release(self.cap_right)

    def _grab(self):
        left = right = None
        if self.has_left:
            if self.cap_left is None:
                return False
            ok, left = self.cap_left.read()
            if not ok or left is None:
                return False
        if self.has_right:
            if self.cap_right is None:
                return False
            ok, right = self.cap_right.read()
            if not ok or right is None:
                return False
        left = _resize_to(left, config.IMG_W_PROCESS, config.IMG_H_PROCESS)
        right = _resize_to(right, config.IMG_W_PROCESS, config.IMG_H_PROCESS)
        with self.lock:
            self.left, self.right = left, right
            self.frame_id += 1
        if not self._logged_first:
            self._logged_first = True
            log.info(f"[StereoCamera] First frame captured "
                     f"(left={'ok' if left is not None else 'missing'}, "
                     f"right={'ok' if right is not None else 'missing'}).")
        return True


class BellyCameraSource(_CaptureSource):
    """Single belly camera. Newest frame (resized to BELLY_CAMERA_WIDTH x
    HEIGHT) and its capture time live in self.frame / self.stamp."""

    name = 'BellyCamera'
    cfg = 'BELLY_CAMERA'

    def __init__(self):
        super().__init__()
        self.frame = None
        self.stamp = None       # capture time (wall clock, s) of self.frame
        self.cap = None
        self.device = None
        self._logged_first = False

    def open(self, exclude=()):
        """Look for the belly camera (see find_belly_capture). True if found."""
        try:
            self.cap, self.device = find_belly_capture(exclude)
        except Exception:
            log.exception("[BellyCamera] Search for the belly camera failed")
            self.cap = None
        if self.cap is None:
            log.warning(f"[BellyCamera] No belly camera found (tried "
                        f"{device_path(config.BELLY_CAMERA_DEVICE)} first, then the other "
                        f"/dev/video* nodes).")
            return False
        log.info(f"[BellyCamera] Opened {self.device} (backend={config.BELLY_CAMERA_BACKEND}, "
                 f"requested {config.BELLY_CAMERA_CAPTURE_WIDTH}x{config.BELLY_CAMERA_CAPTURE_HEIGHT}"
                 f"@{config.BELLY_CAMERA_FPS} {config.BELLY_CAMERA_FOURCC}, exposure="
                 f"{'auto' if config.BELLY_CAMERA_EXPOSURE < 0 else config.BELLY_CAMERA_EXPOSURE}).")
        return True

    def _reopen(self):
        _release(self.cap)
        self.cap = None
        self.cap, dev = find_belly_capture()   # the device node may have changed
        if self.cap is not None:
            self.device = dev
            log.info(f"[BellyCamera] Re-opened on {dev}.")
        else:
            log.warning("[BellyCamera] Re-open failed - belly camera not found.")

    def _release_all(self):
        _release(self.cap)

    def _capture_time(self, t_read):
        """Wall-clock capture time of the frame just read. v4l2 reports the driver's
        buffer timestamp (CLOCK_MONOTONIC) via CAP_PROP_POS_MSEC - closer to the
        exposure than read()'s return; used when plausible, else the read time. The
        visual odometry interpolates the IMU attitude to this stamp."""
        try:
            age = time.monotonic() - self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if 0.0 <= age < 0.5:
                return time.time() - age
        except Exception:
            pass
        return t_read

    def _grab(self):
        if self.cap is None:
            return False
        ok, frame = self.cap.read()
        t_read = time.time()
        if not ok or frame is None:
            return False
        stamp = self._capture_time(t_read)
        if not self._logged_first:
            self._logged_first = True
            h, w = frame.shape[:2]
            log.info(f"[BellyCamera] First frame captured: {w}x{h}, published at "
                     f"{config.BELLY_CAMERA_WIDTH}x{config.BELLY_CAMERA_HEIGHT}.")
            if abs(w / h - config.BELLY_CAMERA_WIDTH / config.BELLY_CAMERA_HEIGHT) > 0.01:
                log.warning(f"[BellyCamera] capture aspect {w}x{h} differs from the published "
                            f"{config.BELLY_CAMERA_WIDTH}x{config.BELLY_CAMERA_HEIGHT} - the resize "
                            "will squash the pixels and bias the VO scale. Match the 16:10 sensor.")
        frame = _resize_to(frame, config.BELLY_CAMERA_WIDTH, config.BELLY_CAMERA_HEIGHT)
        with self.lock:
            self.frame = frame
            self.stamp = stamp
            self.frame_id += 1
        return True


class _StreamStats:
    """Per-stream published-FPS line (INFO, the headless server's sign of life) and
    a DEBUG stats line with capture/failure/re-open counters and encode time."""

    def __init__(self, label, source):
        self.label = label
        self.source = source
        self.window = max(float(config.MEASURE_CAMERA_LATENCY_PRINT_SECS), 5.0)
        self.count = 0
        self.encode_ms = 0.0
        self.last = time.monotonic()
        self.last_captured = 0

    def tick(self, encode_ms, detail=''):
        self.count += 1
        self.encode_ms += encode_ms
        now = time.monotonic()
        elapsed = now - self.last
        if elapsed >= self.window:
            captured = self.source.captured - self.last_captured
            log.info(f"Publishing {self.label} at {self.count / elapsed:.1f} fps{detail}.")
            log.debug(f"[{self.label}] stats: published={self.count} captured={captured} "
                      f"({captured / elapsed:.1f} fps) read_failures={self.source.read_failures} "
                      f"reopens={self.source.reopens} avg_msg_build={self.encode_ms / self.count:.2f}ms")
            self.count = 0
            self.encode_ms = 0.0
            self.last = now
            self.last_captured = self.source.captured


class CameraPublisher(Node):
    def __init__(self, stereo_source=None, belly_source=None):
        super().__init__('cameras_publisher')
        self.bridge = CvBridge()
        self.stereo_source = stereo_source
        self.belly_source = belly_source
        # Same topic/message-type selection as the consumers.
        if config.USE_COMPRESSED_IMAGE_TOPICS:
            self.left_topic = config.TOPIC_STEREO_CAMERA_LEFT_COMPRESSED
            self.right_topic = config.TOPIC_STEREO_CAMERA_RIGHT_COMPRESSED
            self.belly_topic = config.TOPIC_CAMERA_BELLY_COMPRESSED
            self.msg_type = CompressedImage
        else:
            self.left_topic = config.TOPIC_STEREO_CAMERA_LEFT
            self.right_topic = config.TOPIC_STEREO_CAMERA_RIGHT
            self.belly_topic = config.TOPIC_CAMERA_BELLY
            self.msg_type = Image
        fmt = 'compressed' if config.USE_COMPRESSED_IMAGE_TOPICS else 'raw'
        self._stalled = {}
        self._sources = {}

        # --- stereo stream (only the cameras that were found) ---
        has_left = stereo_source is not None and stereo_source.has_left
        has_right = stereo_source is not None and stereo_source.has_right
        if has_left or has_right:
            self.pub_left = self.create_publisher(self.msg_type, self.left_topic, 10) if has_left else None
            self.pub_right = self.create_publisher(self.msg_type, self.right_topic, 10) if has_right else None
            self.pub_left_info = (self.create_publisher(CameraInfo, config.TOPIC_STEREO_CAMERA_LEFT_INFO, 10)
                                  if has_left else None)
            self.pub_right_info = (self.create_publisher(CameraInfo, config.TOPIC_STEREO_CAMERA_RIGHT_INFO, 10)
                                   if has_right else None)
            self.cam_info_left = _build_camera_info(
                'camera_left', config.IMG_W_PROCESS, config.IMG_H_PROCESS, config.FOV_D)
            self.cam_info_right = _build_camera_info(
                'camera_right', config.IMG_W_PROCESS, config.IMG_H_PROCESS, config.FOV_D)
            self.create_timer(config.TOPIC_PUBLISHER_TIMER_STEREO_CAMERA, self.publish_frames)
            self._last_frame_id = -1
            self._first_published = False
            self._stereo_stats = _StreamStats('stereo', stereo_source)
            self._sources['stereo'] = stereo_source
            topics = " / ".join(t for t, on in ((self.left_topic, has_left), (self.right_topic, has_right)) if on)
            log.info(f"Publishing {fmt} stereo images on {topics} every {config.TOPIC_PUBLISHER_TIMER_STEREO_CAMERA}s.")

        # --- belly stream (own timer / rate) ---
        if belly_source is not None:
            self.pub_belly = self.create_publisher(self.msg_type, self.belly_topic, 10)
            self.pub_belly_info = self.create_publisher(CameraInfo, config.TOPIC_CAMERA_BELLY_INFO, 10)
            # Calibrated lens (BELLY_CAMERA_FX.. + distortion) when set - the VO takes
            # its intrinsics from this topic (VO_USE_CAMERA_INFO).
            K, D = belly_camera_intrinsics()
            self.cam_info_belly = fill_camera_info(
                CameraInfo(), 'camera_belly', config.BELLY_CAMERA_WIDTH, config.BELLY_CAMERA_HEIGHT, K, D)
            log.info(f"Belly intrinsics: fx={K[0][0]:.1f} fy={K[1][1]:.1f} cx={K[0][2]:.1f} cy={K[1][2]:.1f} "
                     f"({'calibrated' if config.BELLY_CAMERA_FX else 'from FOV - NOT calibrated'}).")
            self.create_timer(config.TOPIC_PUBLISHER_TIMER_BELLY_CAMERA, self.publish_belly)
            self._last_belly_id = -1
            self._belly_stats = _StreamStats('belly', belly_source)
            self._sources['belly'] = belly_source
            log.info(f"Publishing {fmt} belly images on {self.belly_topic} "
                     f"every {config.TOPIC_PUBLISHER_TIMER_BELLY_CAMERA}s.")

        self.create_timer(WATCHDOG_PERIOD_S, self._watchdog)

    def _to_msg(self, frame, stamp, frame_id):
        if self.msg_type is CompressedImage:
            msg = CompressedImage()
            msg.format = 'jpeg'
            ok, buf = cv2.imencode('.jpg', frame)
            if not ok:
                raise RuntimeError("cv2.imencode failed")
            msg.data = buf.tobytes()
        else:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        return msg

    def _watchdog(self):
        """Reports a stalled/recovered stream and restarts a dead capture thread."""
        now = time.monotonic()
        for name, src in self._sources.items():
            try:
                src.ensure_running()
                age = None if src.last_frame_t is None else now - src.last_frame_t
                stalled = age is None or age > STALL_AFTER_S
                if stalled and not self._stalled.get(name, False):
                    log.warning(f"[{name}] STALLED: "
                                f"{'no frame yet' if age is None else f'no new frame for {age:.1f}s'}.")
                elif not stalled and self._stalled.get(name, False):
                    log.info(f"[{name}] Recovered.")
                self._stalled[name] = stalled
            except Exception:
                _errors.error(f"watchdog-{name}", f"Watchdog error ({name})")

    def publish_frames(self):
        try:
            self._publish_frames()
        except Exception:
            _errors.error('publish-stereo', "Stereo publish failed")

    def _publish_frames(self):
        src = self.stereo_source
        with src.lock:
            if src.frame_id == self._last_frame_id or (src.left is None and src.right is None):
                return  # no new frame captured since the last publish
            self._last_frame_id = src.frame_id
            left = src.left.copy() if src.left is not None else None
            right = src.right.copy() if src.right is not None else None

        t0 = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        for frame, pub, info_pub, cam_info, frame_id in (
                (left, self.pub_left, self.pub_left_info, self.cam_info_left, 'camera_left'),
                (right, self.pub_right, self.pub_right_info, self.cam_info_right, 'camera_right')):
            if frame is None or pub is None:
                continue
            pub.publish(self._to_msg(frame, stamp, frame_id))
            cam_info.header.stamp = stamp
            info_pub.publish(cam_info)

        kind = 'stereo' if left is not None and right is not None else ('left only' if left is not None else 'right only')
        if not self._first_published:
            self._first_published = True
            log.info(f"First {kind} frame published: shape={(left if left is not None else right).shape}")
        self._stereo_stats.tick((time.perf_counter() - t0) * 1000.0, f" ({kind})")

    def publish_belly(self):
        try:
            self._publish_belly()
        except Exception:
            _errors.error('publish-belly', "Belly publish failed")

    def _publish_belly(self):
        src = self.belly_source
        with src.lock:
            if src.frame is None or src.frame_id == self._last_belly_id:
                return
            self._last_belly_id = src.frame_id
            frame = src.frame.copy()
            t_cap = src.stamp
        t0 = time.perf_counter()
        # Stamped with the CAPTURE time, not now: the VO aligns the IMU attitude to
        # this instant (node_visual_odometry.py, VO_TIME_SOURCE='auto').
        sec = int(t_cap)
        stamp = TimeMsg(sec=sec, nanosec=int((t_cap - sec) * 1e9))
        self.pub_belly.publish(self._to_msg(frame, stamp, 'camera_belly'))
        self.cam_info_belly.header.stamp = stamp
        self.pub_belly_info.publish(self.cam_info_belly)
        self._belly_stats.tick((time.perf_counter() - t0) * 1000.0)


def _parse_args():
    """CLI overrides for the config.STEREO_CAMERA_* / topic-format options, so the
    same server covers every rig without editing config.py."""
    p = argparse.ArgumentParser(description="Cameras publisher: stereo pair + belly camera as ROS 2 topics (Jetson).")
    p.add_argument('--backend', choices=('v4l2', 'gstreamer'),
                   help="Stereo capture backend (default: config.STEREO_CAMERA_BACKEND).")
    p.add_argument('--left', help="Left camera v4l2 device index/path (v4l2 backend).")
    p.add_argument('--right', help="Right camera v4l2 device index/path (v4l2 backend).")
    belly = p.add_mutually_exclusive_group()
    belly.add_argument('--belly', help="Belly camera v4l2 device index/path "
                                       "(default: config.BELLY_CAMERA_DEVICE).")
    belly.add_argument('--no-belly', dest='no_belly', action='store_true',
                       help="Do not publish the belly camera (config.BELLY_CAMERA_ENABLED=False).")
    p.add_argument('--no-stereo', dest='no_stereo', action='store_true',
                   help="Do not publish the stereo pair (config.STEREO_CAMERA_ENABLED=False).")
    p.add_argument('--belly-backend', choices=('v4l2', 'gstreamer'),
                   help="Belly capture backend (default: config.BELLY_CAMERA_BACKEND).")
    fmt = p.add_mutually_exclusive_group()
    fmt.add_argument('--compressed', dest='compressed', action='store_true', default=None,
                     help="Publish JPEG CompressedImage topics.")
    fmt.add_argument('--raw', dest='compressed', action='store_false',
                     help="Publish raw Image topics.")
    p.add_argument('--width', type=int, help="Requested v4l2 capture width.")
    p.add_argument('--height', type=int, help="Requested v4l2 capture height.")
    p.add_argument('--fps', type=int, help="Requested v4l2 capture FPS.")
    hw = p.add_mutually_exclusive_group()
    hw.add_argument('--hw-scale', dest='hw_scale', action='store_true', default=None,
                    help="Scale in the GStreamer pipeline (nvvidconv hardware scaler) instead "
                         "of on the CPU. gstreamer backend only.")
    hw.add_argument('--no-hw-scale', dest='hw_scale', action='store_false',
                    help="Disable GStreamer hardware scaling (CPU cv2 resize).")
    # parse_known_args: ROS arguments (--ros-args -r ...) pass through to rclpy.init.
    args, _ = p.parse_known_args()
    return args


def _apply_overrides(args):
    if args.backend is not None:
        config.STEREO_CAMERA_BACKEND = args.backend
    if args.left is not None:
        config.STEREO_CAMERA_LEFT_DEVICE = int(args.left) if args.left.isdigit() else args.left
    if args.right is not None:
        config.STEREO_CAMERA_RIGHT_DEVICE = int(args.right) if args.right.isdigit() else args.right
    if args.belly is not None:
        config.BELLY_CAMERA_DEVICE = int(args.belly) if args.belly.isdigit() else args.belly
        config.BELLY_CAMERA_ENABLED = True
    if args.no_belly:
        config.BELLY_CAMERA_ENABLED = False
    if args.no_stereo:
        config.STEREO_CAMERA_ENABLED = False
    if args.belly_backend is not None:
        config.BELLY_CAMERA_BACKEND = args.belly_backend
    if args.compressed is not None:
        config.USE_COMPRESSED_IMAGE_TOPICS = args.compressed
    if args.width is not None:
        config.STEREO_CAMERA_CAPTURE_WIDTH = args.width
    if args.height is not None:
        config.STEREO_CAMERA_CAPTURE_HEIGHT = args.height
    if args.fps is not None:
        config.STEREO_CAMERA_FPS = args.fps
    if args.hw_scale is not None:
        config.STEREO_CAMERA_GSTREAMER_HW_SCALE = args.hw_scale


def _log_startup():
    """One block of context at the top of every run, for post-mortem debugging."""
    log.info("=" * 70)
    log.info(f"publish_cameras starting (pid {os.getpid()}, python {platform.python_version()}, "
             f"opencv {cv2.__version__}, {platform.machine()}); log file: {LOG_DIR}/publish_cameras.log")
    log.info(f"argv: {' '.join(sys.argv)}")
    log.info(f"enabled: stereo={config.STEREO_CAMERA_ENABLED} belly={config.BELLY_CAMERA_ENABLED}; "
             f"format={'compressed' if config.USE_COMPRESSED_IMAGE_TOPICS else 'raw'}; "
             f"reopen_after stereo={config.STEREO_CAMERA_REOPEN_AFTER_S}s belly={config.BELLY_CAMERA_REOPEN_AFTER_S}s")
    log.debug(f"stereo: backend={config.STEREO_CAMERA_BACKEND} left={config.STEREO_CAMERA_LEFT_DEVICE} "
              f"right={config.STEREO_CAMERA_RIGHT_DEVICE} "
              f"capture={config.STEREO_CAMERA_CAPTURE_WIDTH}x{config.STEREO_CAMERA_CAPTURE_HEIGHT}"
              f"@{config.STEREO_CAMERA_FPS} out={config.IMG_W_PROCESS}x{config.IMG_H_PROCESS} "
              f"hw_scale={config.STEREO_CAMERA_GSTREAMER_HW_SCALE}")
    log.debug(f"belly: backend={config.BELLY_CAMERA_BACKEND} device={config.BELLY_CAMERA_DEVICE} "
              f"out={config.BELLY_CAMERA_WIDTH}x{config.BELLY_CAMERA_HEIGHT}")
    try:
        nodes = sorted(glob.glob('/dev/video*'))
        log.info("video devices: " + (", ".join(f"{n} ('{v4l2_name(n)}')" for n in nodes) or "none"))
    except Exception:
        log.debug("device listing failed", exc_info=True)


def main():
    sys.excepthook = lambda t, v, tb: log.critical("Uncaught exception", exc_info=(t, v, tb))
    _apply_overrides(_parse_args())
    _log_startup()
    if not (config.STEREO_CAMERA_ENABLED or config.BELLY_CAMERA_ENABLED):
        log.error("Both STEREO_CAMERA_ENABLED and BELLY_CAMERA_ENABLED are off - nothing to publish.")
        sys.exit(1)
    _configure_gstreamer_scaling()

    # Belly camera first (it is the one VO needs), then the stereo pair; each camera
    # on its own, and only what is found gets published.
    belly = None
    if config.BELLY_CAMERA_ENABLED:
        belly = BellyCameraSource()
        if not belly.open():
            belly = None
    stereo = None
    if config.STEREO_CAMERA_ENABLED:
        stereo = StereoCameraSource(exclude=[belly.device] if belly is not None and belly.device else ())
        if not stereo.open():
            stereo = None
    if belly is None and stereo is None:
        log.error("No camera found (belly, left or right) - nothing to publish. Exiting.")
        sys.exit(1)
    has_left = stereo is not None and stereo.has_left
    has_right = stereo is not None and stereo.has_right
    log.info(f"Found: belly={'yes' if belly else 'no'}, left={'yes' if has_left else 'no'}, "
             f"right={'yes' if has_right else 'no'}.")

    rclpy.init()
    node = CameraPublisher(stereo_source=stereo, belly_source=belly)
    _tf_broadcaster = None
    try:
        _tf_broadcaster = _broadcast_calibration_tf(  # keep alive: TRANSIENT_LOCAL
            node, with_left=has_left, with_right=has_right, with_belly=belly is not None)
    except Exception:
        log.exception("tf_static broadcast failed - continuing without it")

    # docker stop sends SIGTERM: leave the spin loop and clean up like Ctrl+C.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    sources = [s for s in (stereo, belly) if s is not None]
    for source in sources:
        source.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        log.info("Shutdown requested.")
    except Exception:
        log.exception("Spin loop crashed")
    finally:
        for source in sources:
            try:
                source.stop()
            except Exception:
                log.exception("Error stopping a capture thread")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.info("publish_cameras stopped.")


if __name__ == '__main__':
    main()

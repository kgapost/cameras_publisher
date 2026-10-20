"""Camera snapshots/recordings and their name-only ROS 2 service adapters.

Uses the capture threads' latest BGR frames; never opens a camera a second time.
All store operations run on the node's default, mutually exclusive callback group.
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import time
from uuid import uuid4

import cv2


class CameraUnavailable(ValueError):
    """The requested camera slot has no camera in this deployment."""


class InvalidCaptureRequest(ValueError):
    """A capture name is unsuitable as a recording ID or filename component."""


class CaptureStore:
    def __init__(self, cameras, directory, fps=25.0, jpeg_quality=90, max_age=3.0):
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError('CAPTURE_VIDEO_FPS must be finite and positive')
        if not 0 <= jpeg_quality <= 100:
            raise ValueError('CAPTURE_JPEG_QUALITY must be between 0 and 100')
        self.cameras = cameras  # number -> (label, source, frame attribute)
        self.directory = Path(directory)
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.max_age = max_age
        self.recordings = {}
        self.errors = {}

    def _camera(self, camera):
        try:
            return self.cameras[camera]
        except (KeyError, TypeError):
            raise CameraUnavailable(f'No camera: camera {camera!r} is unavailable') from None

    @staticmethod
    def validate_name(name):
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', name):
            raise InvalidCaptureRequest('Capture name must be 1-64 characters: start with a letter '
                                        'or digit and use only letters, digits, dots, dashes or underscores')

    def _key(self, camera, name):
        self.validate_name(name)
        self._camera(camera)
        return camera, name

    def _frame(self, camera):
        label, source, attribute = self._camera(camera)
        if source is None:
            raise CameraUnavailable(f'No camera: camera {camera} ({label}) is unavailable')
        with source.lock:
            frame = getattr(source, attribute)
            last = source.last_frame_t
            if frame is None or last is None or time.monotonic() - last > self.max_age:
                raise ValueError(f'Camera {camera} ({label}) has no current frame')
            return frame.copy()

    def _path(self, camera, kind, extension, name):
        label = self._camera(camera)[0]
        folder = self.directory / label
        folder.mkdir(parents=True, exist_ok=True)
        date = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        prefix = kind if name == 'default' else f'{kind}_{name}'
        return folder / f'{prefix}_{date}_{uuid4().hex[:8]}.{extension}'

    def image(self, camera, name='default'):
        self._key(camera, name)
        frame = self._frame(camera)
        path = self._path(camera, 'image', 'jpeg', name)
        try:
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]):
                raise OSError('JPEG encoder could not save the image')
        except Exception:
            path.unlink(missing_ok=True)
            raise
        return f'Saved {path}'

    def start(self, camera, name='default'):
        key = self._key(camera, name)
        if key in self.recordings:
            raise ValueError(f'Camera {camera}, recording {name!r} is already recording')
        frame = self._frame(camera)
        path = self._path(camera, 'video', 'mp4', name)
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'),
                                 self.fps, (width, height))
        try:
            if not writer.isOpened():
                raise OSError('MP4 encoder could not open the video file')
            writer.write(frame)
        except Exception:
            writer.release()
            path.unlink(missing_ok=True)
            raise
        self.recordings[key] = (writer, path, (height, width))
        self.errors.pop(key, None)
        return f'Recording camera {camera} to {path}'

    def stop(self, camera, name='default'):
        key = self._key(camera, name)
        if key not in self.recordings:
            label, source, _ = self._camera(camera)
            if source is None:
                raise CameraUnavailable(f'No camera: camera {camera} ({label}) is unavailable')
            detail = self.errors.get(key, 'start recording first')
            raise ValueError(f'Camera {camera}, name {name!r} is not recording: {detail}')
        writer, path, _ = self.recordings.pop(key)
        writer.release()  # finalizes the MP4 container
        return f'Saved {path}'

    def tick(self):
        """Sample at the configured FPS; a camera failure finalizes its recording."""
        for (camera, name), (writer, path, size) in list(self.recordings.items()):
            try:
                frame = self._frame(camera)
                if frame.shape[:2] != size:
                    raise ValueError('Camera resolution changed during recording')
                writer.write(frame)
            except Exception as exc:
                self.stop(camera, name)
                self.errors[camera, name] = f'Recording stopped ({name!r}): {exc}; file: {path}'

    def _camera_health(self, camera):
        label, source, _ = self._camera(camera)
        try:
            self._frame(camera)
            healthy, message = True, 'Camera delivering frames'
        except ValueError as exc:
            healthy, message = False, str(exc)
        recordings = {name: str(path) for (number, name), (_, path, _) in self.recordings.items()
                      if number == camera}
        errors = {name: error for (number, name), error in self.errors.items()
                  if number == camera}
        return dict(camera=label, available=source is not None, healthy=healthy, message=message,
                    recording=bool(recordings), recording_error='; '.join(errors.values()) or None,
                    recordings=recordings, recording_errors=errors)

    def health(self):
        cameras = {str(number): self._camera_health(number) for number in self.cameras}
        expected = [entry for entry in cameras.values() if entry['camera'] != 'none']
        return dict(healthy=bool(expected) and all(entry['healthy'] for entry in expected),
                    timestamp=datetime.now(timezone.utc).isoformat(), cameras=cameras)

    def close(self):
        for camera, name in list(self.recordings):
            self.stop(camera, name)


class CameraCaptureAPI:
    def __init__(self, node, stereo, belly, config):
        try:
            from camera_capture_interfaces.srv import Capture
        except ImportError as exc:
            raise RuntimeError('Build and source ros_interfaces/camera_capture_interfaces before '
                               'starting the camera API; see README_SWARMER.md. For Docker, '
                               'rebuild the image with docker compose up -d --build.') from exc
        from std_msgs.msg import String

        self.node = node
        self.message_type = String
        mode = config.CAPTURE_API_MODE
        if mode == 'auto':
            mode = 'stereo' if config.STEREO_CAMERA_ENABLED else 'belly'
        if mode == 'stereo':
            cameras = {
                1: ('left', stereo if stereo and stereo.has_left else None, 'left'),
                2: ('right', stereo if stereo and stereo.has_right else None, 'right'),
            }
        else:
            cameras = {1: ('belly', belly, 'frame'), 2: ('none', None, 'frame')}
        self.store = CaptureStore(cameras, config.CAPTURE_DIRECTORY,
                                  config.CAPTURE_VIDEO_FPS, config.CAPTURE_JPEG_QUALITY)
        namespace = config.CAPTURE_API_NAMESPACE.rstrip('/')
        self.publisher = node.create_publisher(String, f'{namespace}/health', 10)
        actions = {'health': lambda name: self._health()}
        for number in (1, 2):
            actions[f'image{number}'] = lambda name, n=number: self.store.image(n, name)
            actions[f'video{number}_start'] = lambda name, n=number: self.store.start(n, name)
            actions[f'video{number}_stop'] = lambda name, n=number: self.store.stop(n, name)
        self.actions = actions
        self.services = [node.create_service(Capture, f'{namespace}/{name}', self._callback(name))
                         for name in actions]
        self.timer = node.create_timer(1.0 / self.store.fps, self._tick)
        self.http = None
        if config.HTTP_API_ENABLED:
            from utils_camera_http import CameraHTTPServer, CommandQueue
            self.commands = CommandQueue(self.execute)
            # The default callback group serializes HTTP commands with ROS
            # services and the recording timer, including multithreaded executors.
            self.command_guard = node.create_guard_condition(self.commands.drain)
            self.commands.wake = self.command_guard.trigger
            self.http = CameraHTTPServer(self.commands, config.HTTP_API_HOST,
                                         config.HTTP_API_PORT, config.HTTP_API_TIMEOUT)
        node.get_logger().info(f'Capture API: {namespace} ({mode}); files: {self.store.directory}')

    def start_http(self):
        if self.http is not None:
            self.http.start()
            self.node.get_logger().info(
                f'HTTP capture API: {self.http.host}:{self.http.port}; Swagger UI at /docs')

    def _health(self):
        payload = json.dumps(self.store.health())
        self.publisher.publish(self.message_type(data=payload))
        return payload

    def execute(self, action, name):
        """Called only on the ROS executor by either transport."""
        try:
            self.store.validate_name(name)
            message = self.actions[action](name)
            result = dict(success=True, message=message)
            if action == 'health':
                result['health'] = json.loads(message)
            return result, 200
        except Exception as exc:
            self.node.get_logger().warning(str(exc))
            if isinstance(exc, CameraUnavailable):
                status = 404
            elif isinstance(exc, InvalidCaptureRequest):
                status = 422
            elif isinstance(exc, ValueError):
                status = 409
            else:
                status = 500
            return dict(success=False, message=str(exc)), status

    def _callback(self, action):
        def callback(request, response):
            result, _ = self.execute(action, name=request.name)
            response.success = result['success']
            response.message = result['message']
            return response
        return callback

    def _tick(self):
        previous = dict(self.store.errors)
        self.store.tick()
        for key, error in self.store.errors.items():
            if previous.get(key) != error:
                self.node.get_logger().warning(error)

    def close(self):
        if self.http is not None:
            self.commands.close()
            self.http.close()
        self.timer.cancel()
        self.store.close()

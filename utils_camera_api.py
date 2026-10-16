"""Camera snapshots/recordings and their ROS 2 Trigger service adapter.

Uses the capture threads' latest BGR frames; never opens a camera a second time.
All store operations run on the node's default, mutually exclusive callback group.
"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from uuid import uuid4

import cv2


class CameraUnavailable(ValueError):
    """The requested camera slot has no camera in this deployment."""


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

    def _frame(self, number):
        label, source, attribute = self.cameras[number]
        if source is None:
            raise CameraUnavailable(f'No camera: camera {number} ({label}) is unavailable')
        with source.lock:
            frame = getattr(source, attribute)
            last = source.last_frame_t
            if frame is None or last is None or time.monotonic() - last > self.max_age:
                raise ValueError(f'Camera {number} ({label}) has no current frame')
            return frame.copy()

    def _path(self, number, kind, extension):
        label = self.cameras[number][0]
        folder = self.directory / label
        folder.mkdir(parents=True, exist_ok=True)
        date = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        return folder / f'{kind}_{date}_{uuid4().hex[:8]}.{extension}'

    def image(self, number):
        frame = self._frame(number)
        path = self._path(number, 'image', 'jpeg')
        try:
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]):
                raise OSError('JPEG encoder could not save the image')
        except Exception:
            path.unlink(missing_ok=True)
            raise
        return f'Saved {path}'

    def start(self, number):
        if number in self.recordings:
            raise ValueError(f'Camera {number} is already recording')
        frame = self._frame(number)
        path = self._path(number, 'video', 'mp4')
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
        self.recordings[number] = (writer, path, (height, width))
        self.errors.pop(number, None)
        return f'Recording camera {number} to {path}'

    def stop(self, number):
        if number not in self.recordings:
            label, source, _ = self.cameras[number]
            if source is None:
                raise CameraUnavailable(f'No camera: camera {number} ({label}) is unavailable')
            detail = self.errors.get(number, 'start recording first')
            raise ValueError(f'Camera {number} is not recording: {detail}')
        writer, path, _ = self.recordings.pop(number)
        writer.release()  # finalizes the MP4 container
        return f'Saved {path}'

    def tick(self):
        """Sample at the configured FPS; a camera failure finalizes its recording."""
        for number, (writer, path, size) in list(self.recordings.items()):
            try:
                frame = self._frame(number)
                if frame.shape[:2] != size:
                    raise ValueError('Camera resolution changed during recording')
                writer.write(frame)
            except Exception as exc:
                self.stop(number)
                self.errors[number] = f'Recording stopped: {exc}; file: {path}'

    def health(self):
        cameras = {}
        for number, (label, source, _) in self.cameras.items():
            try:
                self._frame(number)
                healthy, message = True, 'Camera delivering frames'
            except ValueError as exc:
                healthy, message = False, str(exc)
            cameras[str(number)] = dict(camera=label, available=source is not None,
                                        healthy=healthy, message=message,
                                        recording=number in self.recordings,
                                        recording_error=self.errors.get(number))
        expected = [entry for entry in cameras.values() if entry['camera'] != 'none']
        return dict(healthy=bool(expected) and all(entry['healthy'] for entry in expected),
                    timestamp=datetime.now(timezone.utc).isoformat(), cameras=cameras)

    def close(self):
        for number in list(self.recordings):
            self.stop(number)


class CameraCaptureAPI:
    def __init__(self, node, stereo, belly, config):
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

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
        actions = {'health': self._health}
        for number in (1, 2):
            actions[f'image{number}'] = lambda n=number: self.store.image(n)
            actions[f'video{number}_start'] = lambda n=number: self.store.start(n)
            actions[f'video{number}_stop'] = lambda n=number: self.store.stop(n)
        self.actions = actions
        self.services = [node.create_service(Trigger, f'{namespace}/{name}', self._callback(name))
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

    def execute(self, name):
        """Called only on the ROS executor by either transport."""
        try:
            message = self.actions[name]()
            result = dict(success=True, message=message)
            if name == 'health':
                result['health'] = json.loads(message)
            return result, 200
        except Exception as exc:
            self.node.get_logger().warning(str(exc))
            if isinstance(exc, CameraUnavailable):
                status = 404
            elif isinstance(exc, ValueError):
                status = 409
            else:
                status = 500
            return dict(success=False, message=str(exc)), status

    def _callback(self, name):
        def callback(request, response):
            result, _ = self.execute(name)
            response.success = result['success']
            response.message = result['message']
            return response
        return callback

    def _tick(self):
        previous = dict(self.store.errors)
        self.store.tick()
        for number, error in self.store.errors.items():
            if previous.get(number) != error:
                self.node.get_logger().warning(error)

    def close(self):
        if self.http is not None:
            self.commands.close()
            self.http.close()
        self.timer.cancel()
        self.store.close()

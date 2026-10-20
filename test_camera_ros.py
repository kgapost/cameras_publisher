"""Real ROS 2 service round trips with generated interfaces and synthetic frames."""

import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from uuid import uuid4

try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from camera_capture_interfaces.srv import Capture
    from utils_camera_api import CameraCaptureAPI
    ROS_AVAILABLE, ROS_SKIP_REASON = True, ''
except ImportError as exc:
    ROS_AVAILABLE, ROS_SKIP_REASON = False, f'Source ROS and the built capture interfaces: {exc}'


@unittest.skipUnless(ROS_AVAILABLE, ROS_SKIP_REASON)
class ROSCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=211)
        token = uuid4().hex[:8]
        self.server = Node(f'capture_server_{token}', context=self.context)
        self.client = Node(f'capture_client_{token}', context=self.context)
        self.namespace = f'/camera_test_{token}/capture'
        frame = np.full((48, 64, 3), 100, dtype=np.uint8)
        self.source = SimpleNamespace(lock=threading.Lock(), frame=frame,
                                      last_frame_t=time.monotonic())
        config = SimpleNamespace(CAPTURE_API_MODE='belly', STEREO_CAMERA_ENABLED=False,
                                 CAPTURE_DIRECTORY=self.temp.name, CAPTURE_VIDEO_FPS=25,
                                 CAPTURE_JPEG_QUALITY=90, HTTP_API_ENABLED=False,
                                 CAPTURE_API_NAMESPACE=self.namespace)
        self.api = CameraCaptureAPI(self.server, None, self.source, config)
        self.executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        self.executor.add_node(self.server)
        self.executor.add_node(self.client)
        self.clients = {}
        self.addCleanup(self._close)

    def _close(self):
        self.api.close()
        self.executor.shutdown()
        self.client.destroy_node()
        self.server.destroy_node()
        self.context.shutdown()

    def _submit(self, action, name='default'):
        self.source.last_frame_t = time.monotonic()
        if action not in self.clients:
            client = self.client.create_client(Capture, f'{self.namespace}/{action}')
            self.assertTrue(client.wait_for_service(timeout_sec=3), f'{action} service not discovered')
            self.clients[action] = client
        request = Capture.Request(name=name)
        return self.clients[action].call_async(request)

    def _result(self, future):
        self.executor.spin_until_future_complete(future, timeout_sec=3)
        self.assertTrue(future.done(), 'ROS service call timed out')
        return future.result()

    def _call(self, action, **parameters):
        return self._result(self._submit(action, **parameters))

    def test_concurrent_modules_record_and_stop_independently_over_ros(self):
        self.assertEqual(Capture.Request.get_fields_and_field_types(), {'name': 'string'})
        self.assertEqual(len(self.api.services), 7)
        starts = [self._submit('video1_start', name=name) for name in ('odometry', 'inspection')]
        for future in starts:
            response = self._result(future)
            self.assertTrue(response.success, response.message)
        self.assertFalse(self._call('video1_start', name='odometry').success)
        self.assertFalse(self._call('video1_stop').success)
        self.assertFalse(self._call('video1_stop', name='unknown').success)
        health = json.loads(self._call('health').message)['cameras']['1']
        self.assertEqual(set(health['recordings']), {'odometry', 'inspection'})
        self.assertTrue(self._call('video1_stop', name='odometry').success)
        health = json.loads(self._call('health').message)['cameras']['1']
        self.assertEqual(set(health['recordings']), {'inspection'})
        self.assertTrue(self._call('video1_stop', name='inspection').success)
        self.assertFalse(self._call('video1_stop', name='inspection').success)
        videos = list(Path(self.temp.name).glob('belly/video_*.mp4'))
        self.assertEqual(len(videos), 2)
        for path in videos:
            capture = cv2.VideoCapture(str(path))
            try:
                ok, frame = capture.read()
                self.assertTrue(ok)
                self.assertEqual(frame.shape, (48, 64, 3))
            finally:
                capture.release()

    def test_named_ros_snapshot_and_invalid_requests(self):
        self.assertTrue(self._call('image1', name='inspection.snapshot').success)
        images = list(Path(self.temp.name).glob('belly/image_inspection.snapshot_*.jpeg'))
        self.assertEqual(len(images), 1)
        self.assertEqual(cv2.imread(str(images[0])).shape, (48, 64, 3))
        for action in self.api.actions:
            for name in ('', '../escape'):
                with self.subTest(action=action, name=name):
                    self.assertFalse(self._call(action, name=name).success)
        for action in ('image2', 'video2_start', 'video2_stop'):
            self.assertFalse(self._call(action, name='mission').success)
        self.assertEqual(len(list(Path(self.temp.name).rglob('*.*'))), 1)

    def test_default_is_an_explicit_name_on_the_same_ros_services(self):
        self.assertTrue(self._call('video1_start', name='default').success)
        self.assertTrue(self._call('video1_start', name='module').success)
        self.assertTrue(self._call('video1_stop', name='default').success)
        health = json.loads(self._call('health').message)['cameras']['1']
        self.assertEqual(set(health['recordings']), {'module'})
        self.assertTrue(self._call('video1_stop', name='module').success)


if __name__ == '__main__':
    unittest.main()

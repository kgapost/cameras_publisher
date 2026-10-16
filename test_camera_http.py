"""HTTP, command-queue and real JPEG/MP4 integration tests without camera hardware."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import socket
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    import cv2
    import httpx
    import numpy as np
    from fastapi.testclient import TestClient
except ImportError as exc:
    raise unittest.SkipTest(f'Install dev requirements for HTTP integration tests: {exc}')

from utils_camera_api import CameraCaptureAPI
from utils_camera_http import APIUnavailable, CameraHTTPServer, CommandQueue, create_app


class QueueTests(unittest.TestCase):
    def test_submit_waits_for_executor_and_executes_on_drain_thread(self):
        seen = []
        commands = CommandQueue(lambda name: seen.append((name, threading.get_ident())))
        future = commands.submit('image1')
        self.assertFalse(future.done())
        self.assertEqual(seen, [])
        commands.drain()
        self.assertEqual(seen, [('image1', threading.get_ident())])
        self.assertIsNone(future.result())

    def test_cancelled_command_has_no_side_effects(self):
        execute = Mock()
        commands = CommandQueue(execute)
        future = commands.submit('video1_start')
        self.assertTrue(future.cancel())
        commands.drain()
        execute.assert_not_called()

    def test_overload_and_shutdown_reject_commands(self):
        commands = CommandQueue(Mock(), capacity=1)
        future = commands.submit('health')
        with self.assertRaisesRegex(APIUnavailable, 'full'):
            commands.submit('image1')
        commands.close()
        with self.assertRaisesRegex(APIUnavailable, 'shutting down'):
            future.result()
        with self.assertRaises(APIUnavailable):
            commands.submit('health')

    def test_batches_yield_and_wake_for_remaining_commands(self):
        execute = Mock()
        commands = CommandQueue(execute)
        commands.wake = Mock()
        for _ in range(10):
            commands.submit('health')
        commands.wake.reset_mock()
        commands.drain()
        self.assertEqual(execute.call_count, 8)
        commands.wake.assert_called_once()
        commands.drain()
        self.assertEqual(execute.call_count, 10)


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        frame = np.full((48, 64, 3), 100, dtype=np.uint8)
        self.source = SimpleNamespace(lock=threading.Lock(), frame=frame, left=frame,
                                      right=frame, has_left=True, has_right=True,
                                      last_frame_t=time.monotonic())
        self.node = Mock()
        self.config = SimpleNamespace(CAPTURE_API_MODE='belly', STEREO_CAMERA_ENABLED=False,
                                      CAPTURE_DIRECTORY=self.temp.name, CAPTURE_VIDEO_FPS=25,
                                      CAPTURE_JPEG_QUALITY=90, HTTP_API_ENABLED=False,
                                      CAPTURE_API_NAMESPACE='/camera/capture')
        with patch.dict(sys.modules, {
            'std_msgs.msg': SimpleNamespace(String=lambda **kw: SimpleNamespace(**kw)),
            'std_srvs.srv': SimpleNamespace(Trigger=object()),
        }):
            self.adapter = CameraCaptureAPI(self.node, self.source, self.source, self.config)
        self.commands = CommandQueue(self.adapter.execute)
        self.wake = threading.Event()
        self.commands.wake = self.wake.set
        self.done = threading.Event()
        self.executor = threading.Thread(target=self._spin)
        self.executor.start()
        self.client = TestClient(create_app(self.commands, timeout=1))
        self.addCleanup(self._close)

    def _spin(self):
        while not self.done.is_set():
            self.wake.wait(0.1)
            self.wake.clear()
            self.commands.drain()

    def _close(self):
        self.client.close()
        self.commands.close()
        self.done.set()
        self.wake.set()
        self.executor.join(timeout=2)
        self.adapter.close()

    def test_docs_and_openapi_have_exactly_seven_post_commands(self):
        self.assertEqual(self.client.get('/docs').status_code, 200)
        schema = self.client.get('/openapi.json').json()
        expected = {'health', 'image1', 'image2', 'video1_start', 'video1_stop',
                    'video2_start', 'video2_stop'}
        self.assertEqual(set(schema['paths']), {f'/api/{name}' for name in expected})
        for path in schema['paths'].values():
            self.assertEqual(set(path), {'post'})
            self.assertNotIn('parameters', path['post'])
            self.assertEqual(set(path['post']['responses']), {'200', '404', '409', '500', '503', '504'})

    def test_http_health_publishes_ros_message(self):
        response = self.client.post('/api/health')
        self.assertEqual(response.status_code, 200)
        result = response.json()
        self.assertTrue(result['success'])
        self.assertTrue(result['health']['healthy'])
        published = self.node.create_publisher.return_value.publish.call_args.args[0].data
        self.assertEqual(result['message'], published)
        self.assertTrue(json.loads(published)['healthy'])

    def test_real_jpeg_and_absent_camera_warnings(self):
        response = self.client.post('/api/image1')
        self.assertEqual(response.status_code, 200)
        images = list(Path(self.temp.name).glob('belly/image_*.jpeg'))
        self.assertEqual(len(images), 1)
        self.assertEqual(cv2.imread(str(images[0])).shape, (48, 64, 3))
        for name in ('image2', 'video2_start', 'video2_stop'):
            response = self.client.post(f'/api/{name}')
            self.assertEqual(response.status_code, 404)
            self.assertFalse(response.json()['success'])
        self.assertEqual(len(list(Path(self.temp.name).rglob('*.*'))), 1)

    def test_real_mp4_and_recording_state_shared_with_ros(self):
        self.assertEqual(self.client.post('/api/video1_start').status_code, 200)
        # A ROS service callback is executed on the same executor as HTTP commands.
        ros_stop = self.adapter._callback('video1_stop')
        original = self.commands.execute
        self.commands.execute = lambda name: (ros_stop(None, SimpleNamespace())
                                              if name == 'ros_stop' else original(name))
        stopped = self.commands.submit('ros_stop').result(timeout=1)
        self.assertTrue(stopped.success)
        self.assertEqual(self.client.post('/api/video1_stop').status_code, 409)
        videos = list(Path(self.temp.name).glob('belly/video_*.mp4'))
        self.assertEqual(len(videos), 1)
        capture = cv2.VideoCapture(str(videos[0]))
        try:
            ok, frame = capture.read()
            self.assertTrue(ok)
            self.assertEqual(frame.shape, (48, 64, 3))
        finally:
            capture.release()

    def test_concurrent_starts_create_only_one_recording(self):
        with ThreadPoolExecutor(max_workers=2) as pool:
            responses = list(pool.map(lambda _: self.client.post('/api/video1_start'), range(2)))
        self.assertEqual(sorted(response.status_code for response in responses), [200, 409])
        self.assertEqual(self.client.post('/api/video1_stop').status_code, 200)
        self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_*.mp4'))), 1)

    def test_storage_failure_and_stale_camera_status_codes(self):
        with patch.object(self.adapter.store, 'image', side_effect=OSError('disk full')):
            response = self.client.post('/api/image1')
        self.assertEqual(response.status_code, 500)
        self.assertIn('disk full', response.json()['message'])
        self.source.last_frame_t -= 10
        self.assertEqual(self.client.post('/api/image1').status_code, 409)

    def test_timeout_cancels_pending_command(self):
        commands = CommandQueue(Mock())
        with TestClient(create_app(commands, timeout=0.01)) as client:
            response = client.post('/api/video1_start')
        self.assertEqual(response.status_code, 504)
        self.assertIn('cancelled', response.json()['message'])
        commands.drain()
        commands.execute.assert_not_called()

    def test_unavailable_queue_returns_503(self):
        self.commands.close()
        response = self.client.post('/api/health')
        self.assertEqual(response.status_code, 503)

    def test_live_server_start_request_and_shutdown(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        server = CameraHTTPServer(self.commands, '127.0.0.1', port, 1)
        try:
            server.start()
            response = httpx.post(f'http://127.0.0.1:{port}/api/health', timeout=2)
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.json()['health']['healthy'])
        finally:
            server.close()
        self.assertFalse(server.thread.is_alive())

    def test_belly_and_stereo_servers_run_simultaneously_with_separate_state(self):
        stereo_node = Mock()
        config = SimpleNamespace(**vars(self.config))
        config.CAPTURE_API_MODE = 'stereo'
        config.CAPTURE_API_NAMESPACE = '/camera/stereo/capture'
        with patch.dict(sys.modules, {
            'std_msgs.msg': SimpleNamespace(String=lambda **kw: SimpleNamespace(**kw)),
            'std_srvs.srv': SimpleNamespace(Trigger=object()),
        }):
            stereo = CameraCaptureAPI(stereo_node, self.source, None, config)
        commands = CommandQueue(stereo.execute)
        event, done = threading.Event(), threading.Event()
        commands.wake = event.set

        def spin():
            while not done.is_set():
                event.wait(0.1)
                event.clear()
                commands.drain()

        executor = threading.Thread(target=spin)
        executor.start()
        servers = []
        try:
            for queue in (self.commands, commands):
                with socket.socket() as probe:
                    probe.bind(('127.0.0.1', 0))
                    port = probe.getsockname()[1]
                server = CameraHTTPServer(queue, '127.0.0.1', port, 1)
                servers.append(server)
                server.start()
            belly_url, stereo_url = [f'http://127.0.0.1:{s.port}' for s in servers]
            self.assertEqual(httpx.post(f'{belly_url}/api/image2').status_code, 404)
            self.assertEqual(httpx.post(f'{stereo_url}/api/image2').status_code, 200)
            self.assertEqual(httpx.post(f'{belly_url}/api/video1_start').status_code, 200)
            stereo_health = httpx.post(f'{stereo_url}/api/health').json()['health']
            self.assertEqual(stereo_health['cameras']['1']['camera'], 'left')
            self.assertFalse(stereo_health['cameras']['1']['recording'])
            self.assertEqual(httpx.post(f'{stereo_url}/api/video1_start').status_code, 200)
            self.assertEqual(httpx.post(f'{belly_url}/api/video1_stop').status_code, 200)
            self.assertTrue(httpx.post(f'{stereo_url}/api/health').json()['health']['cameras']['1']['recording'])
            self.assertEqual(httpx.post(f'{stereo_url}/api/video1_stop').status_code, 200)
            self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_*.mp4'))), 1)
            self.assertEqual(len(list(Path(self.temp.name).glob('left/video_*.mp4'))), 1)
            self.assertEqual(len(list(Path(self.temp.name).glob('right/image_*.jpeg'))), 1)
        finally:
            commands.close()
            for server in servers:
                server.close()
            done.set()
            event.set()
            executor.join(timeout=2)
            stereo.close()


class ComposeTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('docker'), 'Docker Compose CLI not installed')
    def test_dual_deployment_isolates_cameras_ports_and_ros_names(self):
        output = subprocess.check_output(['docker', 'compose', '-f', 'docker-compose.dual.yml',
                                          'config', '--format', 'json'], text=True,
                                         cwd=Path(__file__).parent)
        services = json.loads(output)['services']
        belly = services['cameras-publisher-belly']['environment']
        stereo = services['cameras-publisher-stereo']['environment']
        self.assertEqual((belly['CP_BELLY_CAMERA_ENABLED'], belly['CP_STEREO_CAMERA_ENABLED']), ('1', '0'))
        self.assertEqual((stereo['CP_BELLY_CAMERA_ENABLED'], stereo['CP_STEREO_CAMERA_ENABLED']), ('0', '1'))
        for setting in ('CP_NODE_NAME', 'CP_HTTP_API_PORT', 'CP_CAPTURE_API_NAMESPACE'):
            self.assertNotEqual(belly[setting], stereo[setting])
        self.assertEqual(belly['ROS_DOMAIN_ID'], stereo['ROS_DOMAIN_ID'])


if __name__ == '__main__':
    unittest.main()

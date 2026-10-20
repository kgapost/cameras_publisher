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

    def test_named_parameters_are_dispatched_only_on_executor(self):
        execute = Mock()
        commands = CommandQueue(execute)
        commands.submit('video1_start', name='odometry')
        cancelled = commands.submit('video1_stop', name='inspection')
        cancelled.cancel()
        execute.assert_not_called()
        commands.drain()
        execute.assert_called_once_with('video1_start', name='odometry')


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
            'camera_capture_interfaces.srv': SimpleNamespace(Capture=object()),
            'std_msgs.msg': SimpleNamespace(String=lambda **kw: SimpleNamespace(**kw)),
        }):
            self.adapter = CameraCaptureAPI(self.node, None, self.source, self.config)
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

    def test_docs_and_openapi_have_exactly_seven_name_only_post_commands(self):
        self.assertEqual(self.client.get('/docs').status_code, 200)
        schema = self.client.get('/openapi.json').json()
        expected = {'health', 'image1', 'image2', 'video1_start', 'video1_stop',
                    'video2_start', 'video2_stop'}
        self.assertEqual(set(schema['paths']), {f'/api/{name}' for name in expected})
        for path in schema['paths'].values():
            self.assertEqual(set(path), {'post'})
            command = path['post']
            self.assertNotIn('parameters', command)
            self.assertEqual(set(command['responses']), {'200', '404', '409', '422', '500', '503', '504'})
            self.assertTrue(command['requestBody']['required'])
            body = command['requestBody']['content']['application/json']['schema']
            self.assertEqual(body['$ref'], '#/components/schemas/CaptureName')
        capture = schema['components']['schemas']['CaptureName']
        self.assertEqual(capture['required'], ['name'])
        self.assertEqual(set(capture['properties']), {'name'})
        self.assertFalse(capture['additionalProperties'])

    def test_http_health_publishes_ros_message(self):
        response = self.client.post('/api/health', json={'name': 'default'})
        self.assertEqual(response.status_code, 200)
        result = response.json()
        self.assertTrue(result['success'])
        self.assertTrue(result['health']['healthy'])
        published = self.node.create_publisher.return_value.publish.call_args.args[0].data
        self.assertEqual(result['message'], published)
        self.assertTrue(json.loads(published)['healthy'])

    def test_real_jpeg_and_absent_camera_warnings(self):
        response = self.client.post('/api/image1', json={'name': 'default'})
        self.assertEqual(response.status_code, 200)
        images = list(Path(self.temp.name).glob('belly/image_*.jpeg'))
        self.assertEqual(len(images), 1)
        self.assertEqual(cv2.imread(str(images[0])).shape, (48, 64, 3))
        for name in ('image2', 'video2_start', 'video2_stop'):
            response = self.client.post(f'/api/{name}', json={'name': 'default'})
            self.assertEqual(response.status_code, 404)
            self.assertFalse(response.json()['success'])
        self.assertEqual(len(list(Path(self.temp.name).rglob('*.*'))), 1)

    def test_real_mp4_and_recording_state_shared_with_ros(self):
        self.assertEqual(self.client.post('/api/video1_start', json={'name': 'default'}).status_code, 200)
        # A ROS service callback is executed on the same executor as HTTP commands.
        ros_stop = self.adapter._callback('video1_stop')
        original = self.commands.execute
        self.commands.execute = lambda action, **parameters: (
            ros_stop(SimpleNamespace(name=parameters['name']), SimpleNamespace())
            if action == 'ros_stop' else original(action, **parameters))
        stopped = self.commands.submit('ros_stop', name='default').result(timeout=1)
        self.assertTrue(stopped.success)
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'default'}).status_code, 409)
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
            responses = list(pool.map(lambda _: self.client.post('/api/video1_start', json={'name': 'default'}), range(2)))
        self.assertEqual(sorted(response.status_code for response in responses), [200, 409])
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'default'}).status_code, 200)
        self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_*.mp4'))), 1)

    def test_modules_record_same_camera_and_stop_only_their_named_video(self):
        def start(name):
            return self.client.post('/api/video1_start', json={'name': name})

        with ThreadPoolExecutor(max_workers=2) as pool:
            responses = list(pool.map(start, ('odometry', 'inspection')))
        self.assertEqual([response.status_code for response in responses], [200, 200])
        self.assertEqual(start('odometry').status_code, 409)
        self.assertEqual(self.client.post('/api/video1_stop', json={
            'name': 'unknown'}).status_code, 409)
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'default'}).status_code, 409)
        health = self.client.post('/api/health', json={'name': 'default'}).json()['health']
        active = health['cameras']['1']['recordings']
        self.assertEqual(set(active), {'odometry', 'inspection'})
        response = self.client.post('/api/video1_stop', json={'name': 'odometry'})
        self.assertEqual(response.status_code, 200)
        health = self.client.post('/api/health', json={'name': 'default'}).json()['health']
        self.assertTrue(health['cameras']['1']['recording'])
        self.assertEqual(set(health['cameras']['1']['recordings']), {'inspection'})
        # Execute a recording timer tick on the same queue before finalizing the second video.
        original = self.commands.execute
        self.commands.execute = lambda action, **parameters: (
            self.adapter.store.tick() if action == 'tick' else original(action, **parameters))
        self.commands.submit('tick').result(timeout=1)
        self.assertEqual(self.client.post('/api/video1_stop', json={
            'name': 'inspection'}).status_code, 200)
        self.assertFalse(self.client.post('/api/health', json={'name': 'default'}).json()['health']['cameras']['1']['recording'])
        self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_*.mp4'))), 2)
        for name, path in active.items():
            self.assertTrue(Path(path).name.startswith(f'video_{name}_'))
            capture = cv2.VideoCapture(path)
            try:
                frames = 0
                while capture.read()[0]:
                    frames += 1
                self.assertEqual(frames, 2 if name == 'inspection' else 1)
            finally:
                capture.release()

    def test_default_and_module_names_are_independent_and_reusing_name_creates_new_file(self):
        self.assertEqual(self.client.post('/api/video1_start', json={'name': 'mission'}).status_code, 200)
        self.assertEqual(self.client.post('/api/video1_start', json={'name': 'default'}).status_code, 200)
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'default'}).status_code, 200)
        health = self.client.post('/api/health', json={'name': 'default'}).json()['health']
        self.assertEqual(set(health['cameras']['1']['recordings']), {'mission'})
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'mission'}).status_code, 200)
        first = list(Path(self.temp.name).glob('belly/video_mission_*.mp4'))
        self.assertEqual(self.client.post('/api/video1_start', json={'name': 'mission'}).status_code, 200)
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'mission'}).status_code, 200)
        self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_mission_*.mp4'))), 2)
        self.assertTrue(first[0].is_file())

    def test_snapshot_and_name_only_request_validation_have_no_other_side_effects(self):
        response = self.client.post('/api/image1', json={'name': 'mission.snapshot'})
        self.assertEqual(response.status_code, 200)
        images = list(Path(self.temp.name).glob('belly/image_mission.snapshot_*.jpeg'))
        self.assertEqual(len(images), 1)
        self.assertEqual(cv2.imread(str(images[0])).shape, (48, 64, 3))
        actions = ('health', 'image1', 'image2', 'video1_start', 'video2_start', 'video1_stop', 'video2_stop')
        for action in actions:
            self.assertEqual(self.client.post(f'/api/{action}').status_code, 422)
            for body in ({}, {'camera': 'belly'}, {'name': ''}, {'name': '../escape'},
                         {'name': 'name\n'}, {'name': 'a' * 65}, {'name': None},
                         {'camera': 'belly', 'name': 'mission'},
                         {'name': 'mission', 'module': 'typo'}):
                with self.subTest(action=action, body=body):
                    self.assertEqual(self.client.post(f'/api/{action}', json=body).status_code, 422)
        for action in ('image2', 'video2_start', 'video2_stop'):
            self.assertEqual(self.client.post(f'/api/{action}', json={'name': 'mission'}).status_code, 404)
        for action in ('image', 'video_start', 'video_stop'):
            self.assertEqual(self.client.post(f'/api/{action}', json={'name': 'mission'}).status_code, 404)
        self.assertEqual(len(list(Path(self.temp.name).rglob('*.*'))), 1)

    def test_ros_and_http_control_the_same_named_recording(self):
        original = self.commands.execute

        def execute(action, **parameters):
            if action in ('ros_start', 'ros_stop'):
                service = 'video1_start' if action == 'ros_start' else 'video1_stop'
                request = SimpleNamespace(name=parameters['name'])
                return self.adapter._callback(service)(request, SimpleNamespace())
            return original(action, **parameters)

        self.commands.execute = execute
        started = self.commands.submit('ros_start', name='mission').result(timeout=1)
        self.assertTrue(started.success)
        self.assertEqual(self.client.post('/api/video1_stop', json={'name': 'mission'}).status_code, 200)
        self.assertEqual(self.client.post('/api/video1_start', json={'name': 'inspection'}).status_code, 200)
        stopped = self.commands.submit('ros_stop', name='inspection').result(timeout=1)
        self.assertTrue(stopped.success)
        self.assertEqual(len(list(Path(self.temp.name).glob('belly/video_*.mp4'))), 2)

    def test_storage_failure_and_stale_camera_status_codes(self):
        with patch.object(self.adapter.store, 'image', side_effect=OSError('disk full')):
            response = self.client.post('/api/image1', json={'name': 'default'})
        self.assertEqual(response.status_code, 500)
        self.assertIn('disk full', response.json()['message'])
        self.source.last_frame_t -= 10
        self.assertEqual(self.client.post('/api/image1', json={'name': 'default'}).status_code, 409)

    def test_timeout_cancels_pending_command(self):
        commands = CommandQueue(Mock())
        with TestClient(create_app(commands, timeout=0.01)) as client:
            response = client.post('/api/video1_start', json={'name': 'default'})
        self.assertEqual(response.status_code, 504)
        self.assertIn('cancelled', response.json()['message'])
        commands.drain()
        commands.execute.assert_not_called()

    def test_unavailable_queue_returns_503(self):
        self.commands.close()
        response = self.client.post('/api/health', json={'name': 'default'})
        self.assertEqual(response.status_code, 503)

    def test_live_server_start_request_and_shutdown(self):
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        server = CameraHTTPServer(self.commands, '127.0.0.1', port, 1)
        try:
            server.start()
            response = httpx.post(f'http://127.0.0.1:{port}/api/health', json={'name': 'default'}, timeout=2)
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
            'camera_capture_interfaces.srv': SimpleNamespace(Capture=object()),
            'std_msgs.msg': SimpleNamespace(String=lambda **kw: SimpleNamespace(**kw)),
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
            self.assertEqual(httpx.post(f'{belly_url}/api/image2', json={'name': 'default'}).status_code, 404)
            self.assertEqual(httpx.post(f'{stereo_url}/api/image2', json={'name': 'default'}).status_code, 200)
            self.assertEqual(httpx.post(f'{belly_url}/api/video1_start', json={'name': 'default'}).status_code, 200)
            stereo_health = httpx.post(f'{stereo_url}/api/health', json={'name': 'default'}).json()['health']
            self.assertEqual(stereo_health['cameras']['1']['camera'], 'left')
            self.assertFalse(stereo_health['cameras']['1']['recording'])
            self.assertEqual(httpx.post(f'{stereo_url}/api/video1_start', json={'name': 'default'}).status_code, 200)
            self.assertEqual(httpx.post(f'{belly_url}/api/video1_stop', json={'name': 'default'}).status_code, 200)
            self.assertTrue(httpx.post(f'{stereo_url}/api/health', json={'name': 'default'}).json()['health']['cameras']['1']['recording'])
            self.assertEqual(httpx.post(f'{stereo_url}/api/video1_stop', json={'name': 'default'}).status_code, 200)
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

"""API state/ROS adapter tests; fake encoders allow running without ROS or cameras."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


# Load the module with a stand-in for OpenCV without changing other imports.
spec = importlib.util.spec_from_file_location('camera_api_under_test',
                                             Path(__file__).with_name('utils_camera_api.py'))
api = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'cv2': Mock()}):
    spec.loader.exec_module(api)


def source():
    frame = Mock(shape=(40, 64, 3))
    frame.copy.return_value = frame
    return SimpleNamespace(lock=threading.Lock(), frame=frame, left=frame, right=frame,
                           has_left=True, has_right=True, last_frame_t=time.monotonic())


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.cv = Mock()
        self.cv.imwrite.return_value = True
        self.writer = self.cv.VideoWriter.return_value
        self.writer.isOpened.return_value = True
        self.cv_patch = patch.object(api, 'cv2', self.cv)
        self.cv_patch.start()
        self.addCleanup(self.cv_patch.stop)
        self.src = source()
        self.store = api.CaptureStore({1: ('belly', self.src, 'frame'),
                                       2: ('none', None, 'frame')}, self.temp.name)
        self.addCleanup(self.store.close)

    def test_image_path_compression_and_unique_names(self):
        self.store.image(1)
        self.store.image(1)
        calls = self.cv.imwrite.call_args_list
        paths = [Path(call.args[0]) for call in calls]
        self.assertNotEqual(paths[0], paths[1])
        self.assertEqual(paths[0].parent, Path(self.temp.name) / 'belly')
        self.assertTrue(paths[0].name.startswith('image_'))
        self.assertEqual(paths[0].suffix, '.jpeg')
        self.assertEqual(calls[0].args[2], [self.cv.IMWRITE_JPEG_QUALITY, 90])

    def test_absent_camera_all_operations_warn_without_writes(self):
        for action in (self.store.image, self.store.start, self.store.stop):
            with self.subTest(action=action.__name__), self.assertRaisesRegex(ValueError, 'No camera'):
                action(2)
        self.cv.imwrite.assert_not_called()
        self.cv.VideoWriter.assert_not_called()

    def test_start_duplicate_stop_and_duplicate_stop(self):
        self.assertIn('.mp4', self.store.start(1))
        with self.assertRaisesRegex(ValueError, 'already recording'):
            self.store.start(1)
        self.store.tick()
        self.assertEqual(self.writer.write.call_count, 2)
        self.assertIn('Saved', self.store.stop(1))
        self.writer.release.assert_called_once()
        with self.assertRaisesRegex(ValueError, 'not recording'):
            self.store.stop(1)

    def test_stop_before_start(self):
        with self.assertRaisesRegex(ValueError, 'start recording first'):
            self.store.stop(1)

    def test_stale_frame_rejects_image_and_start(self):
        self.src.last_frame_t -= 10
        for action in (self.store.image, self.store.start):
            with self.assertRaisesRegex(ValueError, 'no current frame'):
                action(1)

    def test_no_first_frame_rejects_image(self):
        self.src.frame = None
        with self.assertRaisesRegex(ValueError, 'no current frame'):
            self.store.image(1)

    def test_failed_jpeg_reports_error(self):
        self.cv.imwrite.return_value = False
        with self.assertRaisesRegex(OSError, 'JPEG encoder'):
            self.store.image(1)

    def test_failed_video_encoder_does_not_start(self):
        self.writer.isOpened.return_value = False
        with self.assertRaisesRegex(OSError, 'MP4 encoder'):
            self.store.start(1)
        self.assertFalse(self.store.recordings)
        self.writer.release.assert_called_once()

    def test_storage_failure_does_not_start(self):
        self.store.directory = Path(self.temp.name) / 'file'
        self.store.directory.write_text('not a directory')
        with self.assertRaises(OSError):
            self.store.start(1)
        self.assertFalse(self.store.recordings)

    def test_stereo_recordings_are_independent(self):
        self.store.cameras = {1: ('left', self.src, 'left'), 2: ('right', self.src, 'right')}
        writers = [Mock(), Mock()]
        self.cv.VideoWriter.side_effect = writers
        self.store.start(1)
        self.store.start(2)
        self.store.stop(1)
        writers[0].release.assert_called_once()
        writers[1].release.assert_not_called()
        self.assertIn((2, 'default'), self.store.recordings)
        self.store.close()
        writers[1].release.assert_called_once()

    def test_stall_finalizes_recording_and_sets_health_error(self):
        self.store.start(1)
        self.src.last_frame_t -= 10
        self.store.tick()
        self.writer.release.assert_called_once()
        self.assertFalse(self.store.recordings)
        status = self.store.health()
        self.assertFalse(status['healthy'])
        self.assertIn('Recording stopped', status['cameras']['1']['recording_error'])
        self.src.last_frame_t = time.monotonic()
        self.store.start(1)
        self.assertNotIn((1, 'default'), self.store.errors)

    def test_resolution_change_finalizes_video(self):
        self.store.start(1)
        self.src.frame.shape = (80, 128, 3)
        self.store.tick()
        self.assertFalse(self.store.recordings)
        self.assertIn('resolution changed', self.store.errors[1, 'default'])

    def test_named_recordings_share_camera_and_stop_independently(self):
        writers = [Mock(), Mock()]
        self.cv.VideoWriter.side_effect = writers
        self.store.start(1, 'odometry')
        self.store.start(1, 'inspection')
        with self.assertRaisesRegex(ValueError, 'already recording'):
            self.store.start(1, 'odometry')
        self.store.tick()
        self.assertEqual([writer.write.call_count for writer in writers], [2, 2])
        with self.assertRaisesRegex(ValueError, 'not recording'):
            self.store.stop(1, 'unknown')
        with self.assertRaisesRegex(ValueError, 'not recording'):
            self.store.stop(1)
        self.store.stop(1, 'odometry')
        writers[0].release.assert_called_once()
        writers[1].release.assert_not_called()
        self.store.tick()
        self.assertEqual([writer.write.call_count for writer in writers], [2, 3])
        health = self.store.health()
        self.assertTrue(health['cameras']['1']['recording'])
        self.assertEqual(set(health['cameras']['1']['recordings']), {'inspection'})
        self.store.close()
        writers[1].release.assert_called_once()

    def test_same_recording_name_on_different_cameras(self):
        self.store.cameras = {1: ('left', self.src, 'left'), 2: ('right', self.src, 'right')}
        writers = [Mock(), Mock()]
        self.cv.VideoWriter.side_effect = writers
        self.store.start(1, 'mission')
        self.store.start(2, 'mission')
        self.store.stop(1, 'mission')
        writers[0].release.assert_called_once()
        writers[1].release.assert_not_called()
        self.assertEqual(set(self.store.recordings), {(2, 'mission')})

    def test_writer_failure_stops_only_its_recording(self):
        writers = [Mock(), Mock(), Mock()]
        self.cv.VideoWriter.side_effect = writers
        self.store.start(1, 'odometry')
        self.store.start(1, 'inspection')
        writers[0].write.side_effect = OSError('encoder failure')
        self.store.tick()
        writers[0].release.assert_called_once()
        writers[1].release.assert_not_called()
        status = self.store.health()['cameras']['1']
        self.assertEqual(set(status['recordings']), {'inspection'})
        self.assertEqual(set(status['recording_errors']), {'odometry'})
        with self.assertRaisesRegex(ValueError, 'encoder failure'):
            self.store.stop(1, 'odometry')
        self.store.start(1, 'odometry')
        self.assertFalse(self.store.health()['cameras']['1']['recording_errors'])

    def test_camera_stall_finalizes_all_names_and_retains_each_error(self):
        writers = [Mock(), Mock()]
        self.cv.VideoWriter.side_effect = writers
        self.store.start(1, 'odometry')
        self.store.start(1, 'inspection')
        self.src.last_frame_t -= 10
        self.store.tick()
        for writer in writers:
            writer.release.assert_called_once()
        status = self.store.health()['cameras']['1']
        self.assertFalse(status['recording'])
        self.assertEqual(set(status['recording_errors']), {'odometry', 'inspection'})

    def test_named_paths_and_invalid_names(self):
        self.store.image(1, 'mission-1.snapshot')
        self.store.start(1, 'module_2')
        self.assertTrue(Path(self.cv.imwrite.call_args.args[0]).name.startswith('image_mission-1.snapshot_'))
        self.assertTrue(Path(self.cv.VideoWriter.call_args.args[0]).name.startswith('video_module_2_'))
        self.cv.reset_mock()
        for name in ('', '../escape', '/absolute', 'with/slash', 'with\\slash', 'two words',
                     'a' * 65, None, 42, 'name\n'):
            for action in (self.store.image, self.store.start, self.store.stop):
                with self.subTest(name=name, action=action.__name__):
                    with self.assertRaises(api.InvalidCaptureRequest):
                        action(1, name)
        self.cv.imwrite.assert_not_called()
        self.cv.VideoWriter.assert_not_called()
        self.assertIn((1, 'module_2'), self.store.recordings)

    def test_unknown_camera_slots_have_no_side_effects(self):
        for action in (self.store.image, self.store.start, self.store.stop):
            with self.assertRaises(api.CameraUnavailable):
                action(3, 'mission')
        self.cv.imwrite.assert_not_called()
        self.cv.VideoWriter.assert_not_called()

    def test_health_ignores_intentionally_absent_second_belly_camera(self):
        self.assertTrue(self.store.health()['healthy'])
        self.store.cameras[2] = ('right', None, 'right')
        self.assertFalse(self.store.health()['healthy'])

    def test_invalid_configuration(self):
        for fps in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                api.CaptureStore({}, self.temp.name, fps=fps)
        with self.assertRaises(ValueError):
            api.CaptureStore({}, self.temp.name, jpeg_quality=101)


class AdapterTests(unittest.TestCase):
    def build(self, mode='auto', stereo_enabled=False, stereo=None, belly=None):
        config = SimpleNamespace(CAPTURE_API_MODE=mode, STEREO_CAMERA_ENABLED=stereo_enabled,
                                 CAPTURE_DIRECTORY='/tmp/camera-api-test-unused',
                                 CAPTURE_VIDEO_FPS=25, CAPTURE_JPEG_QUALITY=90,
                                 HTTP_API_ENABLED=False,
                                 CAPTURE_API_NAMESPACE='/camera/capture')
        node = Mock()
        messages = SimpleNamespace(String=lambda **kw: SimpleNamespace(**kw))
        services = SimpleNamespace(Capture=object())
        with patch.dict(sys.modules, {'std_msgs.msg': messages,
                                      'camera_capture_interfaces.srv': services}):
            adapter = api.CameraCaptureAPI(node, stereo, belly, config)
        callbacks = {call.args[1].rsplit('/', 1)[1]: call.args[2]
                     for call in node.create_service.call_args_list}
        self.assertTrue(all(call.args[0] is services.Capture
                            for call in node.create_service.call_args_list))
        self.addCleanup(adapter.close)
        return adapter, node, callbacks

    def test_seven_name_only_services_and_belly_mapping(self):
        belly = source()
        adapter, node, callbacks = self.build(belly=belly)
        self.assertEqual(set(callbacks), {'health', 'image1', 'image2', 'video1_start',
                                          'video1_stop', 'video2_start', 'video2_stop'})
        self.assertEqual(set(adapter.actions), set(callbacks))
        self.assertIs(adapter.store.cameras[1][1], belly)
        for action in ('image2', 'video2_start', 'video2_stop'):
            response = callbacks[action](SimpleNamespace(name='default'), SimpleNamespace())
            self.assertFalse(response.success)
            self.assertIn('No camera', response.message)
        self.assertEqual(node.get_logger().warning.call_count, 3)

    def test_health_accepts_name_and_publishes_json(self):
        adapter, node, callbacks = self.build(belly=source())
        response = callbacks['health'](SimpleNamespace(name='default'), SimpleNamespace())
        self.assertTrue(response.success)
        payload = node.create_publisher.return_value.publish.call_args.args[0].data
        self.assertEqual(payload, response.message)
        self.assertTrue(json.loads(payload)['healthy'])
        self.assertFalse(adapter.store.recordings)

    def test_auto_stereo_takes_precedence_and_preserves_missing_left(self):
        stereo = source()
        stereo.has_left = False
        adapter, _, _ = self.build(stereo_enabled=True, stereo=stereo, belly=source())
        self.assertEqual(adapter.store.cameras[1], ('left', None, 'left'))
        self.assertEqual(adapter.store.cameras[2], ('right', stereo, 'right'))

    def test_explicit_belly_mode_with_both_sources(self):
        belly = source()
        adapter, _, _ = self.build(mode='belly', stereo_enabled=True, stereo=source(), belly=belly)
        self.assertEqual(adapter.store.cameras[1], ('belly', belly, 'frame'))
        self.assertIsNone(adapter.store.cameras[2][1])

    def test_endpoint_selects_camera_and_name_can_be_reused_on_both_slots(self):
        adapter, _, callbacks = self.build(mode='stereo', stereo=source())
        with tempfile.TemporaryDirectory() as directory, patch.object(api, 'cv2', Mock()) as cv:
            adapter.store.directory = Path(directory)
            writers = [Mock(), Mock()]
            cv.VideoWriter.side_effect = writers
            request = SimpleNamespace(name='mission')
            for number in (1, 2):
                response = callbacks[f'video{number}_start'](request, SimpleNamespace())
                self.assertTrue(response.success, response.message)
            self.assertEqual(set(adapter.store.recordings), {(1, 'mission'), (2, 'mission')})
            stopped = callbacks['video1_stop'](request, SimpleNamespace())
            self.assertTrue(stopped.success)
            writers[0].release.assert_called_once()
            writers[1].release.assert_not_called()
            self.assertEqual(set(adapter.store.health()['cameras']['2']['recordings']), {'mission'})
            self.assertTrue(callbacks['video2_stop'](request, SimpleNamespace()).success)

    def test_invalid_name_is_reported_as_validation_error(self):
        adapter, _, _ = self.build(belly=source())
        result, status = adapter.execute('video1_start', name='../escape')
        self.assertEqual(status, 422)
        self.assertFalse(result['success'])

    def test_ros_services_isolate_modules_and_share_http_recording_state(self):
        adapter, _, callbacks = self.build(belly=source())
        with tempfile.TemporaryDirectory() as directory, patch.object(api, 'cv2', Mock()) as cv:
            adapter.store.directory = Path(directory)
            writers = [Mock(), Mock()]
            cv.VideoWriter.side_effect = writers
            for name in ('odometry', 'inspection'):
                response = callbacks['video1_start'](SimpleNamespace(name=name), SimpleNamespace())
                self.assertTrue(response.success, response.message)
            duplicate = callbacks['video1_start'](SimpleNamespace(name='odometry'), SimpleNamespace())
            self.assertFalse(duplicate.success)
            for name in ('default', 'unknown'):
                stopped = callbacks['video1_stop'](SimpleNamespace(name=name), SimpleNamespace())
                self.assertFalse(stopped.success)
            stopped = callbacks['video1_stop'](SimpleNamespace(name='odometry'), SimpleNamespace())
            self.assertTrue(stopped.success)
            writers[0].release.assert_called_once()
            writers[1].release.assert_not_called()
            result, status = adapter.execute('video1_stop', name='inspection')
            self.assertEqual(status, 200, result)
            writers[1].release.assert_called_once()

    def test_ros_services_validate_names_before_writes(self):
        adapter, _, callbacks = self.build(belly=source())
        with patch.object(api, 'cv2', Mock()) as cv:
            for action in callbacks:
                for name in ('', '../escape'):
                    with self.subTest(action=action, name=name):
                        result = callbacks[action](SimpleNamespace(name=name), SimpleNamespace())
                        self.assertFalse(result.success)
            cv.imwrite.assert_not_called()
            cv.VideoWriter.assert_not_called()

    def test_ros_snapshot_uses_endpoint_camera_and_prefix(self):
        adapter, _, callbacks = self.build(mode='stereo', stereo=source(), belly=source())
        with tempfile.TemporaryDirectory() as directory, patch.object(api, 'cv2', Mock()) as cv:
            adapter.store.directory = Path(directory)
            for number, label in ((1, 'left'), (2, 'right')):
                response = callbacks[f'image{number}'](SimpleNamespace(name='inspection'), SimpleNamespace())
                self.assertTrue(response.success, response.message)
                path = Path(cv.imwrite.call_args.args[0])
                self.assertEqual(path.parent.name, label)
                self.assertTrue(path.name.startswith('image_inspection_'))


if __name__ == '__main__':
    unittest.main()

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
        self.assertIn(2, self.store.recordings)
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
        self.assertNotIn(1, self.store.errors)

    def test_resolution_change_finalizes_video(self):
        self.store.start(1)
        self.src.frame.shape = (80, 128, 3)
        self.store.tick()
        self.assertFalse(self.store.recordings)
        self.assertIn('resolution changed', self.store.errors[1])

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
        services = SimpleNamespace(Trigger=object())
        with patch.dict(sys.modules, {'std_msgs.msg': messages, 'std_srvs.srv': services}):
            adapter = api.CameraCaptureAPI(node, stereo, belly, config)
        callbacks = {call.args[1].rsplit('/', 1)[1]: call.args[2]
                     for call in node.create_service.call_args_list}
        self.addCleanup(adapter.close)
        return adapter, node, callbacks

    def test_seven_services_and_belly_mapping(self):
        belly = source()
        adapter, node, callbacks = self.build(belly=belly)
        self.assertEqual(set(callbacks), {'health', 'image1', 'image2', 'video1_start',
                                          'video1_stop', 'video2_start', 'video2_stop'})
        self.assertIs(adapter.store.cameras[1][1], belly)
        for name in ('image2', 'video2_start', 'video2_stop'):
            response = callbacks[name](None, SimpleNamespace())
            self.assertFalse(response.success)
            self.assertIn('No camera', response.message)
        self.assertEqual(node.get_logger().warning.call_count, 3)

    def test_health_publishes_json(self):
        adapter, node, callbacks = self.build(belly=source())
        response = callbacks['health'](None, SimpleNamespace())
        self.assertTrue(response.success)
        payload = node.create_publisher.return_value.publish.call_args.args[0].data
        self.assertEqual(payload, response.message)
        self.assertTrue(json.loads(payload)['healthy'])

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


if __name__ == '__main__':
    unittest.main()

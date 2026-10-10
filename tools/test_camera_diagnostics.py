#!/usr/bin/env python3
"""Exercise actual detection filtering and camera-only process ownership offline."""
import asyncio
import importlib.util
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import Mock, patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/camera'))


def load_detector():
    modules = {}
    for name in ('cv2', 'cv_bridge', 'rclpy', 'sensor_msgs', 'sensor_msgs.msg',
                 'geometry_msgs', 'geometry_msgs.msg', 'ament_index_python',
                 'ament_index_python.packages'):
        modules[name] = types.ModuleType(name)
    modules['cv_bridge'].CvBridge = Mock()
    modules['cv2'].imwrite = Mock(return_value=True)
    modules['cv2'].rectangle = Mock()
    modules['cv2'].putText = Mock()
    modules['cv2'].FONT_HERSHEY_SIMPLEX = 0
    modules['sensor_msgs.msg'].Image = Mock()
    modules['geometry_msgs.msg'].Point = types.SimpleNamespace
    modules['ament_index_python.packages'].get_package_share_directory = Mock()
    with patch.dict(sys.modules, modules):
        from camera.detection_utils import DetectionHandler
        return DetectionHandler


class CameraTests(unittest.TestCase):
    def detector(self, raw):
        detector_type = load_detector()
        handler = detector_type.__new__(detector_type)
        handler._frame_lock = threading.Lock()
        handler.current_frame = np.zeros((20, 20, 3), dtype=np.uint8)
        handler.current_depth = np.ones((20, 20), dtype=np.float32)
        handler.depth_encoding = '32FC1'
        handler.inference = Mock()
        handler.inference.predict.return_value = raw
        handler.tf_handler = Mock()
        handler.tf_handler.pixel_to_3d.return_value = (0., 0., 1.)
        handler.tf_handler.transform_to_base.return_value = types.SimpleNamespace(x=0.2, y=0.3, z=0.4)
        handler.visualiser = Mock()
        handler.node = Mock()
        return handler

    def request(self, handler, command='detect'):
        return asyncio.run(handler.handle_request(types.SimpleNamespace(
            command=command, identifier=0, conf=0.3)))

    def test_distinguishes_model_filter_depth_and_intrinsics(self):
        box = dict(box=[5, 5, 15, 15], class_id=0, confidence=0.8)
        detector = self.detector([])
        self.assertIn('YOLO原始框=0', self.request(detector)['message'])
        detector = self.detector([dict(box, confidence=0.2), dict(box, class_id=1)])
        self.assertIn('类别/阈值通过=0', self.request(detector)['message'])
        detector = self.detector([box])
        detector.current_depth[:] = 0
        self.assertIn('无效深度=1', self.request(detector)['message'])
        detector = self.detector([box])
        detector.tf_handler.pixel_to_3d.return_value = None
        self.assertIn('缺少投影/内参=1', self.request(detector)['message'])
        detector = self.detector([box])
        response = self.request(detector)
        self.assertEqual(len(response['coordinates']), 1)
        self.assertAlmostEqual(response['coordinates'][0].x, -0.2)
        self.assertAlmostEqual(response['coordinates'][0].y, -0.3)

    def test_missing_frames_and_worker_failure_are_not_success(self):
        detector = self.detector([])
        detector.current_frame = None
        self.assertFalse(self.request(detector)['success'])
        detector = self.detector([])
        detector.inference.predict.side_effect = RuntimeError('worker failed')
        self.assertFalse(self.request(detector)['success'])

    def test_debug_saves_exact_input_even_with_no_detections(self):
        detector = self.detector([])
        with tempfile.TemporaryDirectory() as directory, patch.dict('os.environ', HARVEST_DIAGNOSTIC_DIR=directory):
            with patch.object(type(detector)._detect_objects.__globals__['cv2'], 'imwrite', return_value=True) as save:
                response = self.request(detector, 'detect_debug')
                self.assertTrue(response['success'])
                self.assertEqual(save.call_count, 2)
                np.testing.assert_array_equal(save.call_args_list[0].args[1], detector.current_frame)
                self.assertIn(directory, response['message'])

    def entry(self):
        spec = importlib.util.spec_from_file_location('camera_check_test', ROOT / 'tools/camera_check.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_single_terminal_launches_only_camera_and_cleans_owned_group(self):
        entry = self.entry()
        rclpy = types.ModuleType('rclpy')
        rclpy.init = Mock()
        rclpy.shutdown = Mock()
        rclpy.ok = Mock(return_value=True)
        rclpy.spin_until_future_complete = Mock()
        node, client, future = Mock(), Mock(), Mock()
        node.create_client.return_value = client
        client.wait_for_service.return_value = True
        client.call_async.return_value = future
        future.done.return_value = True
        future.result.return_value = types.SimpleNamespace(success=True, message='Found 0 objects', coordinates=[])
        rclpy.create_node = Mock(return_value=node)
        srv = types.ModuleType('custom_interface.srv')
        srv.CameraSrv = types.SimpleNamespace(Request=types.SimpleNamespace)
        process = Mock(pid=12345)
        with tempfile.TemporaryDirectory() as directory, patch.object(entry, 'ROOT', Path(directory)), \
                patch.dict(sys.modules, {'rclpy': rclpy, 'custom_interface.srv': srv}), \
                patch.object(sys, 'argv', ['camera-check']), \
                patch.object(entry.subprocess, 'check_output', return_value=''), \
                patch.object(entry.subprocess, 'Popen', return_value=process) as launch, \
                patch.object(entry.os, 'killpg', create=True) as stop:
            entry.main()
            self.assertEqual(launch.call_args.args[0], ['bash', 'scripts/project', 'camera'])
            self.assertTrue(launch.call_args.kwargs['start_new_session'])
            self.assertEqual(client.call_async.call_args.args[0].command, 'detect_debug')
            stop.assert_called_once_with(12345, entry.signal.SIGINT)
            self.assertEqual(len(list(Path(directory).glob('log-project/*/result.json'))), 1)

    def test_existing_camera_is_not_started_or_stopped(self):
        entry = self.entry()
        srv = types.ModuleType('custom_interface.srv')
        srv.CameraSrv = Mock()
        with patch.dict(sys.modules, {'rclpy': types.ModuleType('rclpy'), 'custom_interface.srv': srv}), \
                patch.object(sys, 'argv', ['camera-check']), \
                patch.object(entry.subprocess, 'check_output', return_value='/camera_server\n'), \
                patch.object(entry.subprocess, 'Popen') as launch:
            with self.assertRaises(RuntimeError): entry.main()
            launch.assert_not_called()


if __name__ == '__main__':
    unittest.main()

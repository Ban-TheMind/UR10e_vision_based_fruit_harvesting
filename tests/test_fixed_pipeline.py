"""Run the Humble detection callback with synthetic images and model output."""
import asyncio, threading, unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from camera.detection_utils import DetectionHandler
from camera.tf_utils import TFHandler
from camera.geometry import CAMERA_TO_BASE

class FixedPipelineTests(unittest.TestCase):
    def test_request_coordinates_are_base_link_for_both_encodings(self):
        info=SimpleNamespace(width=640,height=480,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],d=[0.]*5,distortion_model='plumb_bob')
        tf=TFHandler.__new__(TFHandler);tf.camera_info=info;tf.calibration_matrix=CAMERA_TO_BASE
        for command in ('detect', 'detect_flip'):
          for encoding,frame in [('16UC1',np.full((480,640),1000,np.uint16)),('32FC1',np.ones((480,640),np.float32))]:
            detector=DetectionHandler.__new__(DetectionHandler)
            detector.node=Mock();detector.frame_lock=threading.Lock();detector.tf_handler=tf
            detector.current_frame=np.zeros((480,640,3),np.uint8)
            detector.current_depth=frame;detector.depth_encoding=encoding
            detector.visualiser=Mock()
            boxes=SimpleNamespace(xyxy=Mock(),cls=Mock(),conf=Mock())
            for name,value in [('xyxy',[[310.,229.,330.,249.]] if command == 'detect_flip' else [[310.,230.,330.,250.]]),('cls',[0]),('conf',[.9])]:
                getattr(boxes,name).cpu.return_value.numpy.return_value=np.asarray(value)
            detector.model=Mock(return_value=[SimpleNamespace(boxes=boxes)])
            result=asyncio.run(detector._detect_objects(SimpleNamespace(command=command,identifier=0,conf=.5)))
            self.assertTrue(result['success'])
            point=result['coordinates'][0]
            np.testing.assert_allclose((point.x,point.y,point.z),(-.82819,-.26815,.11578),atol=1e-9)
            np.testing.assert_allclose(detector.last_detections[0]['planning_point'],(point.x,point.y,point.z),atol=1e-9)

if __name__=='__main__':unittest.main()

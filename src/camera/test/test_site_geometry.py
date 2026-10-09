"""Shared deployment geometry checks; no ROS graph or hardware is used."""
import sys, math, json, tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera.geometry import CAMERA_TO_BASE, CALIBRATION_FILE, camera_point_to_base, controller_base_to_base_link, deproject_pixel, load_calibration, unflip_vertical_pixel
from camera.depth_utils import median_depth_m

class SiteGeometryTests(unittest.TestCase):
    def test_known_optical_axis_and_planning_frame(self):
        base=camera_point_to_base(0.,0.,1.)
        np.testing.assert_allclose(base,(.82819,.26815,.11578),atol=1e-9)
        np.testing.assert_allclose(controller_base_to_base_link(*base),(-.82819,-.26815,.11578),atol=1e-9)
    def test_encoding_changes_do_not_change_units(self):
        self.assertEqual(median_depth_m(np.full((11,11),1000,np.uint16),'16UC1',5,5),1.)
        self.assertEqual(median_depth_m(np.ones((11,11),np.float32),'32FC1',5,5),1.)
        with self.assertRaises(ValueError): median_depth_m(np.ones((11,11)),'mono16',5,5)
    def test_pixel_order_and_flip(self):
        info=SimpleNamespace(width=640,height=480,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],d=[0.]*5,distortion_model='plumb_bob')
        self.assertEqual(deproject_pixel(370,340,1.,info),(.1,.2,1.))
        self.assertEqual(unflip_vertical_pixel(0,480),479)
        self.assertIsNone(deproject_pixel(370,340,math.nan,info))
    def test_wrong_frame_units_and_rotation_rejected(self):
        for key,value in [('target_frame','base_link'),('units','mm'),('source_frame','camera_link')]:
            data=json.loads(CALIBRATION_FILE.read_text()); data[key]=value
            with tempfile.TemporaryDirectory() as folder:
                path=Path(folder)/'calibration.json'; path.write_text(json.dumps(data))
                with self.assertRaises(ValueError): load_calibration(path)
        data=json.loads(CALIBRATION_FILE.read_text());data['matrix'][0][0]=3
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'calibration.json';path.write_text(json.dumps(data))
            with self.assertRaises(ValueError):load_calibration(path)

if __name__=='__main__':unittest.main()

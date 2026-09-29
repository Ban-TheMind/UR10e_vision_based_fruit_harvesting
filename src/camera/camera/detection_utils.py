import cv2
from .inference_client import InferenceClient
from .depth_utils import median_depth_m
from .geometry import controller_base_to_base_link, unflip_vertical_pixel
import numpy as np
import asyncio
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import os
import threading

class DetectionHandler:
    def __init__(self, node, tf_handler, visualiser):
        self.node = node
        self.tf_handler = tf_handler
        self.visualiser = visualiser

        self.bridge = CvBridge()

        camera_pkg_dir = get_package_share_directory('camera')
        # model_path = os.path.join(camera_pkg_dir, 'models', 'yolo11m.pt')
        model_path = os.path.join(camera_pkg_dir, 'models', 'best.pt')
        with open(model_path, 'rb') as model_file:
            if model_file.read(32).startswith(b'version https://git-lfs'):
                raise RuntimeError('best.pt is a Git LFS pointer; fetch the model weights')
        node.declare_parameter(
            'inference_python', '/home/ubuntu/miniconda3/envs/VPP/bin/python')
        inference_python = node.get_parameter('inference_python').value
        self.inference = InferenceClient(inference_python, model_path)

        self.last_detections = None
        self.rviz_vis_timer = self.node.create_timer(
            0.1, 
            lambda: self.visualiser.update_rviz_visualization(self.last_detections)
        )
        # Current frame data
        self.current_frame = None
        self.current_depth = None
        self.depth_encoding = None
        self._frame_lock = threading.Lock()
        
    async def handle_request(self, request):
        if request.command.startswith("detect"):
            return await self._detect_objects(request)
        else:
            return {'success': False, 'message': f"Unknown command: {request.command}"}
    
    async def _detect_objects(self, request):

        with self._frame_lock:
            saved_frame = None if self.current_frame is None else self.current_frame.copy()
            depth_frame = self.current_depth
            depth_encoding = self.depth_encoding

        if saved_frame is not None and request.command == 'detect_flip':
            saved_frame = cv2.flip(saved_frame, 0)

        """Async handler for detect command"""
        if saved_frame is None or depth_frame is None:
            return {
                'success': False,
                'message': "No frame available"
            }
            
        try:
            results = self.inference.predict(saved_frame)
            
            detections = []
            self.last_detections = []

            for detection in results:
                box = detection['box']
                cls_id = detection['class_id']
                conf = detection['confidence']
                if cls_id == request.identifier and conf > request.conf:
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)
                    # Detection may use a vertically flipped color image, while
                    # aligned depth and camera intrinsics remain unflipped.
                    camera_y = (unflip_vertical_pixel(y_center, saved_frame.shape[0])
                                if request.command == 'detect_flip' else y_center)
                    avg_depth = self.get_average_depth(
                        x_center, camera_y, depth_frame, depth_encoding)

                    # if invalid 
                    if np.isnan(avg_depth):
                        print(f"INVALID DEPTH!!!! SKIPPING!!!!")
                        continue
                    
                    point_3d = self.tf_handler.pixel_to_3d(x_center, camera_y, avg_depth)

                    if point_3d is None:
                        continue

                    point_msg = Point()
                    point_msg.x = point_3d[0]
                    point_msg.y = point_3d[1]
                    point_msg.z = point_3d[2]

                    base_point = self.tf_handler.transform_to_base(point_msg)
                    # UR controller `base` is rotated by 180 degrees about Z
                    # relative to MoveIt's `base_link` planning frame.
                    planning_point = Point()
                    planning_point.x, planning_point.y, planning_point.z = (
                        controller_base_to_base_link(
                            base_point.x, base_point.y, base_point.z))
                    detections.append(planning_point)
                    self.last_detections.append({
                        'box': box,
                        'center': (x_center, y_center),
                        'base_point': base_point,
                        'confidence': conf,
                    })

                    self.node.get_logger().info(
                        f"Transformed coordinates (UR base frame): "
                        f"X: {base_point.x:.3f}, "
                        f"Y: {base_point.y:.3f}, "
                        f"Z: {base_point.z:.3f}")
            
            self.visualiser.update_cv_visualization(saved_frame, self.last_detections)

            return {
                'coordinates': detections,
                'success': True,
                'message': f"Found {len(detections)} objects"
            }
            
        except Exception as e:
            self.node.get_logger().error(f"Detection error: {str(e)}")
            return {
                'success': False,
                'message': f"Detection failed: {str(e)}"
            }
    
    def update_frames(self, color_msg, depth_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if color.shape[:2] != depth.shape[:2]:
                raise ValueError("Color and aligned depth dimensions differ")
            with self._frame_lock:
                self.current_frame = color
                self.current_depth = depth
                self.depth_encoding = depth_msg.encoding
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {str(e)}")

    def get_average_depth(self, x_center, y_center, depth_frame, depth_encoding,
                          sampling_radius=5):
        """Return median aligned depth in meters from a local pixel window."""
        try:
            return median_depth_m(depth_frame, depth_encoding, x_center, y_center,
                                  sampling_radius)
        except ValueError as exc:
            self.node.get_logger().error(str(exc))
            return float('nan')

    def close(self):
        self.inference.close()

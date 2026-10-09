import cv2
import threading
from .depth_utils import median_depth_m
from .geometry import controller_base_to_base_link
from ultralytics import YOLO
import numpy as np
import asyncio
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, TransformStamped
from ament_index_python.packages import get_package_share_directory
import os
import tf2_ros

class DetectionHandler:
    def __init__(self, node, tf_handler, visualiser):
        self.node = node
        self.tf_handler = tf_handler
        self.visualiser = visualiser

        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()

        camera_pkg_dir = get_package_share_directory('camera')
        # model_path = os.path.join(camera_pkg_dir, 'models', 'yolo11m.pt')
        model_path = self.node.get_parameter('model_file').value
        if not os.path.isabs(model_path):
            model_path = os.path.join(camera_pkg_dir, 'models', model_path)
        if not os.path.isfile(model_path):
            raise FileNotFoundError(f'Model not found: {model_path}')
        # Load YOLO model from parameter
        self.model = YOLO(model_path)
        self.model.fuse()

        self.last_detections = None
        self.rviz_vis_timer = self.node.create_timer(
            0.1, 
            lambda: self.visualiser.update_rviz_visualization(self.last_detections)
        )
        # Current frame data
        self.current_frame = None
        self.current_depth = None
        self.depth_encoding = None
        
    async def handle_request(self, request):
        if request.command.startswith("detect"):
            return await self._detect_objects(request)
        else:
            return {'success': False, 'message': f"Unknown command: {request.command}"}
    
    async def _detect_objects(self, request):

        with self.frame_lock:
            saved_frame = None if self.current_frame is None else self.current_frame.copy()
            saved_depth = None if self.current_depth is None else self.current_depth.copy()
            depth_encoding = self.depth_encoding

        if request.command == 'detect_flip' and saved_frame is not None:
            saved_frame = cv2.flip(saved_frame, 0)

        """Async handler for detect command"""
        if saved_frame is None or saved_depth is None or self.tf_handler.camera_info is None:
            return {
                'success': False,
                'message': "No frame available"
            }
            
        try:
            results = self.model(saved_frame, verbose=False)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            detections = []
            self.last_detections = []

            for i, (box, cls_id, conf) in enumerate(zip(boxes, class_ids, confidences)):
                if cls_id == request.identifier and conf > request.conf:
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)
                    depth_y = saved_frame.shape[0] - 1 - y_center if request.command == 'detect_flip' else y_center
                    avg_depth = self.get_average_depth(x_center, depth_y, depth=saved_depth, encoding=depth_encoding)

                    # if invalid 
                    if np.isnan(avg_depth):
                        print(f"INVALID DEPTH!!!! SKIPPING!!!!")
                        continue
                    
                    point_3d = self.tf_handler.pixel_to_3d(x_center, depth_y, avg_depth)

                    if not point_3d:
                        continue

                    # Prepare for visualization (camera frame coordinates)
                    vis_data = {
                        'box': box,
                        'center': (x_center, y_center),
                        'point_3d': point_3d,  # Camera frame coordinates
                        'confidence': conf
                    }

                    point_msg = Point()
                    point_msg.x = point_3d[0]
                    point_msg.y = point_3d[1]
                    point_msg.z = point_3d[2]
                    
                    try:
                        # Transform the point to base frame
                        base_pose = self.tf_handler.transform_to_base(point_msg)
                        
                        if base_pose is None:
                            return {'success': False, 'message': 'Target-frame transform unavailable'}
                        # Create new point with transformed coordinates
                        transformed_point = Point()

                        # the minus sign converts to actual coordinates wrt. base_link
                        transformed_point.x, transformed_point.y, transformed_point.z = controller_base_to_base_link(
                            base_pose.x, base_pose.y, base_pose.z)
                        vis_data['planning_point'] = (transformed_point.x, transformed_point.y, transformed_point.z)
                        
                        detections.append(transformed_point)
                        self.last_detections.append(vis_data)
                        
                        # Print the transformed coordinates
                        self.node.get_logger().info(
                            f"Transformed coordinates (base frame): "
                            f"X: {transformed_point.x:.3f}, "
                            f"Y: {transformed_point.y:.3f}, "
                            f"Z: {transformed_point.z:.3f}")
                            
                    except (tf2_ros.LookupException, 
                            tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        self.node.get_logger().error(f"TF transform failed: {str(e)}")
            
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
            frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if frame.shape[:2] != depth.shape[:2]:
                raise ValueError('Color and aligned depth dimensions differ')
            with self.frame_lock:
                self.current_frame, self.current_depth = frame, depth
                self.depth_encoding = depth_msg.encoding
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {str(e)}")

    def get_average_depth(self, x_center, y_center, sampling_radius=5, depth=None, encoding=None):
        depth = self.current_depth if depth is None else depth
        encoding = self.depth_encoding if encoding is None else encoding
        return median_depth_m(depth, encoding, x_center, y_center, sampling_radius)

import cv2
import threading
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
        
    async def handle_request(self, request):
        if request.command.startswith("detect"):
            return await self._detect_objects(request)
        else:
            return {'success': False, 'message': f"Unknown command: {request.command}"}
    
    async def _detect_objects(self, request):

        with self.frame_lock:
            saved_frame = None if self.current_frame is None else self.current_frame.copy()
            saved_depth = None if self.current_depth is None else self.current_depth.copy()

        if request.command == 'detect_flip' and saved_frame is not None:
            saved_frame = cv2.flip(saved_frame, 0)

        """Async handler for detect command"""
        if saved_frame is None or saved_depth is None or self.tf_handler.intrinsics is None:
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
                    avg_depth = self.get_average_depth(x_center, depth_y, depth=saved_depth)

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
                        signs = self.node.get_parameter('legacy_base_sign').value if self.node.get_parameter('coordinate_mode').value == 'legacy' else [1.0, 1.0, 1.0]
                        transformed_point.x = signs[0] * base_pose.x
                        transformed_point.y = signs[1] * base_pose.y
                        transformed_point.z = signs[2] * base_pose.z
                        
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
            with self.frame_lock:
                self.current_frame, self.current_depth = frame, depth
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {str(e)}")

    def get_average_depth(self, x_center, y_center, sampling_radius=5, depth=None):
        """
        Compute average depth in a small fixed window around center.
        
        Args:
            x_center, y_center (int): Center coordinates
            sampling_radius (int): How many pixels to sample around center (default=2 → 5×5 window)
        
        Returns:
            float: Robust average depth
        """
        # Extract fixed-size patch
        depth = self.current_depth if depth is None else depth
        depth_patch = depth[
            max(0, y_center - sampling_radius):min(depth.shape[0], y_center + sampling_radius + 1),
            max(0, x_center - sampling_radius):min(depth.shape[1], x_center + sampling_radius + 1)
        ]
        
        # Process valid depths
        valid_depths = depth_patch[(depth_patch > 0) & ~np.isnan(depth_patch)]
        if len(valid_depths) < 3:
            return float('nan')
        
        return float(np.median(valid_depths))  # Median is more robust than mean
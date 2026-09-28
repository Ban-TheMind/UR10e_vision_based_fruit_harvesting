import cv2
from ultralytics import YOLO
import numpy as np
import asyncio
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory
import os

class DetectionHandler:
    def __init__(self, node, tf_handler, visualiser):
        self.node = node
        self.tf_handler = tf_handler
        self.visualiser = visualiser

        self.bridge = CvBridge()

        camera_pkg_dir = get_package_share_directory('camera')
        # model_path = os.path.join(camera_pkg_dir, 'models', 'yolo11m.pt')
        model_path = os.path.join(camera_pkg_dir, 'models', 'best.pt')
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

        saved_frame = self.current_frame

        if saved_frame is not None and request.command == 'detect_flip':
            saved_frame = cv2.flip(saved_frame, 0)

        """Async handler for detect command"""
        if saved_frame is None or self.current_depth is None:
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
                    # Detection may use a vertically flipped color image, while
                    # aligned depth and camera intrinsics remain unflipped.
                    camera_y = (saved_frame.shape[0] - 1 - y_center
                                if request.command == 'detect_flip' else y_center)
                    avg_depth = self.get_average_depth(x_center, camera_y)

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
                    planning_point.x = -base_point.x
                    planning_point.y = -base_point.y
                    planning_point.z = base_point.z
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
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {str(e)}")

    def get_average_depth(self, x_center, y_center, sampling_radius=5):
        """
        Compute average depth in a small fixed window around center.
        
        Args:
            x_center, y_center (int): Center coordinates
            sampling_radius (int): How many pixels to sample around center (default=2 → 5×5 window)
        
        Returns:
            float: Robust average depth
        """
        # Extract fixed-size patch
        depth_patch = self.current_depth[
            max(0, y_center - sampling_radius):min(self.current_depth.shape[0], y_center + sampling_radius + 1),
            max(0, x_center - sampling_radius):min(self.current_depth.shape[1], x_center + sampling_radius + 1)
        ]
        
        # Process valid depths
        valid_depths = depth_patch[(depth_patch > 0) & ~np.isnan(depth_patch)]
        if len(valid_depths) < 3:
            return float('nan')
        
        return float(np.median(valid_depths))  # Median is more robust than mean

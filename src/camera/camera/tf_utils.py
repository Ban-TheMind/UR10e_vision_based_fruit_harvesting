import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


# Eye-to-hand calibration: aligned color optical camera frame -> UR base frame.
# Translation is in meters; pixel deprojection below produces camera coordinates in meters.
CAMERA_TO_BASE = (
    (-0.14168, -0.28354,  0.94844, -0.12025),
    (-0.98968,  0.019898, -0.14189,  0.41004),
    ( 0.021358, -0.95875, -0.28343,  0.39921),
    ( 0.0,       0.0,       0.0,      1.0),
)

class TFHandler:
    def __init__(self, node):
        self.node = node
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.camera_info = None
        
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
        )

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if msg.width <= 0 or msg.height <= 0 or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.node.get_logger().error("Invalid aligned depth camera intrinsics")
            return
        if msg.distortion_model not in ("plumb_bob", "rational_polynomial", "none", ""):
            self.node.get_logger().error(
                f"Unsupported camera distortion model: {msg.distortion_model}")
            return
        if self.camera_info is None:
            self.node.get_logger().info("Aligned depth camera intrinsics received")
        self.camera_info = msg

    def transform_to_base(self, point):
        """Convert an optical-frame camera point to the UR base frame."""
        coordinates = (point.x, point.y, point.z)
        result = Point()
        result.x = sum(CAMERA_TO_BASE[0][i] * coordinates[i] for i in range(3)) + CAMERA_TO_BASE[0][3]
        result.y = sum(CAMERA_TO_BASE[1][i] * coordinates[i] for i in range(3)) + CAMERA_TO_BASE[1][3]
        result.z = sum(CAMERA_TO_BASE[2][i] * coordinates[i] for i in range(3)) + CAMERA_TO_BASE[2][3]
        return result
            
    def publish_transform(self, frame_id, child_frame_id, point, orientation=None):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = point[2]
        
        if orientation is None:
            t.transform.rotation.w = 1.0
        else:
            t.transform.rotation = orientation
            
        self.broadcaster.sendTransform(t)
        
    def pixel_to_3d(self, pixel_x, pixel_y, depth_m):
        """Deproject aligned depth in meters to the color optical frame."""
        info = self.camera_info
        if info is None or not np.isfinite(depth_m) or depth_m <= 0:
            return None
        if not (0 <= pixel_x < info.width and 0 <= pixel_y < info.height):
            return None
        camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        pixel = np.asarray([[[pixel_x, pixel_y]]], dtype=np.float64)
        if info.distortion_model in ("plumb_bob", "rational_polynomial"):
            distortion = np.asarray(info.d, dtype=np.float64)
            normalized = cv2.undistortPoints(pixel, camera_matrix, distortion)[0, 0]
        else:
            normalized = ((pixel_x - info.k[2]) / info.k[0],
                          (pixel_y - info.k[5]) / info.k[4])
        return [float(normalized[0] * depth_m),
                float(normalized[1] * depth_m), float(depth_m)]

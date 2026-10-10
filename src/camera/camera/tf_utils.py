import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo
from .geometry import CAMERA_TO_BASE, camera_point_to_base, deproject_pixel, load_calibration


class TFHandler:
    def __init__(self, node):
        self.node = node
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.camera_info = None
        for name, default in (('camera_info_topic', '/camera/aligned_depth_to_color/camera_info'),
                              ('calibration_file', '')):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
        self.calibration_matrix = load_calibration(node.get_parameter('calibration_file').value or None)
        
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            node.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if msg.width <= 0 or msg.height <= 0 or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.node.get_logger().error("Invalid aligned depth camera intrinsics")
            return
        if (msg.distortion_model not in ("plumb_bob", "rational_polynomial", "none", "")
                and any(abs(v) > 1e-12 for v in msg.d)):
            self.node.get_logger().error(
                f"Unsupported camera distortion model: {msg.distortion_model}")
            return
        if self.camera_info is None:
            self.node.get_logger().info("Aligned depth camera intrinsics received")
        self.camera_info = msg

    def transform_to_base(self, point):
        """Convert an optical-frame camera point to the UR base frame."""
        coordinates = camera_point_to_base(point.x, point.y, point.z, getattr(self, 'calibration_matrix', CAMERA_TO_BASE))
        result = Point()
        result.x, result.y, result.z = coordinates
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
        if self.camera_info is None:
            return None
        return deproject_pixel(pixel_x, pixel_y, depth_m, self.camera_info)

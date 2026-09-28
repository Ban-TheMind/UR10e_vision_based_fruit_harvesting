import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
import tf_transformations
import pyrealsense2 as rs
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
        self.intrinsics = None
        
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
        )

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = list(msg.d)
            self.node.get_logger().info("Camera intrinsics received")

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
            q = tf_transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        else:
            t.transform.rotation = orientation
            
        self.broadcaster.sendTransform(t)
        
    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert an aligned color pixel and depth in mm to the optical camera frame."""
        if self.intrinsics is None:
            return None
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x, pixel_y],
            depth_value * 0.001  # mm to meters
        )
        return point_3d

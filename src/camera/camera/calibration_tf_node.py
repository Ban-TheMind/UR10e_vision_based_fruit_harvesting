"""Bridge the external RealSense tree using the same calibration as detection."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformException
from .geometry import load_calibration
from .perception_geometry import camera_root_in_base, quaternion_from_matrix, transform_matrix


class CalibrationTF(Node):
    def __init__(self):
        super().__init__('external_camera_calibration_tf')
        for key, default in (('calibration_file', ''), ('camera_root_frame', 'camera_link'),
                             ('calibration_verified', False)):
            self.declare_parameter(key, default)
        self.calibration = load_calibration(self.get_parameter('calibration_file').value or None)
        self.root_frame = self.get_parameter('camera_root_frame').value
        if not self.root_frame or self.root_frame in ('base', 'base_link', 'world', 'camera_color_optical_frame'):
            raise ValueError('camera_root_frame must be the RealSense driver root')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.warned = False
        if not self.get_parameter('calibration_verified').value:
            self.get_logger().warning('Camera calibration is UNVERIFIED: mapping/plan-only commissioning; verify known scene points before physical avoidance.')
        self.timer = self.create_timer(0.5, self.publish_external_transform)

    def publish_external_transform(self):
        try:
            internal = self.buffer.lookup_transform(
                self.root_frame, 'camera_color_optical_frame', rclpy.time.Time())
        except TransformException as error:
            if not self.warned:
                self.get_logger().warning('Waiting for RealSense internal TF: ' + str(error))
                self.warned = True
            return
        t, q = internal.transform.translation, internal.transform.rotation
        root_to_optical = transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        base_to_root = camera_root_in_base(self.calibration, root_to_optical)
        result = TransformStamped()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id, result.child_frame_id = 'base', self.root_frame
        p, rotation = result.transform.translation, result.transform.rotation
        p.x, p.y, p.z = (base_to_root[i][3] for i in range(3))
        rotation.x, rotation.y, rotation.z, rotation.w = quaternion_from_matrix(base_to_root)
        self.broadcaster.sendTransform(result)
        self.timer.cancel()
        self.get_logger().info('Published external-camera TF base -> ' + self.root_frame)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationTF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

"""Sanitize camera XYZ and relay reliable clouds to the Foxy MoveIt updater."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from .pointcloud_filter import finite_records


class ScenePointCloud(Node):
    def __init__(self):
        super().__init__('harvesting_scene_pointcloud')
        for key, default in (('input_topic', '/camera/depth/color/points'),
                             ('output_topic', '/harvesting/scene_points'), ('max_update_rate', 2.0)):
            self.declare_parameter(key, default)
        rate = float(self.get_parameter('max_update_rate').value)
        if not 0 < rate <= 30:
            raise ValueError('max_update_rate must be in (0, 30] Hz')
        self.interval = 1. / rate
        self.last = -float('inf')
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, QoSProfile(depth=1))
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value, self.receive, qos_profile_sensor_data)

    def receive(self, cloud):
        now = time.monotonic()
        if now - self.last < self.interval:
            return
        self.last = now
        try:
            count, records = finite_records(cloud)
            result = PointCloud2()
            result.header, result.fields = cloud.header, cloud.fields
            result.height, result.width = 1, count
            result.is_bigendian, result.is_dense = False, True
            result.point_step = cloud.point_step
            result.row_step = count * cloud.point_step
            result.data = records
            self.publisher.publish(result)
        except ValueError as error:
            self.get_logger().error('Scene point cloud rejected: ' + str(error))


def main(args=None):
    rclpy.init(args=args)
    node = ScenePointCloud()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

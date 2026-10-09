import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from custom_interface.srv import CameraSrv
from .detection_utils import DetectionHandler
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from .tf_utils import TFHandler
from .visualisation import VisualisationHandler

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        
        for name, value in {
            'model_file': 'best.pt', 'color_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/aligned_depth_to_color/camera_info',
            'show_image': True, 'coordinate_mode': 'eye_to_hand', 'camera_frame': 'camera_color_optical_frame',
            'target_frame': 'base_link', 'calibration_file': '',
        }.items():
            self.declare_parameter(name, value)
        if self.get_parameter('coordinate_mode').value != 'eye_to_hand':
            raise ValueError('coordinate_mode must be eye_to_hand for the fixed laboratory camera')
        if self.get_parameter('target_frame').value != 'base_link':
            raise ValueError('Eye-to-hand detections must be returned in base_link')
        # Setup callback groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.image_group = ReentrantCallbackGroup()

        # Setup components
        self.tf_handler = TFHandler(self)
        self.visualiser = VisualisationHandler(
            node=self,
            tf_handler=self.tf_handler,
        )

        self.detector = DetectionHandler(
            node=self,
            tf_handler=self.tf_handler,
            visualiser=self.visualiser
        )


        self.setup_subscribers()


        # Service
        self.srv = self.create_service(
            CameraSrv, 
            'camera_srv', 
            self.handle_camera_request,
            callback_group=self.service_group
        )
        
        self.get_logger().info("Camera Server ready")

    def setup_subscribers(self):
        """Configure image subscribers and synchronizer"""
        self.color_sub = Subscriber(
            self, 
            Image, 
            self.get_parameter('color_topic').value,
            callback_group=self.image_group, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, 
            Image, 
            self.get_parameter('depth_topic').value,
            callback_group=self.image_group, qos_profile=qos_profile_sensor_data
        )
                
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.detector.update_frames)
        
    async def handle_camera_request(self, request, response):
        result = await self.detector.handle_request(request)

        response.coordinates = result.get('coordinates', [])
        response.success = result.get('success', False)
        response.message = result.get('message', '')
        return response

def main():
    rclpy.init()
    server = CameraServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.visualiser.cleanup()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
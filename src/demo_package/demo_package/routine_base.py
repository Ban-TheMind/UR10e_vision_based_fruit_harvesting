"""Shared request handling: abort the routine on failed/unknown actuator results."""
import time
import math
import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, MovementRequest, GripperCmd, ResetGripperCmd


class RoutineBase(Node):
    def __init__(self, defaults):
        super().__init__('demo_routine')
        parameters = {
            'motion_enabled': False, 'calibration_verified': False,
            'service_timeout': 120.0, 'startup_timeout': 30.0, 'max_cycles': 1,
            'max_attempts': 3, 'max_detect_attempts': 20,
            'detection_class': 0, 'detection_confidence': 0.5, 'detection_min_confidence': 0.2,
            'gripper_open_width': 85, 'gripper_close_width': 0, 'gripper_force': 40,
            'settle_seconds': 1.5, 'grip_seconds': 2.0,
            **defaults,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if not self.motion_enabled or not self.calibration_verified:
            self.destroy_node()
            raise RuntimeError('Routine requires motion_enabled and field-verified calibration')
        if self.max_cycles < 1 or self.max_attempts < 1 or self.max_detect_attempts < 1 or self.service_timeout <= 0 or self.startup_timeout <= 0:
            raise ValueError('Counts and timeouts must be positive')
        for name in defaults:
            value = getattr(self, name)
            if isinstance(value, (list, tuple)) and not all(math.isfinite(x) for x in value):
                raise ValueError(f'Non-finite parameter: {name}')
        self.camera_client = self.create_client(CameraSrv, '/camera_srv')
        self.gripper_client = self.create_client(GripperCmd, '/gripper_cmd')
        self.reset_gripper_client = self.create_client(ResetGripperCmd, '/reset_gripper_cmd')
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        deadline = time.monotonic() + self.startup_timeout
        for client in (self.camera_client, self.gripper_client, self.reset_gripper_client, self.movement_client):
            while not client.wait_for_service(timeout_sec=0.5):
                if not rclpy.ok() or time.monotonic() >= deadline:
                    raise RuntimeError(f'Service unavailable: {client.srv_name}')
        self.get_logger().info('Routine ready; no automatic device reset will be performed')

    def call(self, client, request, actuator=False):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout)
        if not future.done():
            future.cancel()
            # Local cancellation does not cancel a remotely executing movement.
            raise RuntimeError(f'{client.srv_name} timed out; execution state unknown, stop and inspect device')
        response = future.result()
        if response is None or (actuator and not response.success):
            raise RuntimeError(f'{client.srv_name} failed: {getattr(response, "message", "no successful result")}')
        return response

    def send_camera_request(self, command, identifier=None, conf=None):
        request = CameraSrv.Request()
        request.command = command
        request.identifier = self.detection_class if identifier is None else identifier
        request.conf = self.detection_confidence if conf is None else conf
        return self.call(self.camera_client, request)

    def send_gripper_request(self, width, force=None):
        request = GripperCmd.Request()
        request.width = width
        request.force = self.gripper_force if force is None else force
        return self.call(self.gripper_client, request, actuator=True)

    def send_reset_gripper_request(self, reset):
        request = ResetGripperCmd.Request()
        request.reset_gripper = reset
        return self.call(self.reset_gripper_client, request, actuator=True)

    def send_movement_request(self, positions, command='cartesian', constraint='NONE'):
        request = MovementRequest.Request()
        request.command = command
        request.positions = [float(x) for x in positions]
        request.constraints_identifier = constraint
        return self.call(self.movement_client, request, actuator=True)


def run_routine(routine, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = routine()
        node.run_demo()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

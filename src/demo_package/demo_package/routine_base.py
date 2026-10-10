"""Shared request handling: abort the routine on failed/unknown actuator results."""
import time
import math
import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, MovementRequest, GripperCmd, ResetGripperCmd


def configure_execution_timeout(node, seconds):
    """Update Foxy's actual execution manager before sending any movement."""
    from rcl_interfaces.srv import SetParameters, GetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('execution_timeout must be finite and positive')
    # Foxy computes nominal_duration * scaling + margin. Keep cancellation
    # enabled, but use a fixed real execution budget instead of nominal timing.
    expected = [
        ('trajectory_execution.allowed_execution_duration_scaling',
         ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.0)),
        ('trajectory_execution.allowed_goal_duration_margin',
         ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(seconds))),
        ('trajectory_execution.execution_duration_monitoring',
         ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
    ]
    clients = []
    def exchange(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            future.cancel()
            raise RuntimeError('MoveIt timeout configuration request timed out; no movement sent')
        return future.result()
    try:
        for service, suffix in ((SetParameters, 'set_parameters'), (GetParameters, 'get_parameters')):
            client = node.create_client(service, '/moveit_simple_controller_manager/' + suffix)
            clients.append(client)
            if not client.wait_for_service(timeout_sec=8.0):
                raise RuntimeError('MoveIt execution parameter service unavailable; no movement sent')
        request = SetParameters.Request()
        request.parameters = [Parameter(name=name, value=value) for name, value in expected]
        response = exchange(clients[0], request)
        if response is None or len(response.results) != len(expected) or not all(
                result.successful for result in response.results):
            raise RuntimeError('MoveIt rejected execution timeout configuration; no movement sent')
        request = GetParameters.Request()
        request.names = [name for name, _ in expected]
        response = exchange(clients[1], request)
        if response is None or len(response.values) != len(expected):
            raise RuntimeError('MoveIt execution timeout readback failed; no movement sent')
        for actual, (name, wanted) in zip(response.values, expected):
            matches = actual.type == wanted.type
            if wanted.type == ParameterType.PARAMETER_BOOL:
                matches = matches and actual.bool_value == wanted.bool_value
            else:
                matches = matches and math.isfinite(actual.double_value) and abs(
                    actual.double_value - wanted.double_value) < 1e-6
            if not matches:
                raise RuntimeError('MoveIt execution timeout readback mismatch: ' + name)
        node.get_logger().info(
            'MoveIt 每段动作执行上限 {:.1f} 秒；名义时长倍率 0；参数已回读核验'.format(seconds))
    finally:
        for client in clients:
            node.destroy_client(client)


class RoutineBase(Node):
    def __init__(self, defaults):
        super().__init__('demo_routine')
        parameters = {
            'motion_enabled': False, 'calibration_verified': False,
            'service_timeout': 120.0, 'startup_timeout': 30.0, 'execution_timeout': 600.0, 'max_cycles': 1,
            'max_attempts': 3, 'max_detect_attempts': 20,
            'detection_class': 0, 'detection_confidence': 0.5, 'detection_min_confidence': 0.2,
            'gripper_open_width': 85, 'gripper_close_width': 0, 'gripper_force': 40,
            'settle_seconds': 1.5, 'grip_seconds': 2.0,
            **defaults,
        }
        for name, value in parameters.items():
            self.declare_parameter(name, value)
            setattr(self, name, self.get_parameter(name).value)
        if not self.motion_enabled:
            self.destroy_node()
            raise RuntimeError('Routine requires motion_enabled:=true')
        if not self.calibration_verified:
            self.get_logger().warning(
                'Calibration status is unverified; motion was explicitly enabled. '
                'Targets use the configured camera transform, pick offset and orientation.')
        if self.max_cycles < 1 or self.max_attempts < 1 or self.max_detect_attempts < 1 or self.service_timeout <= 0 or self.startup_timeout <= 0:
            raise ValueError('Counts and timeouts must be positive')
        if not math.isfinite(self.execution_timeout) or self.execution_timeout <= 0:
            raise ValueError('execution_timeout must be finite and positive')
        for name in defaults:
            value = getattr(self, name)
            if isinstance(value, (list, tuple)) and not all(math.isfinite(x) for x in value):
                raise ValueError(f'Non-finite parameter: {name}')
        for name, size in (('birds_eye_joint_pos', 6), ('bird_eye_position', 6),
                           ('drop_position', 6), ('approach_offset', 3),
                           ('pick_offset', 3), ('pick_orientation', 3),
                           ('retry_scan_delta', 6)):
            if len(getattr(self, name)) != size:
                raise ValueError('Invalid pose size: ' + name)
        if 'drop_joint_pos' in defaults and len(self.drop_joint_pos) != 6:
            raise ValueError('Invalid pose size: drop_joint_pos')
        if 'drop_motion' in defaults and self.drop_motion not in ('joint', 'cartesian'):
            raise ValueError('drop_motion must be joint or cartesian')
        for name in ('scan_at_home_only', 'return_to_scan_after_drop'):
            if name in defaults and type(getattr(self, name)) is not bool:
                raise ValueError(name + ' must be boolean')
        if not 0 <= self.detection_min_confidence <= self.detection_confidence <= 1:
            raise ValueError('Invalid detection confidence range')
        if not (0 <= self.gripper_close_width <= 85 and 0 <= self.gripper_open_width <= 85
                and 0 <= self.gripper_force <= 255 and self.settle_seconds >= 0
                and self.grip_seconds >= 0 and self.distance_tolerance > 0):
            raise ValueError('Invalid gripper, timing or distance parameters')
        self.camera_client = self.create_client(CameraSrv, '/camera_srv')
        self.gripper_client = self.create_client(GripperCmd, '/gripper_cmd')
        self.reset_gripper_client = self.create_client(ResetGripperCmd, '/reset_gripper_cmd')
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        deadline = time.monotonic() + self.startup_timeout
        for client in (self.camera_client, self.gripper_client, self.reset_gripper_client, self.movement_client):
            while not client.wait_for_service(timeout_sec=0.5):
                if not rclpy.ok() or time.monotonic() >= deadline:
                    raise RuntimeError(f'Service unavailable: {client.srv_name}')
        configure_execution_timeout(self, self.execution_timeout)
        self.get_logger().info('Routine ready; no automatic device reset will be performed')

    def call(self, client, request, actuator=False, timeout=None):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout if timeout is None else timeout)
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
        # Allow planning and response delivery around MoveIt's execution budget.
        return self.call(self.movement_client, request, actuator=True,
                         timeout=max(self.service_timeout, self.execution_timeout + 60.0))


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

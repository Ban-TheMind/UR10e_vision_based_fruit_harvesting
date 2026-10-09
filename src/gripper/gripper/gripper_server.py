#!/usr/bin/env python3
"""ROS services for Robotiq 2F-85; device access requires explicit enablement."""
import rclpy
from rclpy.node import Node
from custom_interface.srv import GripperCmd, ResetGripperCmd
from gripper.robotiq_sdk import RobotiqSDK


class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        self.declare_parameter('commands_enabled', False)
        for name, value in [('serial_port', '/dev/ttyUSB0'), ('baudrate', 115200),
                            ('slave_id', 9), ('serial_timeout', 0.5),
                            ('action_timeout', 10.0), ('speed', 64)]:
            self.declare_parameter(name, value)
        self.srv = self.create_service(GripperCmd, 'gripper_cmd', self.gripper_callback)
        self.reset_srv = self.create_service(ResetGripperCmd, 'reset_gripper_cmd', self.reset_gripper_callback)
        self.get_logger().info('Robotiq 2F-85 services ready; no automatic activation')

    def _driver(self):
        values = {name: self.get_parameter(name).value for name in
                  ('serial_port', 'baudrate', 'slave_id', 'serial_timeout', 'action_timeout', 'speed')}
        values['port'] = values.pop('serial_port')
        return RobotiqSDK(**values)

    def gripper_callback(self, request, response):
        response.success = False
        if not self.get_parameter('commands_enabled').value:
            response.message = 'Gripper commands disabled; explicitly enable motion in launch'
            return response
        try:
            # Validate before invoking the official SDK.
            from gripper.robotiq_sdk import position_for_width
            position_for_width(request.width)
            if not 0 <= request.force <= 255:
                raise ValueError('force must be 0..255 (raw Robotiq value, not N)')
            with self._driver() as driver:
                status = driver.move(request.width, request.force)
            response.success = True
            response.message = ('Gripper stopped: object_state={}, position_raw={}; '
                                'requested width={} mm (nominal), force_raw={}').format(
                                    status.object_state, status.position, request.width, request.force)
        except Exception as error:
            response.message = 'Gripper command failed: {}'.format(error)
        return response

    def reset_gripper_callback(self, request, response):
        response.success = False
        if not self.get_parameter('commands_enabled').value:
            response.message = 'Gripper commands disabled'
            return response
        if not request.reset_gripper:
            response.message = 'Reset denied by user input'
            return response
        try:
            with self._driver() as driver:
                driver.activate()
            response.success = True
            response.message = 'Robotiq reset and activation complete (not tool power reset)'
        except Exception as error:
            response.message = 'Gripper activation failed: {}'.format(error)
        return response


def main(args=None):
    rclpy.init(args=args)
    server = GripperServer()
    try:
        rclpy.spin(server)
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

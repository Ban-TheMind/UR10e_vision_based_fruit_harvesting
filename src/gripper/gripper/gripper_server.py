#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interface.srv import GripperCmd, ResetGripperCmd
import requests

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        self.declare_parameter('gripper_host', '192.168.1.1')
        self.declare_parameter('http_timeout', 5.0)
        self.declare_parameter('commands_enabled', False)
        self.srv = self.create_service(GripperCmd, 'gripper_cmd', self.gripper_callback)
        self.reset_srv = self.create_service(ResetGripperCmd, 'reset_gripper_cmd', self.reset_gripper_callback)
        self.get_logger().info('Gripper Server ready to receive commands...')
        
    def gripper_callback(self, request, response):
        if not self.get_parameter('commands_enabled').value:
            response.success = False
            response.message = 'Gripper commands disabled; launch with motion_enabled:=true'
            return response
        # Validate width
        if not (0 <= request.width <= 100):
            response.success = False
            response.message = f'Width must be between 0-100 (received {request.width})'
            return response
            
        # Validate force
        if not (3 <= request.force <= 40):
            response.success = False
            response.message = f'Force must be between 3-40 (received {request.force})'
            return response
            
        try:
            # Send command to gripper
            url = f"http://{self.get_parameter('gripper_host').value}/api/dc/rgxp2/set_width/0/{request.width}/{request.force}"
            res = requests.get(url, timeout=self.get_parameter('http_timeout').value)
            
            if res.status_code == 200:
                response.success = True
                response.message = f'Gripper set to width {request.width} with force {request.force}'
            else:
                response.success = False
                response.message = f'Gripper command failed with HTTP status {res.status_code}'
                
        except Exception as e:
            response.success = False
            response.message = f'Error communicating with gripper: {str(e)}'
            
        return response

  
    def reset_gripper_callback(self, request, response):  
        if not self.get_parameter('commands_enabled').value:
            response.success = False
            response.message = 'Gripper commands disabled'
            return response
        reset = request.reset_gripper

        if not reset:
            response.success = False
            response.message = f'Reset denied by user input'
            return response

        try:
            # Send command to gripper
            url = f"http://{self.get_parameter('gripper_host').value}/api/dc/reset_tool_power"
            res = requests.get(url, timeout=self.get_parameter('http_timeout').value)
            
            if res.status_code == 200:
                response.success = True
                response.message = f'Gripper reset success'
            else:
                response.success = False
                response.message = f'Gripper command failed with HTTP status {res.status_code}'
                
        except Exception as e:
            response.success = False
            response.message = f'Error communicating with gripper: {str(e)}'
            
        return response


def main(args=None):
    rclpy.init(args=args)
    gripper_server = GripperServer()
    rclpy.spin(gripper_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
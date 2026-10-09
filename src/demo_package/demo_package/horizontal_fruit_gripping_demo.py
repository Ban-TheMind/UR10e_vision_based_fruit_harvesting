"""Field-configurable harvesting strategy; launch through harvesting_bringup."""
import time
import copy
from .routine_base import RoutineBase, run_routine
NO_CONSTRAINT = 'NONE'
DOWN_CONSTRAINT = 'DOWN'
SCAN_MODE = 'scan'
GRIP_MODE = 'grip'

class DemoRoutine(RoutineBase):
    def __init__(self):
        super().__init__({'bird_eye_position': [0.822, 0.183, 0.856, 0.0, 3.14, 0.0], 'drop_position': [0.822, 0.583, 0.556, 0.0, 3.14, 0.0], 'approach_height': 0.67, 'hover_height': 0.5, 'pick_height': 0.19, 'retry_delta': [-0.01, -0.01, -0.02], 'height_tolerance': 0.05, 'detection_class': 47})

    def run_demo(self):
        for cycle in range(self.max_cycles):
            birds_eye_position_copy = copy.deepcopy(self.bird_eye_position)
            detected_apples = self.run_detection_at_pos(birds_eye_position_copy)

            if not detected_apples:
                break

            extracted_apple_height = detected_apples[0].z
            adjusted_birds_eye_position = copy.deepcopy(self.bird_eye_position)
            adjusted_birds_eye_position[0] = detected_apples[0].x
            adjusted_birds_eye_position[1] = detected_apples[0].y
            adjusted_birds_eye_position[2] = extracted_apple_height + self.approach_height

            adjusted_above_drop_off_position = copy.deepcopy(self.drop_position)

            gripping_pos_array = self.run_detection_at_pos(adjusted_birds_eye_position)
            if not gripping_pos_array:
                break
            filtered_pos_array = self.filter_apples_for_pickup(gripping_pos_array, extracted_apple_height)

            if not filtered_pos_array:
                break

            self.get_logger().info("gripper init")
            self.send_gripper_request(self.gripper_open_width)  # Open gripper

            for apple in filtered_pos_array:
                x, y, z = apple.x, apple.y, apple.z
                above_apple_height = z + self.hover_height

                above_apple = [x, y, above_apple_height, 0.0, 3.14, 0.0]
                adjusted_above_drop_off_position[2] = above_apple_height
                pick_position = [x, y, z + self.pick_height, 0.0, 3.14, 0.0]


                self.get_logger().info(f"Processing apple at position: {x}, {y}, {z}")

                self.get_logger().info("Moving above apple")
                self.send_movement_request(above_apple)

                self.get_logger().info("Lowering to pick height")
                self.send_movement_request(pick_position)

                self.get_logger().info("Gripping apple")
                self.send_gripper_request(self.gripper_close_width)  # Close gripper # 78mm is used as 0mm would trigger a safety fault

                time.sleep(0.5)

                self.get_logger().info("Lifting apple")
                self.send_movement_request(above_apple)


                self.get_logger().info("Moving to drop position")
                self.send_movement_request(adjusted_above_drop_off_position)

                self.send_movement_request(self.drop_position)

                self.get_logger().info("Releasing apple")
                self.send_gripper_request(self.gripper_open_width)  # Open gripper


        self.send_movement_request(self.bird_eye_position)

        self.get_logger().info("Demo routine completed")

    def run_detection_at_pos(self, position):
            self.get_logger().info("Detecting apples")

            attempt = 0
            while attempt < self.max_attempts:
                attempt += 1
                self.get_logger().info(f"Detection attempt {attempt}/{self.max_attempts}")

                position[:3] = [p + d for p, d in zip(position[:3], self.retry_delta)]

                self.get_logger().info("Moving to bird's eye view")
                self.send_movement_request(position)

                time.sleep(self.settle_seconds)

                camera_response = self.send_camera_request("detect", self.detection_class)

                if camera_response and camera_response.success and camera_response.coordinates:
                    self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                    return camera_response.coordinates

                self.get_logger().info(f"Detection failed on attempt {attempt}")
                time.sleep(0.5)  # Brief pause between attempts

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None

    def filter_apples_for_pickup(self, pos_array, target_height, height_tolerance=None):
        """
        Filters apples to only those within height tolerance of target height.

        Args:
            pos_array: List of Point messages or arrays containing apple positions
            target_height: Desired Z height (in meters)
            height_tolerance: Allowed ± variation from target height (default: 0.15m)

        Returns:
            List of filtered apples meeting height criteria
        """
        height_tolerance = self.height_tolerance if height_tolerance is None else height_tolerance
        filtered_apples = []

        for apple in pos_array:
            if abs(apple.z - target_height) <= height_tolerance:
                filtered_apples.append(apple)

        return filtered_apples

def main(args=None):
    run_routine(DemoRoutine, args)


if __name__ == '__main__':
    main()

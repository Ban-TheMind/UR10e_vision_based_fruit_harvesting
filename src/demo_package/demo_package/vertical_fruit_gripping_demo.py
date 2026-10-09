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
        super().__init__({'bird_eye_position': [0.54, 0.174, 0.9, -1.56, 0.0, -1.571], 'birds_eye_joint_pos': [-1.9, 1.2, -2.0, -1.57, 0.0, 0.0], 'drop_position': [0.822, 0.583, 0.556, 0.0, 3.14, 0.0], 'approach_offset': [-0.5, 0.0, -0.1], 'pick_offset': [-0.18, 0.06, -0.03], 'pick_orientation': [-1.56, 0.0, -1.571], 'retry_scan_delta': [0.0, -0.08, 0.15, -0.3, 0.0, 0.0], 'retry_grip_delta': [0.0, 0.05, -0.03, 0.0, 0.0, 0.0], 'distance_tolerance': 0.06})

    def run_demo(self):

        for cycle in range(self.max_cycles):

            self.send_movement_request(self.birds_eye_joint_pos, "joint", NO_CONSTRAINT)

            detected_apples = self.run_detection_at_curr_pos()
            print(f"detected_apples are {detected_apples}")

            if not detected_apples:
                print(f"no apple detected, ending")
                break

            extracted_apple_distance = detected_apples[0].x
            adjusted_birds_eye_position = copy.deepcopy(self.bird_eye_position)
            adjusted_birds_eye_position[0] = extracted_apple_distance + self.approach_offset[0]
            adjusted_birds_eye_position[1] = detected_apples[0].y + self.approach_offset[1]
            adjusted_birds_eye_position[2] = detected_apples[0].z + self.approach_offset[2]

            adjusted_above_drop_off_position = copy.deepcopy(self.drop_position)

            gripping_pos_array = self.run_detection_at_pos(adjusted_birds_eye_position, SCAN_MODE)

            if not gripping_pos_array:
                print(f"no apples detected, ending")
                break

            filtered_pos_array = self.filter_apples_for_pickup(gripping_pos_array, extracted_apple_distance)
            print(f"gripper pose detected_apples are {gripping_pos_array}")

            if not filtered_pos_array:
                print(f"bad photo, redetecting")
                continue

            self.get_logger().info("gripper init")
            self.send_gripper_request(self.gripper_open_width)  # Open gripper

            for apple in filtered_pos_array:
                x, y, z = apple.x, apple.y, apple.z
                above_apple_distance = x + self.approach_offset[0]
                y_compensation = y + self.pick_offset[1]
                z_compensation = z + self.pick_offset[2]

                above_apple = [above_apple_distance, y_compensation, z_compensation, -1.56, -0.0, -1.571]
                adjusted_above_drop_off_position[0] = above_apple_distance
                pick_position = [x + self.pick_offset[0], y_compensation, z_compensation, *self.pick_orientation]


                self.get_logger().info(f"Processing apple at position: {x}, {y}, {z}")


                self.get_logger().info("Lowering to pick height")
                self.send_movement_request(pick_position)

                self.get_logger().info("Gripping apple")
                self.send_gripper_request(self.gripper_close_width)  # Close Robotiq gripper

                time.sleep(self.grip_seconds)




                self.get_logger().info("Moving to drop position")

                self.send_movement_request(self.drop_position)

                self.get_logger().info("Releasing apple")
                self.send_gripper_request(self.gripper_open_width)  # Open gripper


        self.send_movement_request(self.birds_eye_joint_pos, "joint", NO_CONSTRAINT)

        self.get_logger().info("Demo routine completed")

    def run_detection_at_pos(self, position, mode):
            self.get_logger().info("Detecting apples")

            attempt = 0
            while attempt < self.max_attempts:
                attempt += 1
                self.get_logger().info(f"Detection attempt {attempt}/{self.max_attempts}")

                if (attempt != 1 and mode == SCAN_MODE):
                    position[:] = [p + d for p, d in zip(position, self.retry_scan_delta)]
                elif (mode == GRIP_MODE):
                    position[:] = [p + d for p, d in zip(position, self.retry_grip_delta)]

                self.get_logger().info("Moving to bird's eye view")
                self.send_movement_request(position)

                time.sleep(self.settle_seconds)


                max_detect_attempt = self.max_detect_attempts
                conf = self.detection_confidence
                for attempt in range(1, max_detect_attempt + 1):
                    camera_response = self.send_camera_request("detect", self.detection_class, conf)
                    if camera_response and camera_response.success and camera_response.coordinates:
                        self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                        return camera_response.coordinates
                    else:
                        self.get_logger().info(f"YOLO detection attempt {attempt} with conf: {conf} failed")
                        time.sleep(0.5)  # Brief pause between attempts

                        conf = max(self.detection_min_confidence, conf - 0.05)

                self.get_logger().info(f"Detection failed on attempt {attempt}")
                time.sleep(0.5)  # Brief pause between attempts

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None


    def run_detection_at_curr_pos(self):
            self.get_logger().info("Detecting apples")


            max_detect_attempt = self.max_detect_attempts
            conf = self.detection_confidence
            for attempt in range(1, max_detect_attempt + 1):
                camera_response = self.send_camera_request("detect", self.detection_class, conf)
                if camera_response and camera_response.success and camera_response.coordinates:
                    self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                    return camera_response.coordinates
                else:
                    self.get_logger().info(f"YOLO detection attempt {attempt} with conf: {conf} failed")
                    conf = max(self.detection_min_confidence, conf - 0.05)
                time.sleep(0.5)

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None

    def filter_apples_for_pickup(self, pos_array, target_distance, distance_tolerance=None):
        """
        Filters apples to only those within height tolerance of target height.

        Args:
            pos_array: List of Point messages or arrays containing apple positions
            target_distance: Desired distance between camera and item (in meters)
            height_tolerance: Allowed ± variation from target height (default: 0.15m)

        Returns:
            List of filtered apples meeting height criteria
        """
        distance_tolerance = self.distance_tolerance if distance_tolerance is None else distance_tolerance
        filtered_apples = []

        if len(pos_array) == 1:
            return pos_array

        for apple in pos_array:
            if abs(apple.x - target_distance) <= distance_tolerance:
                filtered_apples.append(apple)
            else:
                print(f"filtered out, difference is {abs(apple.x - target_distance)}")

        return filtered_apples

def main(args=None):
    run_routine(DemoRoutine, args)


if __name__ == '__main__':
    main()

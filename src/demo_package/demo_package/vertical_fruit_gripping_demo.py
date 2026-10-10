"""Bounded harvesting strategy; configure through harvesting_bringup."""
import time
from .routine_base import RoutineBase, run_routine


class DemoRoutine(RoutineBase):
    def __init__(self):
        # Standard UR order: pan, lift, elbow, wrist1, wrist2, wrist3; radians.
        # These inherited poses are examples and require field verification.
        super().__init__({
            'bird_eye_position': [0.54, 0.174, 0.9, -1.56, 0.0, -1.571],
            'birds_eye_joint_pos': [0.0, -1.9, 1.2, -2.0, -1.57, 0.0],
            'drop_position': [0.822, 0.583, 0.556, 0.0, 3.14, 0.0],
            'drop_motion': 'cartesian',
            'drop_joint_pos': [-3.141592653589793, -1.5707963267948966,
                               2.2165681500327987, -2.1467549799530254,
                               4.71238898038469, 0.0],
            'scan_at_home_only': False,
            'return_to_scan_after_drop': True,
            'approach_offset': [-0.5, 0.0, -0.1],
            'pick_offset': [-0.18, 0.06, -0.03],
            'pick_orientation': [-1.56, 0.0, -1.571],
            'retry_scan_delta': [0.0, -0.08, 0.15, -0.3, 0.0, 0.0],
            'distance_tolerance': 0.06,
        })

    def run_demo(self):
        for cycle in range(self.max_cycles):
            self.send_movement_request(self.birds_eye_joint_pos, 'joint', 'NONE')
            if self.scan_at_home_only:
                time.sleep(self.settle_seconds)
            apples = self.run_detection_at_curr_pos()
            if not apples:
                break
            if not self.scan_at_home_only:
                reference = apples[0]
                scan = list(self.bird_eye_position)
                scan[:3] = [reference.x + self.approach_offset[0],
                            reference.y + self.approach_offset[1],
                            reference.z + self.approach_offset[2]]
                apples = self.run_detection_at_pos(scan)
                if not apples:
                    break
                apples = self.filter_apples_for_pickup(apples, reference.x)
                if not apples:
                    continue
            self.send_gripper_request(self.gripper_open_width)
            for apple in apples:
                pick = [apple.x + self.pick_offset[0],
                        apple.y + self.pick_offset[1],
                        apple.z + self.pick_offset[2], *self.pick_orientation]
                self.send_movement_request(pick)
                self.send_gripper_request(self.gripper_close_width)
                time.sleep(self.grip_seconds)
                # Keep holding until the drop movement has returned success.
                if self.drop_motion == 'joint':
                    self.send_movement_request(self.drop_joint_pos, 'joint', 'NONE')
                else:
                    self.send_movement_request(self.drop_position)
                self.send_gripper_request(self.gripper_open_width)
        if self.return_to_scan_after_drop:
            self.send_movement_request(self.birds_eye_joint_pos, 'joint', 'NONE')
        self.get_logger().info('Demo routine completed')

    def run_detection_at_pos(self, position):
        position = list(position)
        for attempt in range(self.max_attempts):
            if attempt:
                position = [value + delta for value, delta in zip(position, self.retry_scan_delta)]
            self.send_movement_request(position)
            time.sleep(self.settle_seconds)
            apples = self.run_detection_at_curr_pos()
            if apples:
                return apples
        return []

    def run_detection_at_curr_pos(self):
        confidence = self.detection_confidence
        for attempt in range(self.max_detect_attempts):
            response = self.send_camera_request('detect', self.detection_class, confidence)
            if response.success and response.coordinates:
                return response.coordinates
            confidence = max(self.detection_min_confidence, confidence - 0.05)
            time.sleep(0.5)
        return []

    def filter_apples_for_pickup(self, positions, target_distance, distance_tolerance=None):
        tolerance = self.distance_tolerance if distance_tolerance is None else distance_tolerance
        return [apple for apple in positions if abs(apple.x - target_distance) <= tolerance]


def main(args=None):
    run_routine(DemoRoutine, args)


if __name__ == '__main__':
    main()

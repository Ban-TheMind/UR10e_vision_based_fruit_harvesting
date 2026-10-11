"""Tabletop target geometry in MoveIt's base_link frame; no ROS dependency."""
import math

def tabletop_poses(apple, height, orientation, xy_offset, clearance):
    values = [apple.x, apple.y, apple.z, height, clearance, *orientation, *xy_offset]
    if len(orientation) != 3 or len(xy_offset) != 2 or not all(math.isfinite(x) for x in values):
        raise ValueError('Tabletop geometry requires finite height, RPY[3] and XY offset[2]')
    if clearance <= 0:
        raise ValueError('Tabletop clearance must be positive')
    # Camera Z measures the visible fruit surface, not the tool grasp height.
    pick = [apple.x + xy_offset[0], apple.y + xy_offset[1], height, *orientation]
    above = list(pick)
    above[2] += clearance
    return above, pick

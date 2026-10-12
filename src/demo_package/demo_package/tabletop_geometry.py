"""Tabletop geometry in base_link. Taught height is always tool0 Z."""
import math

DEFAULT_TOOL_OFFSET = (0.0, 0.0, 0.1628)

def rpy_rotation(orientation):
    r, p, y = orientation
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))

def rotate_offset(rotation, offset):
    return [sum(row[i]*offset[i] for i in range(3)) for row in rotation]

def gripper_center_to_tool0(center, orientation, tool_offset=DEFAULT_TOOL_OFFSET):
    """p_tool0 = p_center - R_base_link_tool0 * offset_tool0_center."""
    if (len(center) != 3 or len(orientation) != 3 or len(tool_offset) != 3 or
            not all(math.isfinite(x) for x in (*center, *orientation, *tool_offset))):
        raise ValueError('Tool transform requires finite XYZ, RPY and tool_offset_m[3]')
    delta = rotate_offset(rpy_rotation(orientation), tool_offset)
    return [center[i]-delta[i] for i in range(3)] + list(orientation)

def tabletop_poses(apple, height, orientation, xy_offset, clearance,
                   tool_offset=DEFAULT_TOOL_OFFSET):
    values = [apple.x, apple.y, apple.z, height, clearance, *orientation, *xy_offset, *tool_offset]
    if (len(orientation) != 3 or len(xy_offset) != 2 or len(tool_offset) != 3 or
            not all(math.isfinite(x) for x in values)):
        raise ValueError('Tabletop geometry requires finite tool0 height, RPY[3], XY offset[2] and tool offset[3]')
    if clearance <= 0:
        raise ValueError('Tabletop clearance must be positive')
    # Height was taught/read through FK at tool0, not at the fingertips.
    # Recover center Z first, then invert the full rigid tool translation;
    # this preserves taught tool0 Z instead of compensating its height twice.
    delta = rotate_offset(rpy_rotation(orientation), tool_offset)
    center = [apple.x+xy_offset[0], apple.y+xy_offset[1], height+delta[2]]
    pick = gripper_center_to_tool0(center, orientation, tool_offset)
    pick[2] = height
    above = list(pick)
    above[2] += clearance
    return above, pick

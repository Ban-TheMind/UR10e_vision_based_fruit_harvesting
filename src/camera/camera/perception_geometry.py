"""Rigid transforms for the external-camera TF bridge (no ROS dependency)."""
import math


def transform_matrix(translation, quaternion):
    if len(translation) != 3 or len(quaternion) != 4:
        raise ValueError('Expected translation XYZ and quaternion XYZW')
    if not all(math.isfinite(v) for v in (*translation, *quaternion)):
        raise ValueError('Transform must be finite')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-12:
        raise ValueError('Zero quaternion')
    x, y, z, w = [v/norm for v in quaternion]
    return (
        (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), translation[0]),
        (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), translation[1]),
        (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), translation[2]),
        (0., 0., 0., 1.))


def compose(left, right):
    return tuple(tuple(sum(left[i][k]*right[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def inverse_rigid(matrix):
    rows = [tuple(matrix[j][i] for j in range(3)) for i in range(3)]
    return tuple(row + (-sum(row[k]*matrix[k][3] for k in range(3)),)
                 for row in rows) + ((0., 0., 0., 1.),)


def quaternion_from_matrix(matrix):
    """Convert a proper rotation to a normalized XYZW quaternion."""
    r = matrix
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.) * 2
        q = ((r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s,
             (r[1][0]-r[0][1])/s, s/4)
    else:
        i = max(range(3), key=lambda k: r[k][k])
        j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(max(0., 1.+r[i][i]-r[j][j]-r[k][k])) * 2
        if s < 1e-12:
            raise ValueError('Invalid rotation')
        q = [0., 0., 0., 0.]
        q[i] = s/4
        q[j] = (r[i][j]+r[j][i])/s
        q[k] = (r[i][k]+r[k][i])/s
        q[3] = (r[k][j]-r[j][k])/s
    norm = math.sqrt(sum(v*v for v in q))
    return tuple(v/norm for v in q)


def camera_root_in_base(camera_to_base, root_to_optical):
    """B_T_root = B_T_optical * inverse(root_T_optical).

    Attach the driver's root frame, never give its existing optical child a
    second parent. UR's base_link-to-base rotation remains in the robot TF.
    """
    return compose(camera_to_base, inverse_rigid(root_to_optical))

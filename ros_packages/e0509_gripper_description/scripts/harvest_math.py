#!/usr/bin/env python3
"""Small math helpers shared by harvest runtime scripts.

Quaternion convention in this package is wxyz unless a function name says
otherwise. Keep this module free of ROS/cuRobo imports so it can be unit-checked
without launching the robot stack.
"""

import numpy as np


def quat_multiply_wxyz(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def quat_normalize_wxyz(q):
    q = np.array(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-9 or not np.all(np.isfinite(q)):
        return None
    return (q / n).tolist()


def quat_from_axis_angle(axis, angle_rad):
    axis = np.array(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return [1.0, 0.0, 0.0, 0.0]
    axis = axis / n
    s = np.sin(angle_rad / 2.0)
    return [np.cos(angle_rad / 2.0), axis[0] * s, axis[1] * s, axis[2] * s]


def quat_rotate_vec(q_wxyz, v):
    """Rotate vector v by quaternion q_wxyz=[w,x,y,z]."""
    w, x, y, z = q_wxyz
    qvec = np.array([x, y, z])
    v = np.array(v, dtype=float)
    t = 2.0 * np.cross(qvec, v)
    return v + w * t + np.cross(qvec, t)


def normalized_vec(v, min_norm=1e-9):
    v = np.array(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < min_norm or not np.all(np.isfinite(v)):
        return None
    return v / n

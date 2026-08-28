"""
Small rotation helpers, kept free of ROS so they can be tested on their own.

Written out rather than pulled from transforms3d: this package otherwise needs
nothing but numpy, and one more runtime dependency on the robot for a dozen
lines of algebra is a poor trade.
"""

import numpy as np


def quat_from_matrix(r):
    """
    Return ``(w, x, y, z)`` for a 3x3 rotation matrix.

    Shepperd's method: the trace branch is the numerically stable one whenever
    the trace is positive, and otherwise the largest diagonal element decides
    which of the three remaining branches is safe to divide by.  Picking one
    branch unconditionally is what produces silently wrong orientations near
    180 degrees.
    """
    r = np.asarray(r, dtype=np.float64)
    trace = float(r[0, 0] + r[1, 1] + r[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return (0.25 / s,
                float(r[2, 1] - r[1, 2]) * s,
                float(r[0, 2] - r[2, 0]) * s,
                float(r[1, 0] - r[0, 1]) * s)

    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(max(1e-12, 1.0 + float(r[i, i]) - float(r[j, j]) - float(r[k, k])))
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = float(r[k, j] - r[j, k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = float(r[j, i] + r[i, j]) / s
    q[k + 1] = float(r[k, i] + r[i, k]) / s
    return tuple(q)

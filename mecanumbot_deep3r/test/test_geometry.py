"""Tests for the rotation helper, which has no ROS in it and needs none."""

import numpy as np

from mecanumbot_deep3r.geometry import quat_from_matrix


def rotation(axis, angle):
    """Rodrigues, so the test does not depend on the code it is testing."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * (k @ k)


def as_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_identity():
    assert np.allclose(quat_from_matrix(np.eye(3)), (1.0, 0.0, 0.0, 0.0))


def test_round_trips_for_a_spread_of_rotations():
    rng = np.random.default_rng(0)
    for _ in range(50):
        axis = rng.normal(size=3)
        angle = rng.uniform(-np.pi, np.pi)
        r = rotation(axis, angle)

        q = quat_from_matrix(r)

        assert np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(as_matrix(q), r, atol=1e-9)


def test_near_180_degrees():
    """The branch that exists for this case; one unconditional branch gets it wrong."""
    for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0]):
        r = rotation(axis, np.pi - 1e-6)

        q = quat_from_matrix(r)

        assert np.isclose(np.linalg.norm(q), 1.0)
        assert np.allclose(as_matrix(q), r, atol=1e-6)

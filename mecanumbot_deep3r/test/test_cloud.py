"""
Tests for the wire-format decoder, with no ROS and no server.

The cloud arrives base64'd, quantised and rescaled against its own extent, so
a mistake here does not raise -- it produces a plausible cloud in the wrong
place, which is the failure mode a costmap is least able to survive and a
human is least likely to spot.
"""

import base64

import numpy as np
import pytest

from mecanumbot_deep3r import cloud as C


def encode(points, colors=None):
    """Quantise exactly the way the server does, so the tests exercise its format."""
    points = np.asarray(points, dtype=np.float64)
    origin = points.min(axis=0)
    extent = float(np.max(points.max(axis=0) - origin))
    scale = extent / 65535.0 if extent > 0 else 1.0
    q = np.clip((points - origin) / scale, 0, 65535).astype("<u2")
    out = {
        "n_points": int(points.shape[0]),
        "encoding": C.ENCODING,
        "origin": origin.tolist(),
        "scale": scale,
        "xyz_u16": base64.b64encode(q.tobytes()).decode("ascii"),
        "rgb_u8": base64.b64encode(np.asarray(colors, np.uint8).tobytes()).decode("ascii")
                  if colors is not None else "",
    }
    return out


def test_decode_round_trips_within_quantisation_error():
    pts = np.random.default_rng(0).uniform(-5, 5, size=(200, 3))

    back, colors = C.decode(encode(pts))

    assert back.dtype == np.float32
    assert colors is None
    # 16 bits over a 10 m extent is a fifth of a millimetre.
    assert np.abs(back - pts).max() < 1e-3


def test_decode_returns_colors_when_present():
    pts = np.array([[0.0, 0.0, 1.0], [1.0, 1.0, 2.0]])
    rgb = np.array([[10, 20, 30], [40, 50, 60]], np.uint8)

    _, colors = C.decode(encode(pts, rgb))

    assert colors.tolist() == rgb.tolist()


def test_an_empty_cloud_is_a_normal_frame():
    """Everything filtered out is a reading, not an error."""
    points, colors = C.decode({"n_points": 0})

    assert points.shape == (0, 3)
    assert colors is None


def test_an_unknown_encoding_is_refused():
    """A future format must not be silently read as this one."""
    cloud = encode(np.zeros((2, 3)))
    cloud["encoding"] = "u32le-xyz"

    with pytest.raises(C.CloudFormatError, match="unknown cloud encoding"):
        C.decode(cloud)


def test_a_truncated_payload_is_refused():
    cloud = encode(np.random.default_rng(1).uniform(0, 1, size=(10, 3)))
    cloud["n_points"] = 20

    with pytest.raises(C.CloudFormatError, match="claims 20 points"):
        C.decode(cloud)


def test_mismatched_colours_are_dropped_not_fatal():
    """Geometry is what navigation needs; losing a cloud over its colours is worse."""
    pts = np.array([[0.0, 0.0, 1.0], [1.0, 1.0, 2.0]])
    cloud = encode(pts, np.array([[1, 2, 3], [4, 5, 6]], np.uint8))
    cloud["rgb_u8"] = base64.b64encode(bytes([1, 2, 3])).decode("ascii")

    points, colors = C.decode(cloud)

    assert points.shape == (2, 3)
    assert colors is None


def test_has_cloud_rejects_a_skipped_result():
    """A skipped frame still replies -- the client counts replies -- but has nothing."""
    assert C.has_cloud({"cloud": {}}) is True
    assert C.has_cloud({"cloud": {}, "status": "skipped"}) is False
    assert C.has_cloud({}) is False
    assert C.has_cloud(None) is False


def test_to_xyzrgb_lays_points_out_for_pointcloud2():
    pts = np.array([[1.0, 2.0, 3.0]], np.float32)
    rgb = np.array([[255, 128, 64]], np.uint8)

    packed = C.to_xyzrgb(pts, rgb)

    assert packed.shape == (1, 4)
    assert packed[0, :3].tolist() == [1.0, 2.0, 3.0]
    # RViz reads the fourth float's *bits* as 0x00RRGGBB, so the round trip
    # has to go through the bit pattern rather than the value.
    assert packed[0, 3:4].view(np.uint32)[0] == (255 << 16) | (128 << 8) | 64


def test_to_xyzrgb_without_colours_leaves_the_channel_zero():
    packed = C.to_xyzrgb(np.array([[1.0, 2.0, 3.0]], np.float32), None)
    assert packed[0, 3] == 0.0

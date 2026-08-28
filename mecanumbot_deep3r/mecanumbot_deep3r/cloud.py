"""
Decode the point cloud the server sends back.

Pure numpy: no ROS, no ZeroMQ, no server.  The wire format is small enough to
describe in one place and awkward enough (base64, quantised, per-cloud origin)
that getting it wrong yields a plausible-looking cloud in the wrong place, so
it is worth being able to test it on its own.

The server quantises each cloud against its *own* extent rather than a fixed
scale, so ``origin`` and ``scale`` differ from frame to frame and both are
needed to reconstruct metres:

    p = origin + scale * u        u is little-endian uint16, xyz interleaved
"""

import base64

import numpy as np

#: What the server writes in ``cloud["encoding"]``.  Checked rather than
#: assumed: a future encoding would otherwise be silently misread as this one.
ENCODING = "u16le-xyz"


class CloudFormatError(ValueError):
    """The payload is not a cloud this decoder understands."""


def has_cloud(result):
    """
    Say whether a result carries a cloud at all.

    A result with ``status`` is one the server chose not to run the model on
    (``every_n``), and it still has to be counted -- the client sizes its
    in-flight window from replies -- but there is nothing to publish.
    """
    return isinstance(result, dict) and "cloud" in result and "status" not in result


def decode(cloud):
    """
    Return ``(points, colors)`` for one cloud.

    ``points`` is (N, 3) float32 in metres; ``colors`` is (N, 3) uint8 or None
    when the server was configured without them.  An empty cloud -- everything
    filtered out, which is a normal frame rather than an error -- comes back as
    a (0, 3) array rather than None, so callers do not need a second branch.
    """
    if not isinstance(cloud, dict):
        raise CloudFormatError(f"expected a dict, got {type(cloud).__name__}")

    n = int(cloud.get("n_points", 0))
    if n == 0:
        return np.zeros((0, 3), np.float32), None

    encoding = cloud.get("encoding")
    if encoding != ENCODING:
        raise CloudFormatError(
            f"unknown cloud encoding {encoding!r}; this decoder handles {ENCODING!r}"
        )

    raw = np.frombuffer(base64.b64decode(cloud["xyz_u16"]), "<u2")
    if raw.size != n * 3:
        raise CloudFormatError(
            f"cloud claims {n} points but carries {raw.size / 3:.1f}"
        )

    origin = np.asarray(cloud["origin"], dtype=np.float64)
    scale = float(cloud["scale"])
    points = (origin + scale * raw.reshape(-1, 3)).astype(np.float32)

    colors = None
    packed = cloud.get("rgb_u8") or ""
    if packed:
        rgb = np.frombuffer(base64.b64decode(packed), np.uint8)
        if rgb.size == n * 3:
            colors = rgb.reshape(-1, 3)
        # A colour array of the wrong length is dropped rather than raised on:
        # the geometry is what navigation needs, and losing a whole cloud over
        # its colours would be the wrong trade.

    return points, colors


def to_xyzrgb(points, colors):
    """
    Pack into the interleaved xyz+rgb buffer PointCloud2 wants.

    RViz and the Nav2 costmaps read ``rgb`` as a float32 whose bits are
    ``0x00RRGGBB``; that is a convention rather than anything numeric, hence
    the view rather than a cast.
    """
    n = points.shape[0]
    out = np.zeros((n, 4), dtype=np.float32)
    out[:, :3] = points
    if colors is not None:
        packed = (colors[:, 0].astype(np.uint32) << 16
                  | colors[:, 1].astype(np.uint32) << 8
                  | colors[:, 2].astype(np.uint32))
        out[:, 3] = packed.view(np.float32)
    return out

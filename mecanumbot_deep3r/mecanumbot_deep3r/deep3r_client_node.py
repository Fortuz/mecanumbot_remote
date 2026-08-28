"""
Stream camera frames to the off-board CUT3R server and republish the cloud.

The reconstruction runs on the cluster, not here: a monocular pointmap model
does not fit on the Orin alongside the rest of the stack.  This node is the
robot's end of that loop -- it takes the frames the camera is already
publishing, hands them to the RoboCam client, and turns what comes back into a
``PointCloud2``.

Why it subscribes rather than opening the camera
------------------------------------------------
``mecanumbot_camera_stream`` already owns the camera on the Orin.  A second
process opening ``/dev/video0`` would either fail or take the device away from
it, so this node consumes the compressed topic that node publishes.  That also
means the JPEG is already encoded: the frames are forwarded to the server
untouched, with no decode-re-encode round trip on the robot's CPU.

Timestamps
----------
The cloud has to be stamped with when the picture was *taken*, not when the
reply arrived -- a round trip is a couple of hundred milliseconds, and on a
moving robot a cloud stamped on arrival is placed where the robot is now
rather than where it was looking, which smears the map by exactly the distance
travelled during the round trip.

The client does not carry per-frame metadata, so the correlation is done on
its clock: every reply echoes ``t_send_ns``, a ``time.monotonic_ns()`` reading
from *this machine*, and this node records the same clock when it hands each
frame over.  Matching a reply to the most recent hand-over at or before its
``t_send_ns`` recovers the right image, and does so without assuming that
every frame handed over was sent -- the client drops frames at the source
under load, which would break any scheme based on counting.
"""

import collections
import importlib.util
import os
import queue
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from std_msgs.msg import Header

from . import cloud as cloud_codec
from .geometry import quat_from_matrix


class RosImageSource:
    """
    Feed a RoboCam ``FrameSource`` from a ROS subscription.

    Duck-typed rather than subclassed: the client is a standalone file deployed
    from ``RoboCamStreamProcessing/link``, not a package this one can import at
    module scope.  The contract is small -- ``width``, ``height``, ``fps``,
    ``frames()`` and ``close()``.

    ``width``/``height`` are advertised metadata only; the server reports what
    it actually decoded rather than what the client claimed, so they do not
    have to be right for the pipeline to be correct.
    """

    def __init__(self, width, height, fps, queue_depth=2):
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        # One slot more than the client's in-flight window is plenty: a frame
        # waiting here is a frame going stale, and the robot would rather send
        # the newest one than a queued old one.
        self._q = queue.Queue(maxsize=max(1, queue_depth))
        self._stop = threading.Event()
        #: (monotonic_ns at hand-over, ROS stamp ns) for reply correlation.
        self.handovers = collections.deque(maxlen=256)
        self.dropped = 0

    def submit(self, jpeg, stamp_ns):
        """Offer a frame; drop the oldest rather than block the ROS executor."""
        try:
            self._q.put_nowait((jpeg, stamp_ns))
        except queue.Full:
            try:
                self._q.get_nowait()
                self._q.put_nowait((jpeg, stamp_ns))
            except queue.Empty:  # pragma: no cover - lost the race, harmless
                pass
            self.dropped += 1

    def frames(self):
        while not self._stop.is_set():
            try:
                jpeg, stamp_ns = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            self.handovers.append((time.monotonic_ns(), stamp_ns))
            # (None, bytes) means "already encoded" -- see the client's CSI
            # source, which does the same with its hardware-encoded frames.
            yield None, jpeg

    def close(self):
        self._stop.set()


def _load_client(path):
    """
    Import the standalone client file from wherever it was deployed.

    It is one file with no package around it, on purpose -- that is what makes
    it deployable to the robot with a single ``scp`` -- so it is loaded by
    path rather than imported by name.
    """
    path = os.path.expanduser(path)
    if os.path.isdir(path):
        path = os.path.join(path, "robocam_client.py")
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"robocam_client.py not found at {path}. Deploy it from "
            "RoboCamStreamProcessing/link and set the client_path parameter."
        )
    spec = importlib.util.spec_from_file_location("robocam_client", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Deep3RClientNode(Node):
    """Frames out to the server, ``PointCloud2`` back in."""

    def __init__(self):
        super().__init__("mecanumbot_deep3r_client_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                # 127.0.0.1 by default, not the server's own address: the robot
                # cannot route to it. mecanumbot-deep3r-tunnel.service forwards
                # the port here. See RoboCamStreamProcessing/link/README.md.
                ("server", "tcp://127.0.0.1:5555"),
                ("client_path", "~/server/RoboCamStreamProcessing/link/robocam_client.py"),
                ("camera_topic", "camera/image_raw/compressed"),
                ("cloud_topic", "deep3r/points"),
                ("pose_topic", "deep3r/pose"),
                ("publish_pose", True),
                # The cloud is in CUT3R's own world frame, anchored on the first
                # frame of each map_id. It is NOT yet connected to the robot's
                # TF tree -- see this package's README.
                ("frame_id", "deep3r_world"),
                ("advertised_width", 1280),
                ("advertised_height", 720),
                ("advertised_fps", 15.0),
                ("max_inflight", 2),
                ("queue_depth", 2),
                ("log_every", 30),
            ],
        )
        gp = self.get_parameter
        self.frame_id = str(gp("frame_id").value)
        self.publish_pose = bool(gp("publish_pose").value)
        self.log_every = int(gp("log_every").value)

        self.source = RosImageSource(
            gp("advertised_width").value,
            gp("advertised_height").value,
            gp("advertised_fps").value,
            queue_depth=int(gp("queue_depth").value),
        )
        # Results are handed to the executor rather than published from the
        # client's thread, so every publish happens on one thread and the
        # node's clock is read from the same place it always is.
        self._results = queue.Queue(maxsize=8)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            CompressedImage, str(gp("camera_topic").value),
            self._on_image, sensor_qos,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(gp("cloud_topic").value), 1)
        self.pose_pub = (
            self.create_publisher(PoseStamped, str(gp("pose_topic").value), 10)
            if self.publish_pose else None
        )
        self.create_timer(0.02, self._drain_results)

        client_module = _load_client(str(gp("client_path").value))
        self.client = client_module.RoboCamClient(
            server=str(gp("server").value),
            client_id=f"mecanumbot-{os.uname().nodename}",
            max_inflight=int(gp("max_inflight").value),
            on_result=self._on_result,
        )
        self._frames_in = 0
        self._clouds_out = 0
        self._last_map_id = None
        self._thread = threading.Thread(target=self._run_client, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"deep3r client -> {gp('server').value}, frames from "
            f"{gp('camera_topic').value}, cloud on {gp('cloud_topic').value} "
            f"in frame '{self.frame_id}'"
        )

    # -- plumbing ------------------------------------------------------------

    def _run_client(self):
        try:
            self.client.run(self.source, status_every=0)
        except Exception as exc:  # pragma: no cover - reported, not raised
            self.get_logger().error(f"RoboCam client stopped: {exc}")

    def _on_image(self, msg):
        self._frames_in += 1
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec))
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self.source.submit(bytes(msg.data), stamp_ns)

    def _on_result(self, result):
        """Take one reply on the client's thread; do no ROS work here."""
        try:
            self._results.put_nowait(result)
        except queue.Full:
            pass  # a stale cloud is worth less than the next one

    # -- publishing ----------------------------------------------------------

    def _drain_results(self):
        while True:
            try:
                result = self._results.get_nowait()
            except queue.Empty:
                return
            try:
                self._publish(result)
            except cloud_codec.CloudFormatError as exc:
                self.get_logger().warn(f"undecodable cloud: {exc}")

    def _publish(self, result):
        data = result.get("data") or {}
        if not cloud_codec.has_cloud(data):
            return
        points, colors = cloud_codec.decode(data["cloud"])
        stamp = self._stamp_for(result)

        map_id = data.get("map_id")
        if map_id != self._last_map_id:
            if self._last_map_id is not None:
                # The world frame restarts with the map, so anything
                # accumulated against the old one is now meaningless.
                self.get_logger().warn(
                    f"map_id {self._last_map_id} -> {map_id} "
                    f"({data.get('reset_reason', 'reset')}): the world frame "
                    "restarted, discard whatever was accumulated before this"
                )
            self._last_map_id = map_id

        if points.shape[0]:
            self.cloud_pub.publish(self._to_msg(points, colors, stamp))
            self._clouds_out += 1

        pose = data.get("pose_c2w")
        if self.pose_pub is not None and pose:
            self.pose_pub.publish(self._to_pose(np.asarray(pose), stamp))

        if self.log_every and self._clouds_out % self.log_every == 1:
            check = data.get("scale_check") or {}
            self.get_logger().info(
                f"map {map_id} frame {data.get('frames_in_state')}: "
                f"{points.shape[0]} pts, infer {data.get('infer_ms')} ms, "
                f"in {self._frames_in} dropped {self.source.dropped}"
                + (f", lidar/cloud ratio {check['ratio']}" if check.get("ratio")
                   else "")
            )

    def _stamp_for(self, result):
        """
        Recover the ROS stamp of the image this reply belongs to.

        Falls back to now when the reply carries no ``t_send_ns`` or predates
        every hand-over this node remembers -- which happens for the first
        replies after a reconnect, and is better than refusing to publish.
        """
        sent = result.get("t_send_ns")
        if sent:
            best = None
            for handed, stamp_ns in self.source.handovers:
                if handed <= sent and (best is None or handed > best[0]):
                    best = (handed, stamp_ns)
            if best is not None:
                return rclpy.time.Time(nanoseconds=best[1]).to_msg()
        return self.get_clock().now().to_msg()

    def _to_msg(self, points, colors, stamp):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        packed = cloud_codec.to_xyzrgb(points, colors)
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = packed.tobytes()
        return msg

    def _to_pose(self, c2w, stamp):
        msg = PoseStamped()
        msg.header = Header(stamp=stamp, frame_id=self.frame_id)
        msg.pose.position.x = float(c2w[0, 3])
        msg.pose.position.y = float(c2w[1, 3])
        msg.pose.position.z = float(c2w[2, 3])
        qw, qx, qy, qz = quat_from_matrix(c2w[:3, :3])
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        return msg

    def destroy_node(self):
        self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Deep3RClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# mecanumbot_deep3r

Off-board 3D reconstruction. The robot's camera frames go to the cluster, where
CUT3R turns them into a metric point cloud, and the cloud comes back and is
republished as `sensor_msgs/PointCloud2`.

This is the robot half of the "Deep3R" box in the system diagram. The model
half is the `deep3r` processor in `RoboCamStreamProcessing`; nothing in this
package imports torch, and the node runs on the Orin with numpy and pyzmq.

```text
  camera_stream ──► /camera/image_raw/compressed ──► this node
                                                       │  JPEG, forwarded as-is
                                                       ▼
                                              robocam_client (ZeroMQ)
                                                       │  tcp://127.0.0.1:5555
                                                       ▼  (forward SSH tunnel)
                                              nipg36: deep3r / CUT3R
                                                       │
  /mecanumbot/deep3r/points ◄── PointCloud2 ◄──────────┘
  /mecanumbot/deep3r/pose   ◄── PoseStamped
```

## Node: mecanumbot_deep3r_client_node

### Subscribers

| Topic | Data type | Processing |
| --- | --- | --- |
| `camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Forwarded to the server **as the JPEG it already is** — no decode/re-encode on the robot. |

### Publishers

| Topic | Data type | Function |
| --- | --- | --- |
| `deep3r/points` | `sensor_msgs/msg/PointCloud2` | The returned cloud: `xyz` + packed `rgb`, metres. |
| `deep3r/pose` | `geometry_msgs/msg/PoseStamped` | CUT3R's camera pose for that frame, same frame as the cloud. Disable with `publish_pose`. |

### Parameters

| Parameter | Default | Function |
| --- | --- | --- |
| `server` | `tcp://127.0.0.1:5555` | Local end of the forward tunnel — **not** the server's own address. |
| `client_path` | `~/robocam_client.py` | The deployed standalone client; a directory is also accepted. |
| `camera_topic` | `camera/image_raw/compressed` | Where frames come from. |
| `cloud_topic` | `deep3r/points` | Point cloud output. |
| `pose_topic` | `deep3r/pose` | Camera pose output. |
| `publish_pose` | `true` | Publish the pose alongside the cloud. |
| `frame_id` | `deep3r_world` | Frame the cloud and pose are stamped in. See the caveat below. |
| `advertised_width` / `_height` / `_fps` | `1280` / `720` / `15.0` | Metadata the client announces. The server reports what it actually decoded, so these need not be exact. |
| `max_inflight` | `2` | Frames awaiting a reply. |
| `queue_depth` | `2` | Frames buffered between the subscription and the client. |
| `log_every` | `30` | Log one line every N clouds; 0 disables. |

## Why it subscribes instead of opening the camera

`mecanumbot_camera_stream` already owns the camera device on the Orin. A second
process opening `/dev/video0` would either fail or take it away. Consuming the
compressed topic also means the JPEG is already encoded, so frames are
forwarded untouched rather than decoded and re-encoded on the robot's CPU.

## Timestamps

The cloud is stamped with when the **picture was taken**, not when the reply
arrived. A round trip is a couple of hundred milliseconds; a cloud stamped on
arrival is placed where the robot is *now* rather than where it was looking,
smearing the map by exactly the distance travelled in between.

The client carries no per-frame metadata, so the correlation runs on its clock.
Every reply echoes `t_send_ns`, a `time.monotonic_ns()` reading taken on this
same machine, and the node records that clock when it hands each frame over.
Matching a reply to the most recent hand-over at or before its `t_send_ns`
recovers the right image — and does so without assuming every frame handed over
was sent, which matters because the client drops frames at the source under
load. Anything based on counting frames would drift silently the first time it
did.

## The frame is not connected to TF yet

`frame_id` defaults to `deep3r_world`, and **nothing publishes a transform for
it.** The cloud arrives in CUT3R's own world frame, anchored on the first frame
of each `map_id` and drifting independently of `odom`. Feeding that to a
costmap means two odometry estimates fighting, and every reset teleports the
map.

The fix is on the server, not here: return the per-frame points in the
*camera* frame (CUT3R's `pts3d_in_self_view`, skipping the pose transform) and
let the existing `head_link → base_footprint → odom → map` chain place them,
with the costmap accumulating as it already does for the LiDAR. Until that
option exists, this node is useful for inspecting the reconstruction in RViz
against a static transform, not for navigation.

`map_id` changes are logged as a warning for the same reason: the world frame
restarts with them, so anything accumulated against the previous one is void.

## Running

The tunnel has to be up first — see `RoboCamStreamProcessing/link/README.md`.

```bash
# once, on the robot
scp -P 10113 csengehubay@nipg36.inf.elte.hu:~/mecanumbot_repos/RoboCamStreamProcessing/link/robocam_client.py ~/
pip3 install pyzmq

ros2 launch mecanumbot_deep3r deep3r.launch.py
ros2 topic hz /mecanumbot/deep3r/points
```

Expect roughly 6 Hz with the 512 checkpoint on an RTX 3090, less on nipg36's
TITAN RTX. `log_every` prints the point count, the server's inference time, and
— when the LiDAR is attached — the ratio between the cloud's near depth and the
LiDAR's forward range, which is the cheapest check that CUT3R's metric scale is
actually metric.

## Tests

```bash
PYTHONPATH=. python3 -m pytest test/test_cloud.py test/test_geometry.py -q
```

Both run without ROS, without a server and without a GPU: the wire decoder and
the rotation helper are plain numpy, and they are where a mistake produces a
plausible cloud in the wrong place rather than an error.

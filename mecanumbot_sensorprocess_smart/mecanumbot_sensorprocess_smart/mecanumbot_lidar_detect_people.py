#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.interpolate import interp1d
from scipy.ndimage import median_filter
import tf2_geometry_msgs
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter
from dr_spaam.detector import Detector
from ament_index_python.packages import get_package_share_directory 
from rclpy.executors import MultiThreadedExecutor
import torch



# ---- 1. Determine Device Dynamically ----
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ---- 2. Dynamic Monkey-patch for torch.load ----
_original_torch_load = torch.load

def processor_load(path, *args, **kwargs):
    # Map to the dynamically detected device (cuda or cpu)
    return _original_torch_load(path, map_location=DEVICE)

class Track:
    """Represents a single tracked person."""
    def __init__(self, detection, track_id):
        self.track_id = track_id
        # State: [x, y, vx, vy] | Measurement: [x, y]
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([detection[0], detection[1], 0.0, 0.0]).reshape(4, 1)
        
        # State transition matrix (assuming ~10Hz LiDAR, dt = 0.1)
        dt = 0.1 
        self.kf.F = np.array([[1, 0, dt, 0],
                              [0, 1, 0, dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        
        # Measurement function (we only measure x, y)
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        
        # Tuning matrices - adjust R if your LiDAR is extremely noisy
        self.kf.P *= 10.0      # Initial uncertainty
        self.kf.R *= 0.5       # Measurement noise (LiDAR accuracy)
        self.kf.Q *= 0.01      # Process noise (how erratically people move)
        
        self.time_since_update = 0
        self.hits = 1
        self.has_moved = False
        self.speed_thresh = 0.1  # Minimum m/s to be considered "human"

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.kf.x[:2].reshape(-1)

    def update(self, detection):
        self.kf.update(detection.reshape(2, 1))
        self.time_since_update = 0
        self.hits += 1
        
        # Extract velocity from the Kalman state [x, y, vx, vy]
        vx = self.kf.x[2, 0]
        vy = self.kf.x[3, 0]
        speed = np.hypot(vx, vy)
        
        # If the track ever moves faster than the threshold, lock it as a valid moving object
        if speed > self.speed_thresh:
            self.has_moved = True

class MultiObjectTracker:
    """Manages all active tracks and matches new detections."""
    def __init__(self, max_distance=0.5, max_missed_frames=4, min_hits=2):
        self.max_distance = max_distance        # Max meters a person can move between frames
        self.max_missed_frames = max_missed_frames # Frames to keep track alive without seeing them
        self.min_hits = min_hits                # Frames a track must exist before being published
        self.tracks = []
        self.next_id = 0

    def update(self, detections):
        """
        Takes raw [N, 2] detections, updates tracks, 
        and returns validated [M, 2] tracked positions.
        """
        # 1. Predict next positions for all tracks
        predicted_positions = np.array([track.predict() for track in self.tracks])
        
        matched_indices = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(range(len(self.tracks)))

        # 2. Match detections to tracks using Hungarian algorithm
        if len(self.tracks) > 0 and len(detections) > 0:
            cost_matrix = np.linalg.norm(predicted_positions[:, None, :] - detections[None, :, :], axis=2)
            track_indices, det_indices = linear_sum_assignment(cost_matrix)

            for t_idx, d_idx in zip(track_indices, det_indices):
                if cost_matrix[t_idx, d_idx] < self.max_distance:
                    matched_indices.append((t_idx, d_idx))
                    unmatched_detections.remove(d_idx)
                    unmatched_tracks.remove(t_idx)

        # 3. Update matched tracks with new measurements
        for t_idx, d_idx in matched_indices:
            self.tracks[t_idx].update(detections[d_idx])

        # 4. Create new tracks for unmatched detections
        for d_idx in unmatched_detections:
            self.tracks.append(Track(detections[d_idx], self.next_id))
            self.next_id += 1

        # 5. Remove dead tracks (missed for too many frames)
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_missed_frames]

        # 6. Return only "mature" tracks to avoid publishing random 1-frame noise flashes
        valid_positions = []
        valid_positions = []
        for t in self.tracks:
            # Must exist for a few frames AND must have moved at least once
            if t.hits >= self.min_hits and t.has_moved:
                valid_positions.append(t.kf.x[:2].reshape(-1))

        return np.array(valid_positions) if len(valid_positions) > 0 else np.empty((0, 2))




class DrSpaamNode(Node):
    """ROS2 node performing people detection using DR-SPAAM on 2D LiDAR."""
    
    def __init__(self):
        super().__init__("mecanumbot_lidar_detect_people")
        #Set processor type
        torch.load = processor_load
        self.get_logger().info(f'DEVICE: {DEVICE}')
        # ---- Declare parameters ----
        self.declare_parameter("weight_file", "dr_spaam_5_on_frog.pth")
        self.declare_parameter("conf_thresh", 0.6)
        self.declare_parameter("stride", 1)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("detections_topic", "dets")
        self.declare_parameter("rviz_topic", "dets_marker")
        self.declare_parameter("leading_mode",True)

        self.weight_file = self.get_parameter("weight_file").value
        self.conf_thresh = self.get_parameter("conf_thresh").value
        self.stride = self.get_parameter("stride").value
        self.leading_mode = self.get_parameter("leading_mode")

        pkg_share = get_package_share_directory('mecanumbot_sensorprocess_smart')
        weight_file = self.get_parameter("weight_file").value
        weight_path = os.path.join(pkg_share, 'models', weight_file)
        self.pose_out = None
        self.last_pose_out = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)

        if not os.path.isfile(weight_path):
            self.get_logger().error(f"DR-SPAAM model file not found: {weight_path}")
            raise FileNotFoundError(weight_path)
        else:
            self.get_logger().info(f'Weight path set to: {weight_path}')
        self.detector = Detector(
            model_name="DR-SPAAM",
            ckpt_file=weight_path,
            gpu=False,   # CPU-compatible
            stride=self.stride
        )
        # Initialize the tracker here
        self.tracker = MultiObjectTracker(max_distance=0.5, max_missed_frames=3, min_hits=2)
        # ---- Publishers ----
        self.dets_pub = self.create_publisher(PoseArray,
                                              self.get_parameter("detections_topic").value,
                                              10)
        
        self.rviz_pub = self.create_publisher(Marker,
                                              self.get_parameter("rviz_topic").value,
                                              10)
        if self.leading_mode:
            self.subject_pub = self.create_publisher(PoseStamped,
                                                    "subject_pose",
                                                    10)
        # ---- Subscriber ----

        self.scan_sub = self.create_subscription(
                                                    LaserScan,
                                                    'scan',
                                                    self.scan_callback,
                                                    qos_profile_sensor_data
                                                )

        self.get_logger().info("DR-SPAAM ROS2 detector node started.")


    def scan_callback(self, msg: LaserScan,expected_points = 240):

        if not self.detector.laser_spec_set():
            self.detector.set_laser_spec(angle_inc=0.026, num_pts=expected_points)

        scan = np.array(msg.ranges)
        scan = preprocess_lidar(scan, target_len=expected_points, max_range=10.0)
        dets_xy, dets_cls, _ = self.detector(scan)

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)

        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]
        dets_xy = -1 * dets_xy
        
        # Filter the raw network detections through the Kalman tracker
        tracked_xy = self.tracker.update(dets_xy)

        # Publish PoseArray using the TRACKED positions, not the raw ones
        dets_msg = self._dets_to_pose_array(tracked_xy) 
        dets_msg.header = msg.header
        self.dets_pub.publish(dets_msg)

        if self.leading_mode and len(dets_msg.poses) > 0:
            self.pose_out = self._parse_subject_pose(dets_msg)

        if self.pose_out is not None:
            self.last_pose_out = self.pose_out
            self.subject_pub.publish(self.pose_out)
        else:
            if self.last_pose_out is not None:
                self.subject_pub.publish(self.last_pose_out)
            
        # Publish RViz marker using the TRACKED positions
        marker_msg = self._dets_to_marker(tracked_xy) 
        marker_msg.header = msg.header
        self.rviz_pub.publish(marker_msg)

    def _parse_subject_pose(self,dets_msg):
        ps_msg = Pose()
        ps_msg.position.x = dets_msg.poses[0].position.x
        ps_msg.position.y = dets_msg.poses[0].position.y
        ps_msg.position.z = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                                                    'map',
                                                    'mecanumbot/base_scan',
                                                    rclpy.time.Time(),  # Latest available
                                                )
            pose_out = PoseStamped()
            pose_out.header.stamp = self.get_clock().now().to_msg()
            pose_out.header.frame_id = 'map'
            pose_out.pose = tf2_geometry_msgs.do_transform_pose(ps_msg, transform)
            return pose_out
        except Exception as e:
            self.get_logger().error(f"TF transform error: {e}")
            return None
        
    def _dets_to_pose_array(self, dets_xy):
        msg = PoseArray()
        for xy in dets_xy:
            p = Pose()
            p.position.x = xy[1]
            p.position.y = xy[0]
            p.position.z = 0.0
            msg.poses.append(p)

        return msg


    def _dets_to_marker(self, dets_xy):

        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = "dr_spaam"
        msg.id = 0
        msg.type = Marker.LINE_LIST
        msg.scale.x = 0.03
        msg.color.r = 1.0
        msg.color.a = 1.0
        
        r = 0.2
        ang = np.linspace(0, 2*np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        for d_xy in dets_xy:
            for i in range(len(xy_offsets) - 1):
                p0 = Point()
                p1 = Point()

                p0.x = d_xy[1] + xy_offsets[i][0]
                p0.y = d_xy[0] + xy_offsets[i][1]

                p1.x = d_xy[1] + xy_offsets[i+1][0]
                p1.y = d_xy[0] + xy_offsets[i+1][1]

                msg.points.append(p0)
                msg.points.append(p1)

        return msg

def preprocess_lidar(scan, target_len=240, max_range=10.0):
    """
    Cleans and resizes LiDAR data using depth-safe mathematics 
    to prevent mid-air artifacts.
    """
    scan = np.array(scan, dtype=float)

    # Handle Invalid Points Safely
    # Zeros, infs, and NaNs usually mean "no return", the light emmitted did not arrive back to the sensor.
    invalid = (scan <= 0.01) | np.isinf(scan) | np.isnan(scan)
    scan[invalid] = max_range

    # Apply a Median Filter
    # A size=3 median filter removes "salt and pepper" noise
    scan = median_filter(scan, size=3)

    # Resize using Nearest Neighbor - snaps interpolated points to the nearest physical object,
    if len(scan) != target_len:
        x_old = np.linspace(0, 1, len(scan))
        x_new = np.linspace(0, 1, target_len)
        
        # kind='nearest' ensures no fake depth gradients are created
        interpolator = interp1d(x_old, scan, kind='nearest')
        scan = interpolator(x_new)

    return scan

def main(args=None):
    rclpy.init(args=args)
    node = DrSpaamNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import os
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

# --- NEW: Imported binary_dilation for efficient map inflation ---
from scipy.interpolate import interp1d
from scipy.ndimage import median_filter, binary_dilation 
from scipy.optimize import linear_sum_assignment
from filterpy.kalman import KalmanFilter

import torch
from dr_spaam.detector import Detector
from ament_index_python.packages import get_package_share_directory 

# ---- 1. Determine Device Dynamically ----
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# ---- 2. Dynamic Monkey-patch for torch.load ----
_original_torch_load = torch.load

def processor_load(path, *args, **kwargs):
    return _original_torch_load(path, map_location=DEVICE)

class Track:
    """Represents a single tracked person."""
    def __init__(self, detection, track_id):
        self.track_id = track_id
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([detection[0], detection[1], 0.0, 0.0]).reshape(4, 1)
        
        dt = 0.1 
        self.kf.F = np.array([[1, 0, dt, 0],
                              [0, 1, 0, dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        
        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        
        self.kf.P *= 10.0      
        self.kf.R *= 0.5       
        self.kf.Q *= 0.01      
        
        self.time_since_update = 0
        self.hits = 1
        self.has_moved = False
        self.speed_thresh = 0.1  

    def predict(self):
        self.kf.predict()
        self.time_since_update += 1
        return self.kf.x[:2].reshape(-1)

    def update(self, detection):
        self.kf.update(detection.reshape(2, 1))
        self.time_since_update = 0
        self.hits += 1
        
        vx = self.kf.x[2, 0]
        vy = self.kf.x[3, 0]
        speed = np.hypot(vx, vy)
        
        if speed > self.speed_thresh:
            self.has_moved = True

class MultiObjectTracker:
    """Manages all active tracks and matches new detections."""
    def __init__(self, max_distance=0.5, max_missed_frames=4, min_hits=2):
        self.max_distance = max_distance        
        self.max_missed_frames = max_missed_frames 
        self.min_hits = min_hits                
        self.tracks = []
        self.next_id = 0

    def update(self, detections):
        predicted_positions = np.array([track.predict() for track in self.tracks])
        
        matched_indices = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(range(len(self.tracks)))

        if len(self.tracks) > 0 and len(detections) > 0:
            cost_matrix = np.linalg.norm(predicted_positions[:, None, :] - detections[None, :, :], axis=2)
            track_indices, det_indices = linear_sum_assignment(cost_matrix)

            for t_idx, d_idx in zip(track_indices, det_indices):
                if cost_matrix[t_idx, d_idx] < self.max_distance:
                    matched_indices.append((t_idx, d_idx))
                    unmatched_detections.remove(d_idx)
                    unmatched_tracks.remove(t_idx)

        for t_idx, d_idx in matched_indices:
            self.tracks[t_idx].update(detections[d_idx])

        for d_idx in unmatched_detections:
            self.tracks.append(Track(detections[d_idx], self.next_id))
            self.next_id += 1

        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_missed_frames]

        valid_positions = []
        for t in self.tracks:
            if t.hits >= self.min_hits and t.has_moved:
                valid_positions.append(t.kf.x[:2].reshape(-1))

        return np.array(valid_positions) if len(valid_positions) > 0 else np.empty((0, 2))


class DrSpaamNode(Node):
    """ROS2 node performing people detection using DR-SPAAM on 2D LiDAR."""
    
    def __init__(self):
        super().__init__("mecanumbot_lidar_detect_people")
        torch.load = processor_load
        self.get_logger().info(f'DEVICE: {DEVICE}')
        
        self.declare_parameter("weight_file", "dr_spaam_5_on_frog.pth")
        self.declare_parameter("conf_thresh", 0.6)
        self.declare_parameter("stride", 1)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("detections_topic", "dets")
        self.declare_parameter("rviz_topic", "dets_marker")
        self.declare_parameter("leading_mode", True)
        self.declare_parameter("obstacle_exclusion_radius", 0.70) 
        self.declare_parameter("detection_frame", "base_scan")

        self.weight_file = self.get_parameter("weight_file").value
        self.conf_thresh = self.get_parameter("conf_thresh").value
        self.stride = self.get_parameter("stride").value
        self.leading_mode = self.get_parameter("leading_mode").value
        self.exclusion_radius = self.get_parameter("obstacle_exclusion_radius").value
        self.detection_frame = str(self.get_parameter("detection_frame").value)

        if self.detection_frame not in ("base_scan", "map"):
            self.get_logger().warn(f"Invalid detection_frame '{self.detection_frame}', defaulting to 'base_scan'.")
            self.detection_frame = "base_scan"

        pkg_share = get_package_share_directory('mecanumbot_sensorprocess_smart')
        weight_path = os.path.join(pkg_share, 'models', self.weight_file)
        self.pose_out = None
        self.last_pose_out = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if not os.path.isfile(weight_path):
            self.get_logger().error(f"DR-SPAAM model file not found: {weight_path}")
            raise FileNotFoundError(weight_path)
            
        self.detector = Detector(
            model_name="DR-SPAAM",
            ckpt_file=weight_path,
            gpu=False,
            stride=self.stride
        )
        
        self.tracker = MultiObjectTracker(max_distance=0.5, max_missed_frames=3, min_hits=2)
        
        # ---- Map State Data ----
        self.map_data = None
        self.extended_map = None # --- NEW: Holds the inflated occupancy boolean mask ---
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0

        self.dets_pub = self.create_publisher(PoseArray, self.get_parameter("detections_topic").value, 10)
        self.rviz_pub = self.create_publisher(Marker, self.get_parameter("rviz_topic").value, 10)
        
        if self.leading_mode:
            self.subject_pub = self.create_publisher(PoseStamped, "subject_pose", 10)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.get_logger().info("DR-SPAAM ROS2 detector node started.")

    def map_callback(self, msg: OccupancyGrid):
        """Updates internal static map grid and generates the obstacle exclusion zone."""
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        raw_map = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.map_data = raw_map
        
        # --- NEW: Build the Obstacle Exclusion Zone ---
        # Consider map cells with probability > 50 to be static obstacles
        obstacles = raw_map > 65
        
        # Calculate pixel radius for dilation
        radius_px = int(math.ceil(self.exclusion_radius / self.map_resolution))
        
        if radius_px > 0:
            # Create a circular footprint to ensure accurate radial dilation
            y, x = np.ogrid[-radius_px:radius_px+1, -radius_px:radius_px+1]
            circular_footprint = x**2 + y**2 <= radius_px**2
            
            # Dilate the obstacles. Any True pixel in extended_map is a "no-detection zone"
            self.extended_map = binary_dilation(obstacles, structure=circular_footprint)
        else:
            self.extended_map = obstacles

        self.get_logger().info(f"Occupancy map received. Extended exclusion zone built (Radius: {radius_px}px).")

    # --- NEW: Filter helper function ---
    def _filter_detections_by_map(self, dets_xy, sensor_frame):
        """Discards detections that fall within the inflated map occupancy."""
        if self.extended_map is None:
            self.get_logger().warn("Map not loaded yet. Skipping map-based filtering.", throttle_duration_sec=2.0)
            return dets_xy 
            
        try:
            # Get the real-time transform from map to the LiDAR sensor
            t = self.tf_buffer.lookup_transform(
                'map',
                sensor_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF error during map filtering: {e}", throttle_duration_sec=2.0)
            return dets_xy # Safely return all detections if TF fails

        # Extract translation
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        
        # Convert Quaternion to Yaw angle (Euler)
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        valid_dets = []
        for xy in dets_xy:
            # Per your packing format: x is xy[1], y is xy[0]
            local_x = xy[1]
            local_y = xy[0]
            
            # 1. Transform local sensor coordinates into global map coordinates
            global_x = tx + (local_x * cos_yaw) - (local_y * sin_yaw)
            global_y = ty + (local_x * sin_yaw) + (local_y * cos_yaw)
            
            # 2. Convert physical global coordinates to map pixel indices
            px = int((global_x - self.map_origin_x) / self.map_resolution)
            py = int((global_y - self.map_origin_y) / self.map_resolution)
            
            # 3. Check if index is within map bounds and inside an exclusion zone
            if 0 <= px < self.map_width and 0 <= py < self.map_height:
                if self.extended_map[py, px]:
                    continue # It's a wall/static object! Skip adding it.
            
            valid_dets.append(xy) # Keep it if it's safe
            
        return np.array(valid_dets) if len(valid_dets) > 0 else np.empty((0, 2))


    def scan_callback(self, msg: LaserScan, expected_points=240):
        if not self.detector.laser_spec_set():
            self.detector.set_laser_spec(angle_inc=0.026, num_pts=expected_points)

        scan = np.array(msg.ranges)
        scan = preprocess_lidar(scan, target_len=expected_points, max_range=10.0)
        dets_xy, dets_cls, _ = self.detector(scan)

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]
        dets_xy = -1 * dets_xy
       

        # ----------------------------------------
        # --- NEW: Apply the Static Map Filter --
        dets_xy = self._filter_detections_by_map(dets_xy, msg.header.frame_id)
        self.get_logger().info(f"Map: {self.map_data.shape if self.map_data is not None else 'None'}, {np.max(self.map_data) if self.map_data is not None else 'None'}, Dets after map filter: {dets_xy.shape[0]}")
        # ----------------------------------------

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
            
        marker_msg = self._dets_to_marker(tracked_xy) 
        marker_msg.header = msg.header
        self.rviz_pub.publish(marker_msg)

    def _parse_subject_pose(self, dets_msg):
        ps_msg = Pose()
        ps_msg.position.x = dets_msg.poses[0].position.x
        ps_msg.position.y = dets_msg.poses[0].position.y
        ps_msg.position.z = 0.0
        try:
            transform = self.tf_buffer.lookup_transform(
                                                    'map',
                                                    'mecanumbot/base_scan',
                                                    rclpy.time.Time(),  
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
    scan = np.array(scan, dtype=float)
    invalid = (scan <= 0.01) | np.isinf(scan) | np.isnan(scan)
    scan[invalid] = max_range
    scan = median_filter(scan, size=3)

    if len(scan) != target_len:
        x_old = np.linspace(0, 1, len(scan))
        x_new = np.linspace(0, 1, target_len)
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
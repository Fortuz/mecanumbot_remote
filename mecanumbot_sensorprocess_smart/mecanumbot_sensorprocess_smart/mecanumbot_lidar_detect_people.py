#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker
import torch



# Monkey-patch torch.load globally:
_original_torch_load = torch.load

def cpu_load(path, *args, **kwargs):
    return _original_torch_load(path, map_location=torch.device("cpu"))

torch.load = cpu_load

from dr_spaam.detector import Detector
from ament_index_python.packages import get_package_share_directory


class DrSpaamNode(Node):
    """ROS2 node performing people detection using DR-SPAAM on 2D LiDAR."""
    
    def __init__(self):
        super().__init__("mecanumbot_lidar_detect_people")

        # ---- Declare parameters ----
        self.declare_parameter("weight_file", "dr_spaam_5_on_frog.pth")
        self.declare_parameter("conf_thresh", 0.35)
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
        # ---- Publishers ----
        self.dets_pub = self.create_publisher(PoseArray,
                                              self.get_parameter("detections_topic").value,
                                              10)
        
        self.rviz_pub = self.create_publisher(Marker,
                                              self.get_parameter("rviz_topic").value,
                                              10)
        if self.leading_mode:
            self.subject_pub = self.create_publisher(Pose,
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
        #self.get_logger().info(f"############ Raw Scan size: {len(scan)}##########")
        #self.get_logger().info(f'Scan params: {msg.angle_min}, {msg.angle_increment}, {msg.angle_max}')
        scan = fill_with_neighbor_average(scan)
        scan = interpolate_scan(scan,expected_points)
        #self.get_logger().info(f"############ Scan size: {scan.shape}##########")
        dets_xy, dets_cls, _ = self.detector(scan) # fails if scan size does not match the first recieved

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)

        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]
        dets_xy = -1 * dets_xy

        # Publish PoseArray
        dets_msg = self._dets_to_pose_array(dets_xy)
        dets_msg.header = msg.header
        self.dets_pub.publish(dets_msg)
        if self.leading_mode and i == 0:
                ps_msg = PoseStamped()
                ps_msg.header = msg.header
                ps_msg.pose = dets_msg.poses[0]
                self.subject_pub.publish(ps_msg)
        # Publish RViz marker
        marker_msg = self._dets_to_marker(dets_xy)
        marker_msg.header = msg.header
        self.rviz_pub.publish(marker_msg)


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

def interpolate_scan(scan, target_len=250):
    x_old = np.linspace(0, 1, len(scan))
    x_new = np.linspace(0, 1, target_len)
    return np.interp(x_new, x_old, scan)

def fill_with_neighbor_average(scan):
    scan = np.array(scan, dtype=float)

    # Mark invalid values
    invalid = (scan == 0.0) | np.isinf(scan) | np.isnan(scan)

    # Copy for output
    filled = scan.copy()

    for i in np.where(invalid)[0]:
        # Search left
        left = i - 1
        while left >= 0 and invalid[left]:
            left -= 1

        # Search right
        right = i + 1
        while right < len(scan) and invalid[right]:
            right += 1

        left_val = scan[left] if left >= 0 else None
        right_val = scan[right] if right < len(scan) else None

        # Compute replacement
        if left_val is not None and right_val is not None:
            filled[i] = (left_val + right_val) / 2.0
        elif left_val is not None:
            filled[i] = left_val
        elif right_val is not None:
            filled[i] = right_val
        else:
            # All values invalid — fallback to 0 or any default
            filled[i] = 0.0

    return filled

def main(args=None):
    rclpy.init(args=args)
    node = DrSpaamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
# Smart sensor processer nodes for the mecanumbot package

In this folder, nodes are situated which extract information from the mecanumbot's on-board sensors

## Node: mecanumbot_lidar_detect_people

### Publishers

| Topic                                    | Data type                     | Function                                                                                           |
| ---------------------------------------- | ----------------------------- | -------------------------------------------------------------------------------------------------- |
| dets (configurable by detections_topic)  | geometry_msgs/msg/PoseArray   | Publishes filtered and tracked people detections as 2D poses.                                      |
| dets_marker (configurable by rviz_topic) | visualization_msgs/msg/Marker | Publishes RViz LINE_LIST circles around tracked detections.                                        |
| subject_pose                             | geometry_msgs/msg/PoseStamped | Publishes selected leading subject pose transformed into map frame (when leading mode is enabled). |

### Subscribers

| Topic | Data type                 | Processing                                                                                                                 |
| ----- | ------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| scan  | sensor_msgs/msg/LaserScan | Preprocesses scan, runs DR-SPAAM inference, confidence filtering, multi-object tracking, then publishes detection outputs. |

### Behavior

- Loads DR-SPAAM weights from package share models folder.
- Detects CPU/GPU capability via torch and maps model loading accordingly.
- Uses nearest-neighbor interpolation and median filtering for robust scan preprocessing.
- Uses a Kalman filter + Hungarian assignment tracker to suppress one-frame noise and keep stable tracked targets.
- Uses TF lookup and pose transform from mecanumbot/base_scan to map for subject_pose output.

### External dependency notes

- DR-SPAAM package: https://github.com/VisualComputingInstitute/DR-SPAAM-Detector
- Dataset reference: https://robotics.upo.es/datasets/frog/laser2d_people/

## File functions

| File or folder                                                   | Function                                                      |
| ---------------------------------------------------------------- | ------------------------------------------------------------- |
| mecanumbot_sensorprocess_smart/mecanumbot_lidar_detect_people.py | Main detection node and tracking pipeline.                    |
| launch/mecanumbot_peopledetect.launch.py                         | Launches the detection node with parameter file.              |
| param/lidar_peopledetect_config.yaml                             | Runtime ROS parameters for node topics and thresholds.        |
| models/dr_spaam_5_on_frog.pth                                    | DR-SPAAM pretrained weights file used for inference.          |
| config/lidar_peopledetect_config.yaml                            | Additional packaged config copy for deployment compatibility. |

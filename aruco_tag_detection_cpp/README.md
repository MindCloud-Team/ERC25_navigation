# Enhanced ArUco Tag Detection with Multi-Camera and LiDAR Fusion

This package provides an enhanced ArUco tag detector that:
- Uses all 4 cameras (front, back, left, right) simultaneously
- Integrates LiDAR data to correct distance measurements
- Provides accurate 3D pose estimation in the robot's base_link frame

## Features

- **Multi-Camera Support**: Detects ArUco markers from 4 cameras simultaneously
- **LiDAR Distance Correction**: Uses LiDAR point cloud to correct RGB camera distance estimates
- **Sensor Fusion**: Combines detections from multiple cameras for robust tracking
- **TF2 Integration**: Transforms all poses to a common base_link frame
- **Configurable Parameters**: Easy configuration of marker size, camera topics, etc.

## Topics

### Subscribed Topics:
- `/front_cam/zed_node/rgb/image_rect_color` - Front camera RGB image
- `/back_cam/zed_node/rgb/image_rect_color` - Back camera RGB image  
- `/left_cam/zed_node/rgb/image_rect_color` - Left camera RGB image
- `/right_cam/zed_node/rgb/image_rect_color` - Right camera RGB image
- `/lidar/velodyne_points` - LiDAR point cloud for distance correction

### Published Topics:
- `aruco_poses` (geometry_msgs/PoseArray) - Detected marker poses in base_link frame

## Parameters

- `marker_size` (double, default: 0.05) - Size of ArUco markers in meters
- `camera_topics` (string array) - List of camera image topics to subscribe to
- `lidar_topic` (string, default: "/lidar/velodyne_points") - LiDAR point cloud topic

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_tag_detection_cpp
source install/setup.bash
```

## Usage

### Basic Usage:
```bash
ros2 run aruco_tag_detection_cpp aruco_tag_detector
```

### Using Launch File:
```bash
ros2 launch aruco_tag_detection_cpp multi_camera_aruco_detector.launch.py
```

### With Custom Parameters:
```bash
ros2 launch aruco_tag_detection_cpp multi_camera_aruco_detector.launch.py marker_size:=0.1
```

## Configuration

### Camera Calibration
Edit `config/camera_calibration.yaml` with your actual camera calibration parameters.
You can obtain these using:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/your_camera/image_raw camera:=/your_camera
```

### Transform Setup
Ensure your TF tree has the following transforms:
- `base_link` → `front_camera_optical_frame`
- `base_link` → `back_camera_optical_frame`
- `base_link` → `left_camera_optical_frame`  
- `base_link` → `right_camera_optical_frame`

## Algorithm Details

1. **Multi-Camera Detection**: Each camera independently detects ArUco markers
2. **Pose Estimation**: Uses camera intrinsics to estimate 3D pose of each marker
3. **Transform to Base Frame**: Converts poses from camera frames to base_link using TF2
4. **LiDAR Distance Correction**: 
   - Searches for LiDAR points near the estimated marker position
   - Corrects the distance using the nearest LiDAR measurement
   - Scales the position vector while preserving direction
5. **Sensor Fusion**: Combines detections from multiple cameras, removing duplicates

## Dependencies

- rclcpp
- sensor_msgs
- geometry_msgs
- cv_bridge
- OpenCV (with ArUco support)
- PCL (Point Cloud Library)
- tf2 and tf2_ros
- message_filters

## Troubleshooting

### No ArUco Detections:
- Check camera topics are publishing images
- Verify marker size parameter matches your physical markers
- Ensure good lighting conditions
- Check that markers are visible and not occluded

### No LiDAR Correction:
- Verify LiDAR topic is publishing point clouds
- Check that markers are within LiDAR range and field of view
- Adjust `search_radius` parameter if needed

### Transform Errors:
- Ensure all camera transforms are published to TF
- Check frame names match between your robot and the detector
- Verify transform timestamps are recent

## Example Output

```
[INFO] [multi_camera_aruco_detector]: Detected 2 markers from camera: /front_cam/zed_node/rgb/image_rect_color
[INFO] [multi_camera_aruco_detector]: Corrected marker 1 distance from 2.34 to 2.28 meters
[INFO] [multi_camera_aruco_detector]: Published 2 marker poses
```

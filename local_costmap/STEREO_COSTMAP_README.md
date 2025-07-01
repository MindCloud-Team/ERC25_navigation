# Stereo Camera Local Costmap

This package provides a stereo camera-based local costmap generation system for 3D terrain navigation, capable of handling rocks, walls, hills, and uneven terrain.

## Features

- **3D Terrain Analysis**: Uses stereo vision to create depth maps and analyze terrain height
- **Traversability Assessment**: Evaluates slopes, obstacles, and terrain roughness
- **Real-time Processing**: Generates costmaps at 5Hz for real-time navigation
- **Flexible Configuration**: Adjustable parameters for different terrain types and robot capabilities

## Node: stereo_costmap_node

### Subscribed Topics

- `/stereo/left/image_raw` (sensor_msgs/Image): Left stereo camera image
- `/stereo/right/image_raw` (sensor_msgs/Image): Right stereo camera image  
- `/stereo/left/camera_info` (sensor_msgs/CameraInfo): Left camera calibration info
- `/stereo/right/camera_info` (sensor_msgs/CameraInfo): Right camera calibration info

### Published Topics

- `/stereo_costmap` (nav_msgs/OccupancyGrid): 2D costmap for navigation
- `/stereo_pointcloud` (sensor_msgs/PointCloud2): 3D point cloud for visualization

### Parameters

- `grid_size` (double, default: 6.0): Size of costmap in meters (square grid)
- `resolution` (double, default: 0.1): Resolution in meters per cell
- `max_height_diff` (double, default: 0.3): Max height difference for traversable terrain
- `min_obstacle_height` (double, default: 0.15): Min height to consider as obstacle
- `range_limit` (double, default: 5.0): Maximum range to consider

## Terrain Analysis

### Height-based Classification

- **Obstacles**: Points above `min_obstacle_height` (default 15cm)
- **Traversable**: Points within height tolerance
- **Unknown**: Areas without depth data

### Slope Analysis

- **Free (cost 0)**: Slopes < 5.7° (0.1 rad)
- **Low cost (25)**: Slopes 5.7° - 16.7° (0.1-0.3 rad)  
- **High cost (75)**: Slopes 16.7° - 26.6° (0.3-0.5 rad)
- **Obstacle (100)**: Slopes > 26.6° (0.5 rad)

### Safety Features

- **Inflation**: 25cm safety buffer around obstacles
- **Conservative unknown areas**: Areas near obstacles marked as higher cost

## Usage

### Basic Launch

```bash
ros2 launch local_costmap stereo_costmap.launch.py
```

### With Custom Parameters

```bash
ros2 launch local_costmap stereo_costmap.launch.py grid_size:=8.0 resolution:=0.05
```

### Camera Topic Remapping

Edit the launch file to match your camera topics:

```python
remappings=[
    ('/stereo/left/image_raw', '/your_camera/left/image_raw'),
    ('/stereo/right/image_raw', '/your_camera/right/image_raw'),
    # ... etc
]
```

## Stereo Camera Setup

### Requirements

1. **Calibrated stereo camera pair** with known baseline
2. **Synchronized image capture** (hardware or software sync)
3. **Sufficient baseline** (recommended 8-15cm for close-range terrain)
4. **Good lighting** for reliable stereo matching

### Camera Configuration

Update the baseline in `stereo_costmap_node.py`:

```python
self.baseline = 0.12  # Your camera baseline in meters
```

### Calibration

Use ROS camera calibration tools:

```bash
ros2 run camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.108 \
    right:=/camera/right/image_raw left:=/camera/left/image_raw \
    right_camera:=/camera/right left_camera:=/camera/left
```

## Integration with Navigation

The stereo costmap can be used with navigation stacks by:

1. **Direct replacement**: Use `/stereo_costmap` instead of laser-based costmaps
2. **Fusion**: Combine with other sensors (IMU, LIDAR, etc.)
3. **Layered costmaps**: Use as an additional layer in navigation2

## Troubleshooting

### Poor Stereo Matching

- Check camera synchronization
- Ensure sufficient lighting
- Verify camera calibration
- Adjust stereo matcher parameters

### High CPU Usage

- Reduce image resolution
- Increase processing interval (decrease timer frequency)
- Optimize stereo matcher settings

### Inaccurate Terrain Assessment

- Verify camera baseline measurement
- Check camera mounting angle
- Adjust height thresholds for your robot
- Calibrate slope analysis parameters

## Dependencies

- OpenCV (python3-opencv)
- cv_bridge
- numpy
- scipy

Install with:

```bash
sudo apt install python3-opencv ros-jazzy-cv-bridge
```
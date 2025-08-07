# ERC25 Rover Web UI - Custom ROS2 Solution

A completely custom web-based user interface for ROS2 rover control and monitoring, built without external dependencies on rosbridge, roslibjs, or web_video_server.

## 🚀 Features

- **Direct ROS2 Communication**: No external bridges or libraries required
- **Real-time Data Streaming**: Live sensor data, camera feeds, and map visualization
- **Custom Video Streaming**: Built-in camera streaming with base64 encoding
- **Interactive Controls**: Virtual joystick and keyboard controls
- **Responsive Design**: Works on desktop and mobile devices
- **Modern UI**: Clean, professional interface with real-time status indicators

## 🏗️ Architecture

### Backend (Python/Flask)
- **Direct ROS2 Node**: Custom ROS2 node that subscribes to topics and publishes commands
- **Flask Web Server**: Serves the web interface and handles WebSocket connections
- **Socket.IO**: Real-time bidirectional communication with web clients
- **Custom Video Processing**: OpenCV-based image processing and base64 encoding

### Frontend (JavaScript/HTML/CSS)
- **Socket.IO Client**: Real-time communication with the server
- **Canvas-based Map Rendering**: Custom 2D map visualization
- **Virtual Joystick**: Touch/mouse-based control interface
- **Responsive Layout**: Three-panel design with controls, map, and cameras

## 📦 Dependencies

### Python Dependencies
```python
flask
flask-socketio
opencv-python
cv-bridge
numpy
rclpy
sensor_msgs
nav_msgs
geometry_msgs
std_msgs
```

### Frontend Dependencies
- Socket.IO Client (CDN)
- Font Awesome Icons (CDN)

## 🚀 Quick Start

### 1. Build the Package
```bash
cd ~/ros2_ws
colcon build --packages-select web_ui
source install/setup.bash
```

### 2. Launch the Web UI
```bash
# Simple launch (just the web UI)
ros2 launch web_ui web_ui.launch.py

# Or run directly
ros2 run web_ui web_ui_server
```

### 3. Access the Interface
Open your browser and navigate to:
- **Main Interface**: http://localhost:8000
- **Test Page**: http://localhost:8000/test_custom.html

## 🎮 Controls

### Keyboard Controls
- **W**: Move forward
- **S**: Move backward  
- **A**: Turn left
- **D**: Turn right

### Virtual Joystick
- **Mouse/Touch**: Drag the joystick knob to control movement
- **Linear**: Forward/backward movement
- **Angular**: Left/right turning

## 📊 Data Sources

The web UI subscribes to the following ROS2 topics:

### Navigation
- `/costmap` (nav_msgs/msg/OccupancyGrid) - 2D occupancy grid map
- `/cmd_vel` (geometry_msgs/msg/TwistStamped) - Command velocity (publishes)
- `/goal_pose` (geometry_msgs/msg/PoseStamped) - Goal pose (publishes)

### Sensors
- `/imu/data` (sensor_msgs/msg/Imu) - IMU data
- `/battery/battery_status` (sensor_msgs/msg/BatteryState) - Battery status
- `/hardware/e_stop` (std_msgs/msg/Bool) - E-stop status
- `/odometry/filtered` (nav_msgs/msg/Odometry) - Robot pose and velocity

### Cameras
- `/front_cam/zed_node/rgb/image_rect_color` (sensor_msgs/msg/Image)
- `/back_cam/zed_node/rgb/image_rect_color` (sensor_msgs/msg/Image)
- `/left_cam/zed_node/rgb/image_rect_color` (sensor_msgs/msg/Image)
- `/right_cam/zed_node/rgb/image_rect_color` (sensor_msgs/msg/Image)

## 🔧 Configuration

### Launch Parameters
- `port` (default: 8000) - Web server port
- `host` (default: 0.0.0.0) - Web server host address

### Customization
- **Topic Names**: Modify the topic subscriptions in `web_server.py`
- **Camera Topics**: Update camera topic names in the `setup_subscribers()` method
- **UI Layout**: Modify `index.html` and `style.css`
- **Map Rendering**: Customize map visualization in `app.js`

## 🧪 Testing

### Test Page
Access the test page at `http://localhost:8000/test_custom.html` to:
- Verify WebSocket connection
- Test data reception
- Test control commands
- Test API endpoints

### API Endpoints
- `GET /api/latest_data` - Get all current sensor data
- `POST /api/cmd_vel` - Send command velocity

## 🎯 Advantages Over External Solutions

### vs rosbridge + roslibjs
- ✅ **No External Dependencies**: No need for rosbridge_server or roslibjs
- ✅ **Better Performance**: Direct ROS2 communication without WebSocket overhead
- ✅ **Full Control**: Complete control over data processing and formatting
- ✅ **Simplified Setup**: Single executable, no additional ROS packages required

### vs web_video_server
- ✅ **Integrated Solution**: Video streaming built into the web UI
- ✅ **Custom Processing**: Full control over image compression and encoding
- ✅ **Better Integration**: Seamless integration with other sensor data
- ✅ **Reduced Complexity**: No separate video server process

## 🔍 Troubleshooting

### Common Issues

1. **Port Already in Use**
   ```bash
   # Check what's using port 8000
   netstat -tlnp | grep 8000
   # Kill the process if needed
   kill -9 <PID>
   ```

2. **No Map Data**
   - Ensure LIO-SAM or another SLAM system is running
   - Check that `/costmap` topic is being published
   ```bash
   ros2 topic list | grep costmap
   ros2 topic info /costmap
   ```

3. **No Camera Feeds**
   - Verify camera topics are being published
   - Check camera node is running
   ```bash
   ros2 topic list | grep camera
   ```

4. **Connection Issues**
   - Check firewall settings
   - Verify the web server is running on the correct port
   - Check browser console for WebSocket errors

### Debug Mode
Run with debug output:
```bash
ros2 run web_ui web_ui_server --ros-args --log-level debug
```

## 📁 File Structure

```
web_ui/
├── web_ui/
│   ├── web_server.py          # Main Flask/ROS2 server
│   └── www/                   # Web assets
│       ├── index.html         # Main interface
│       ├── style.css          # Styles
│       ├── app.js             # Frontend logic
│       ├── test_custom.html   # Test page
│       └── topics.json        # Topic configuration
├── launch/
│   └── web_ui.launch.py      # Launch file
├── setup.py                   # Package configuration
└── README.md                  # This file
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License.

## 🙏 Acknowledgments

- ROS2 community for the excellent robotics framework
- Flask and Socket.IO for the web framework
- OpenCV for computer vision capabilities

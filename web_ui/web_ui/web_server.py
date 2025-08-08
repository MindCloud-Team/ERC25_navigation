#!/usr/bin/env python3
"""
Custom web server with direct ROS2 communication and video streaming.
No external dependencies on rosbridge or web_video_server.
"""

import os
import sys
import threading
import rclpy
import math
import json
import base64
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu, BatteryState, Joy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from sensor_msgs_py import point_cloud2
from flask import Flask, send_from_directory, send_file, jsonify, request
from flask_socketio import SocketIO, emit
import cv_bridge

def get_package_share_path():
    """Get the path to the web assets directory (www)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = get_package_share_directory('web_ui')
        www_path = os.path.join(package_share, 'www')
        if os.path.exists(www_path):
            return www_path
        return package_share
    except (ImportError, Exception):
        pass
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    www_path = os.path.join(current_dir, '..', 'www')
    if os.path.exists(www_path):
        return os.path.normpath(www_path)
    # Fallback to package root if www not found
    return os.path.normpath(os.path.join(current_dir, '..'))

# Flask and SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app, async_mode='threading', cors_allowed_origins="*")

# Global ROS node
ros_node = None

class CustomROSNode(Node):
    """Custom ROS2 node that handles all communication directly."""
    
    def __init__(self):
        super().__init__('web_ui_custom_node')
        self.get_logger().info('Custom Web UI ROS node started')
        
        # Declare parameters
        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('topics_config', 'config/topics.json')
        
        # Initialize CV bridge for image processing
        self.bridge = cv_bridge.CvBridge()
        
        # Load topics configuration
        self.topics_config = self.load_topics_config()
        if self.topics_config:
            self.get_logger().info('Loaded topics configuration')
        else:
            self.get_logger().warn('Failed to load topics configuration, using hardcoded defaults')
        
        # Store latest data
        self.latest_data = {
            'map': None,
            'robot_pose': {'x': 0, 'y': 0, 'theta': 0},
            'imu': {'accel': {'x': 0, 'y': 0, 'z': 0}, 'gyro': {'x': 0, 'y': 0, 'z': 0}},
            'battery': {'voltage': 0, 'current': 0, 'percentage': 0},
            'estop': False,
            'odometry': {'pos': {'x': 0, 'y': 0, 'z': 0}, 'vel': {'linear': 0, 'angular': 0}},
            'cameras': {}
        }
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup publishers
        self.setup_publishers()
        
        # Start data broadcasting thread
        self.broadcast_thread = threading.Thread(target=self.broadcast_data, daemon=True)
        self.broadcast_thread.start()

    def resolve_topics_path(self, config_value: str) -> str:
        """Resolve the topics config file path, supporting absolute path or various common relative roots."""
        attempted_paths = []
        try:
            # Absolute path directly
            if os.path.isabs(config_value) and os.path.exists(config_value):
                return config_value

            roots = []
            # 1) Served www directory in install or devel space
            roots.append(get_package_share_path())
            # 2) Source-relative www directory
            current_dir = os.path.dirname(os.path.abspath(__file__))
            roots.append(os.path.normpath(os.path.join(current_dir, '..', 'www')))
            # 3) Package root (in case user passed 'topics.json' without 'config/')
            roots.append(os.path.normpath(os.path.join(current_dir, '..')))

            for root in roots:
                for rel in [config_value, os.path.join('config', config_value)]:
                    candidate = os.path.join(root, rel)
                    attempted_paths.append(candidate)
                    if os.path.exists(candidate):
                        return candidate
        except Exception:
            pass

        # Fallback to default bundled file under www/config
        fallback = os.path.join(get_package_share_path(), 'config', 'topics.json')
        attempted_paths.append(fallback)
        self.get_logger().error(f"Could not find topics config. Tried: {attempted_paths}")
        return fallback

    def load_topics_config(self):
        """Load topics config JSON into memory; fallback to a built-in default if not found."""
        try:
            cfg_param = self.get_parameter('topics_config').get_parameter_value().string_value
            cfg_path = self.resolve_topics_path(cfg_param)
            with open(cfg_path, 'r') as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().error(f'Could not load topics config: {exc}')
            # Built-in safe defaults that match simulation topics structure
            default_cfg = {
                'topics': {
                    'cameras': {
                        'front': '/front_cam/zed_node/rgb/image_rect_color',
                        'back': '/back_cam/zed_node/rgb/image_rect_color',
                        'left': '/left_cam/zed_node/rgb/image_rect_color',
                        'right': '/right_cam/zed_node/rgb/image_rect_color'
                    },
                    'sensors': {
                        'imu': '/imu/data',
                        'battery': '/battery/battery_status',
                        'estop': '/hardware/e_stop',
                        'odometry': '/odometry/filtered',
                        'joy': '/joy'
                    },
                    'navigation': {
                        'map': '/costmap',
                        'cmd_vel': '/cmd_vel',
                        'goal_pose': '/goal_pose'
                    }
                },
                'message_types': {
                    'cameras': 'sensor_msgs/msg/Image',
                    'imu': 'sensor_msgs/msg/Imu',
                    'battery': 'sensor_msgs/msg/BatteryState',
                    'estop': 'std_msgs/msg/Bool',
                    'odometry': 'nav_msgs/msg/Odometry',
                    'joy': 'sensor_msgs/msg/Joy',
                    'map': 'nav_msgs/msg/OccupancyGrid',
                    'cmd_vel': 'geometry_msgs/msg/TwistStamped',
                    'goal_pose': 'geometry_msgs/msg/PoseStamped'
                }
            }
            self.get_logger().warn('Using built-in default topics configuration')
            return default_cfg

    def setup_subscribers(self):
        """Setup all ROS2 subscribers"""
        topics = (self.topics_config or {}).get('topics', {})
        sensors = topics.get('sensors', {})
        navigation = topics.get('navigation', {})
        cameras = topics.get('cameras', {})

        # Resolve sensor and navigation topics with safe fallbacks
        map_topic = navigation.get('map', '/costmap')
        imu_topic = sensors.get('imu', '/imu/data')
        battery_topic = sensors.get('battery', '/battery/battery_status')
        estop_topic = sensors.get('estop', '/hardware/e_stop')
        odom_topic = sensors.get('odometry', '/odometry/filtered')

        # Map subscriber
        self.get_logger().info(f'Subscribing map -> {map_topic}')
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        # IMU subscriber
        self.get_logger().info(f'Subscribing imu -> {imu_topic}')
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        # Battery subscriber
        self.get_logger().info(f'Subscribing battery -> {battery_topic}')
        self.battery_sub = self.create_subscription(BatteryState, battery_topic, self.battery_callback, 10)
        # E-stop subscriber
        self.get_logger().info(f'Subscribing estop -> {estop_topic}')
        self.estop_sub = self.create_subscription(Bool, estop_topic, self.estop_callback, 10)
        # Odometry subscriber
        self.get_logger().info(f'Subscribing odometry -> {odom_topic}')
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odometry_callback, 10)

        # Camera subscribers based on config
        self.camera_subs = {}
        # Expected camera labels
        for label in ['front', 'back', 'left', 'right']:
            if label in cameras:
                topic_template = cameras[label]
                topic_resolved = topic_template.replace('${camera}', label)
                self.get_logger().info(f'Subscribing camera {label} -> {topic_resolved}')
                self.camera_subs[label] = self.create_subscription(
                    Image,
                    topic_resolved,
                    lambda msg, name=label: self.camera_callback(msg, name),
                    5
                )

    def setup_publishers(self):
        """Setup ROS2 publishers"""
        topics = (self.topics_config or {}).get('topics', {})
        navigation = topics.get('navigation', {})
        cmd_vel_topic = navigation.get('cmd_vel', '/cmd_vel')
        goal_pose_topic = navigation.get('goal_pose', '/goal_pose')

        # Command velocity publisher
        self.get_logger().info(f'Publishing cmd_vel -> {cmd_vel_topic}')
        self.cmd_vel_pub = self.create_publisher(TwistStamped, cmd_vel_topic, 10)
        
        # Goal pose publisher
        self.get_logger().info(f'Publishing goal_pose -> {goal_pose_topic}')
        self.goal_pose_pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)

    def map_callback(self, msg):
        """Handle map data"""
        try:
            # Convert numpy array to list for JSON serialization
            map_data = list(msg.data) if hasattr(msg.data, 'tolist') else list(msg.data)
            
            map_info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': float(msg.info.resolution),
                'origin': {
                    'x': float(msg.info.origin.position.x),
                    'y': float(msg.info.origin.position.y)
                },
                'data': map_data
            }
            self.latest_data['map'] = map_info
            socketio.emit('map_data', map_info)
        except Exception as e:
            self.get_logger().error(f'Error processing map data: {e}')

    def imu_callback(self, msg):
        """Handle IMU data"""
        try:
            imu_data = {
                'accel': {
                    'x': float(msg.linear_acceleration.x),
                    'y': float(msg.linear_acceleration.y),
                    'z': float(msg.linear_acceleration.z)
                },
                'gyro': {
                    'x': float(msg.angular_velocity.x),
                    'y': float(msg.angular_velocity.y),
                    'z': float(msg.angular_velocity.z)
                },
                'orientation': {
                    'x': float(msg.orientation.x),
                    'y': float(msg.orientation.y),
                    'z': float(msg.orientation.z),
                    'w': float(msg.orientation.w)
                }
            }
            self.latest_data['imu'] = imu_data
            socketio.emit('imu_data', imu_data)
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {e}')

    def battery_callback(self, msg):
        """Handle battery data"""
        try:
            battery_data = {
                'voltage': float(msg.voltage),
                'current': float(msg.current),
                'percentage': float(msg.percentage)
            }
            self.latest_data['battery'] = battery_data
            socketio.emit('battery_data', battery_data)
        except Exception as e:
            self.get_logger().error(f'Error processing battery data: {e}')

    def estop_callback(self, msg):
        """Handle E-stop data"""
        try:
            self.latest_data['estop'] = bool(msg.data)
            socketio.emit('estop_data', {'pressed': bool(msg.data)})
        except Exception as e:
            self.get_logger().error(f'Error processing E-stop data: {e}')

    def odometry_callback(self, msg):
        """Handle odometry data"""
        try:
            # Calculate yaw from quaternion
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            odom_data = {
                'pos': {
                    'x': float(msg.pose.pose.position.x),
                    'y': float(msg.pose.pose.position.y),
                    'z': float(msg.pose.pose.position.z)
                },
                'vel': {
                    'linear': float(math.sqrt(
                        msg.twist.twist.linear.x**2 + 
                        msg.twist.twist.linear.y**2 + 
                        msg.twist.twist.linear.z**2
                    )),
                    'angular': float(abs(msg.twist.twist.angular.z))
                }
            }
            
            self.latest_data['odometry'] = odom_data
            self.latest_data['robot_pose'] = {
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'theta': float(yaw)
            }
            
            socketio.emit('odometry_data', odom_data)
            socketio.emit('robot_pose', self.latest_data['robot_pose'])
            
        except Exception as e:
            self.get_logger().error(f'Error processing odometry data: {e}')

    def camera_callback(self, msg, camera_name):
        """Handle camera data"""
        try:
            # Convert ROS image to OpenCV BGR format with robust fallbacks
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                # Try native conversion first
                cv_image = self.bridge.imgmsg_to_cv2(msg)
                # Ensure BGR for JPEG
                encoding = (msg.encoding or '').lower()
                if len(cv_image.shape) == 2:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                elif len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                    if encoding.startswith('rgb'):
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                    # if already bgr-like, keep as is
                elif len(cv_image.shape) == 3 and cv_image.shape[2] == 4:
                    if encoding.startswith('rgba'):
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
                    elif encoding.startswith('bgra'):
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            
            # Compress image to JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            jpeg_data = base64.b64encode(buffer).decode('utf-8')
            
            camera_data = {
                'camera': camera_name,
                'image': jpeg_data,
                'timestamp': int(msg.header.stamp.sec)
            }
            
            self.latest_data['cameras'][camera_name] = camera_data
            socketio.emit('camera_data', camera_data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera data for {camera_name}: {e}')

    def broadcast_data(self):
        """Broadcast data to all connected clients"""
        while True:
            try:
                # Create a JSON-serializable copy of latest_data
                broadcast_data = {}
                for key, value in self.latest_data.items():
                    if key == 'map' and value is not None:
                        # Ensure map data is serializable
                        broadcast_data[key] = {
                            'width': value['width'],
                            'height': value['height'],
                            'resolution': value['resolution'],
                            'origin': value['origin'],
                            'data': value['data'] if isinstance(value['data'], list) else list(value['data'])
                        }
                    elif key == 'cameras':
                        # Handle camera data
                        broadcast_data[key] = {}
                        for cam_name, cam_data in value.items():
                            broadcast_data[key][cam_name] = {
                                'camera': cam_data['camera'],
                                'image': cam_data['image'],
                                'timestamp': cam_data['timestamp']
                            }
                    else:
                        broadcast_data[key] = value
                
                # Send latest data to all connected clients
                socketio.emit('latest_data', broadcast_data)
                threading.Event().wait(0.1)  # 10Hz update rate
            except Exception as e:
                self.get_logger().error(f'Error in broadcast thread: {e}')

    def publish_cmd_vel(self, linear_x, linear_y, angular_z):
        """Publish command velocity"""
        try:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = 'base_link'
            twist_msg.twist.linear.x = float(linear_x)
            twist_msg.twist.linear.y = float(linear_y)
            twist_msg.twist.angular.z = float(angular_z)
            
            self.cmd_vel_pub.publish(twist_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing cmd_vel: {e}')

# --- Flask Routes ---
www_path = get_package_share_path()

@app.route('/')
def index():
    """Serve the main index.html file"""
    return send_from_directory(www_path, 'index.html')

@app.route('/<path:filename>')
def serve_static(filename):
    """Serve static files from the www directory"""
    return send_from_directory(www_path, filename)

@app.route('/api/latest_data')
def get_latest_data():
    """API endpoint to get latest data"""
    if ros_node:
        # Create JSON-serializable copy
        data = {}
        for key, value in ros_node.latest_data.items():
            if key == 'map' and value is not None:
                data[key] = {
                    'width': value['width'],
                    'height': value['height'],
                    'resolution': value['resolution'],
                    'origin': value['origin'],
                    'data': value['data'] if isinstance(value['data'], list) else list(value['data'])
                }
            elif key == 'cameras':
                data[key] = {}
                for cam_name, cam_data in value.items():
                    data[key][cam_name] = {
                        'camera': cam_data['camera'],
                        'image': cam_data['image'],
                        'timestamp': cam_data['timestamp']
                    }
            else:
                data[key] = value
        return jsonify(data)
    return jsonify({})

@app.route('/api/config/topics')
def get_topics_config():
    """Serve the active topics configuration to the frontend."""
    global ros_node
    if ros_node and getattr(ros_node, 'topics_config', None):
        return jsonify(ros_node.topics_config)
    # Fallback: attempt to load default from disk
    try:
        default_path = os.path.join(www_path, 'config', 'topics.json')
        with open(default_path, 'r') as f:
            return jsonify(json.load(f))
    except Exception:
        return jsonify({})

@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    """API endpoint to send command velocity"""
    if ros_node:
        data = request.get_json()
        linear_x = data.get('linear_x', 0.0)
        linear_y = data.get('linear_y', 0.0)
        angular_z = data.get('angular_z', 0.0)
        
        ros_node.publish_cmd_vel(linear_x, linear_y, angular_z)
        return jsonify({'status': 'success'})
    return jsonify({'status': 'error', 'message': 'ROS node not available'})

# --- WebSocket Events ---
@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('connected', {'status': 'connected'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('cmd_vel')
def handle_cmd_vel(data):
    """Handle command velocity from client"""
    if ros_node:
        linear_x = data.get('linear_x', 0.0)
        linear_y = data.get('linear_y', 0.0)
        angular_z = data.get('angular_z', 0.0)
        ros_node.publish_cmd_vel(linear_x, linear_y, angular_z)

# --- Main Entry Point ---
def main():
    """Start the Flask server and ROS node."""
    global ros_node
    
    # Initialize ROS first to get parameters
    rclpy.init()
    ros_node = CustomROSNode()
    
    # Get port and host from ROS parameters
    port = ros_node.get_parameter('port').get_parameter_value().integer_value
    host = ros_node.get_parameter('host').get_parameter_value().string_value
    
    print(f"Starting Custom Web UI server on http://{host}:{port}")
    print(f"Serving files from: {www_path}")
    print("Direct ROS2 communication - no external dependencies!")
    
    # Start ROS node in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    # Start the Flask-SocketIO server
    socketio.run(app, host=host, port=port, debug=False, allow_unsafe_werkzeug=True)
    
    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

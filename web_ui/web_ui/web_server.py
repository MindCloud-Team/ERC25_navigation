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
    """Get the path to the package share directory"""
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
        return os.path.join(current_dir, '..')
    
    return os.path.join(current_dir, '..')

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
        
        # Initialize CV bridge for image processing
        self.bridge = cv_bridge.CvBridge()
        
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

    def setup_subscribers(self):
        """Setup all ROS2 subscribers"""
        
        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.map_callback,
            10
        )
        
        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Battery subscriber
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery/battery_status',
            self.battery_callback,
            10
        )
        
        # E-stop subscriber
        self.estop_sub = self.create_subscription(
            Bool,
            '/hardware/e_stop',
            self.estop_callback,
            10
        )
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )
        
        # Camera subscribers
        camera_topics = [
            '/front_cam/zed_node/rgb/image_rect_color',
            '/back_cam/zed_node/rgb/image_rect_color',
            '/left_cam/zed_node/rgb/image_rect_color',
            '/right_cam/zed_node/rgb/image_rect_color'
        ]
        
        self.camera_subs = {}
        for topic in camera_topics:
            camera_name = topic.split('/')[1]  # front_cam, back_cam, etc.
            self.camera_subs[camera_name] = self.create_subscription(
                Image,
                topic,
                lambda msg, name=camera_name: self.camera_callback(msg, name),
                5
            )

    def setup_publishers(self):
        """Setup ROS2 publishers"""
        
        # Command velocity publisher
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Goal pose publisher
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

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
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
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

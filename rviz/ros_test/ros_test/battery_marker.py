# battery_marker.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState , Imu 
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class BatteryMarker(Node):
    def __init__(self):
        super().__init__('battery_marker')

        #subscribers
        self.battery_sub = self.create_subscription( BatteryState, '/battery/battery_status', self.battery_callback, 10)
        self.imu_sub = self.create_subscription( Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription( Odometry, 'odometry/filtered', self.odom_callback, 10)

        #publishers
        self.battery_pub = self.create_publisher(Marker, '/battery_marker', 10)
        self.imu_pub = self.create_publisher(Marker, '/imu_marker', 10)
        self.odom_pub = self.create_publisher(Marker, '/odom_marker', 10)


    def battery_callback(self, msg):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'battery'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.0
        marker.pose.position.z = 3.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.text = f'Battery: {msg.percentage * 100:.1f}%'
        self.battery_pub.publish(marker)


    def imu_callback(self, msg):
        angular = msg.angular_velocity
        orientation = msg.orientation

        text = (
            f"imu data :\n"
            f"angular velocity: ({angular.x:.2f}, {angular.y:.2f}, {angular.z:.2f})\n"
            f"orientation: ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f})\n"

        )

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'imu'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = 0.9
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = text
        
        self.imu_pub.publish(marker)


    def odom_callback(self, msg: Odometry):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            lin = msg.twist.twist.linear
            ang = msg.twist.twist.angular

            text = (
                f"position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})\n"
                f"orientation: ({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f})\n"
                f"linear vel: ({lin.x:.2f}, {lin.y:.2f}, {lin.z:.2f})\n"
                f"angular vel: ({ang.x:.2f}, {ang.y:.2f}, {ang.z:.2f})"
            )

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id or "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "odom_text"
            marker.id = 1
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position.x = 0.0
            marker.pose.position.y = 3.0
            marker.pose.position.z = 1.0  # Show above robot
            marker.pose.orientation.w = 1.0

            marker.scale.z = 0.25
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.text = text
            marker.lifetime.sec = 0  # Keep forever

            self.odom_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

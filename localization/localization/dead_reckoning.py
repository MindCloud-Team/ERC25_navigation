#! /usr/bin/env python3
"""
A Sample for Localization of the rover using IMU and Encoders

This sample predicts the position of the robot using the encoders
and the orientation using the average of the values optained from the
Encoders and the IMU
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Int16
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rovers_interfaces.msg import Ticks
from tf2_ros import TransformBroadcaster
from math import pi, cos, sin
from transforms3d.euler import euler2quat

class LocalizationImuEncoder(Node):
    """
    ROS2 Node for locating the rover using IMU and Encoders

    This node subscribes to (/imu, /left_wheel_encoder and /left_wheel_encoder
    topic),  publishs Odometry message to /odom and broadcasts Transforms

    Attributes:
        ~rate (int): Proccessing rate in Hz
        imu_sub (Subscriber): Subscribes to IMU data
        l_encoder_sub (Subscriber): Subscribes to "DELTA TICKS" of the left wheel
        r_encoder_sub (Subscriber): Subscribes to "DELTA TICKS" of the right wheel
        odom_pub (Publisher): Publisher Odometry Data
        tf_broadcaster (Broadcaster): Broadcasts Transforms
        base_width (float): distance between wheels in cm
        distance_per_tick (float): distance moved per one tick of the encoder
        tf_msg (TransformStamped): broadcasted transforms
        odom_msg (Odometry): Published message to Odom
        yaw (float): Current angular velocity of the robot
        left_wheel_ticks (Int64): received ticks of the encoder
        right_wheel_ticks (Int64): received ticks of the encoder
        x (float): estimated x position of the rover
        y (float): estimated y position of the rover
        theta_both (float): estimated theta of the rover
        prev_theta (float): used to calculate delta theta
        self.dt (float): Time Period
        main_timer (timer): loops the update function with rate = self.rate

    ROS Parameters:
        ~rate (int): Proccessing rate in Hz
        ~wheel_diameter (float): Wheel diameter in cm
        ~base_width (float): distance between wheels in cm
        ~frame_id (string): main frame id
        ~child_frame_id (string): rover's frame id
        ~ticks_per_rev (int): Number of ticks per One Revolution of the motor

    ROS Subscriber:
        /imu (sensor_msgs/Imu)
        /left_wheel_encoder (std_msgs/Int16)
        /right_wheel_encoder (std_msgs/Int16)

    ROS Publishers:
        /odom (nav_msgs/Odometry)

    ROS Broadcasters:
        /odom -> base_link (geometry_msgs/TransformStamped)
    """

    def __init__(self):
        """
        Initialize the LocalizationImuEncoder.

        Sets up ROS publishers, subscribers.
        """
        super().__init__("localization_imu_encoder")

        # Declare Parameters
        self.declare_parameter("rate", 50)
        self.declare_parameter("wheel_diameter", 0.12)
        self.declare_parameter("ticks_per_rev", 600)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("base_width", 0.60)

        # Get Parameters
        self.rate = self.get_parameter("rate").value
        self.base_width = self.get_parameter("base_width").value
        wheel_diameter = self.get_parameter("wheel_diameter").value
        ticks_per_rev = self.get_parameter("ticks_per_rev").value

        # Set Variables
        self.distance_per_tick = (pi * wheel_diameter) / ticks_per_rev
        self.tf_msg = TransformStamped()
        self.odom_msg = Odometry()
        self.yaw = 0.0
        self.left_wheel_ticks = 0
        self.right_wheel_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.theta_both = 0.0
        self.prev_theta = 0.0
        self.dt = 1 / self.rate
        self.main_timer = self.create_timer(1 / self.rate, self._update)
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        self.l_multiplier = 0
        self.r_multiplier = 0
        self.l_prev_real_ticks = 0
        self.r_prev_real_ticks = 0

        # Create Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self._imu_callback,
            10
        )

        # self.l_encoder_sub = self.create_subscription(
        #     Int16,
        #     "/left_wheel_encoder",
        #     self._left_wheel_callback,
        #     10
        # )

        # self.r_encoder_sub = self.create_subscription(
        #     Int16,
        #     "/right_wheel_encoder",
        #     self._right_wheel_callback,
        #     10
        # )

        self._encoders_sub = self.create_subscription(
            Int16,
            "/encoder_ticks",
            self._encoders_callback,
            10
        )

        # Create Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        # Create Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def _update(self):
        """
        Main Function which processes the data and locate the robot
        """
        # Store Previous Theta value
        self.prev_theta = self.theta_both

        # Calculate distance
        l_distance = self.left_wheel_ticks * self.distance_per_tick
        r_distance = self.right_wheel_ticks * self.distance_per_tick
        self.distance = (l_distance + r_distance) / 2

        # Estimate Position and Orientation
        self.x += self.distance * cos(self.theta_both)
        self.y += self.distance * sin(self.theta_both)
        theta_enc = self.theta_both + ((l_distance - r_distance) / self.base_width)
        theta_imu = self.theta_both + self.yaw * self.dt
        self.theta_both = (theta_enc + theta_imu) / 2  # Basic Sensor Fusion for now

        # Publish and Broadcast messages
        self._create_odom_msg()
        self._create_tf_msg()
        
        self.get_logger().info(f"X = {self.x}, Y = {self.y}, theta = {self.theta_both}")

    def _create_tf_msg(self):
        """
        Creates the transform message and broadcasts
        """
        self.tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_msg.header.frame_id = self.get_parameter("frame_id").value
        self.tf_msg.child_frame_id = self.get_parameter("child_frame_id").value

        # Set position
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        self.tf_msg.transform.translation.z = 0.0

        # Set Orientation
        qx, qy, qz, qw = euler2quat(0, 0, self.theta_both)
        self.tf_msg.transform.rotation.x = qx
        self.tf_msg.transform.rotation.y = qy
        self.tf_msg.transform.rotation.z = qz
        self.tf_msg.transform.rotation.w = qw

        # Broadcast
        self.tf_broadcaster.sendTransform(self.tf_msg)

    def _create_odom_msg(self):
        """
        Creates the Odometry message and publishs to /odom
        """
        # Transform from euler to quaternion
        qx, qy, qz, qw = euler2quat(0, 0, self.theta_both)

        self.odom_msg.header.frame_id = self.get_parameter("frame_id").value
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Set position
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.0

        # Set Orientation
        self.odom_msg.pose.pose.orientation.x = qx
        self.odom_msg.pose.pose.orientation.y = qy
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.pose.pose.orientation.z = qz

        self.odom_msg.child_frame_id = self.get_parameter("child_frame_id").value

        # Set velocity
        self.odom_msg.twist.twist.linear.x = self.distance / self.dt
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.angular.z = (self.theta_both - self.prev_theta) / self.dt

        # Publish
        self.odom_pub.publish(self.odom_msg)

    def _imu_callback(self, msg: Imu):
        """
        Stores the yaw from the IMU

        Args:
            msg (sensor_msgs/Imu)
        """
        self.yaw = msg.angular_velocity.z

    # def _right_wheel_callback(self, msg: Int16):
    #     """
    #     Stores the ticks from the encoder

    #     Args:
    #         msg (sts_msgs/Int16)
    #     """
    #     self.right_wheel_ticks = msg.data

    # def _left_wheel_callback(self, msg: Int16):
    #     """
    #     Stores the ticks from the encoder

    #     Args:
    #         msg (sts_msgs/Int16)
    #     """
    #     self.left_wheel_ticks = msg.data

    def _encoders_callback(self, msg: Ticks):
        l_real_ticks = msg.ticks_left
        r_real_ticks = msg.ticks_right

        if (l_real_ticks < self.encoder_low_wrap and self.l_prev_real_ticks > self.encoder_high_wrap):
            self.l_multiplier += 1

        if (l_real_ticks > self.encoder_high_wrap and self.l_prev_real_ticks < self.encoder_low_wrap):
            self.l_multiplier -= 1

        self.left_wheel_ticks = 1.0 * (l_real_ticks + self.l_multiplier * (self.encoder_max - self.encoder_min))
        self.l_prev_real_ticks = l_real_ticks

        if (r_real_ticks < self.encoder_low_wrap and self.r_prev_real_ticks > self.encoder_high_wrap):
            self.r_multiplier += 1

        if (r_real_ticks > self.encoder_high_wrap and self.r_prev_real_ticks < self.encoder_low_wrap):
            self.r_multiplier -= 1

        self.right_wheel_ticks = 1.0 * (r_real_ticks + self.r_multiplier * (self.encoder_max - self.encoder_min))
        self.r_prev_real_ticks = r_real_ticks

        self.get_logger().info(f"left ticks: {self.left_wheel_ticks}")
        self.get_logger().info(f"right ticks: {self.right_wheel_ticks}")


def main(args=None):
    """
    Main entry point for the localization node.
    """
    rclpy.init(args=args)
    node = LocalizationImuEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

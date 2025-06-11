#!/usr/bin/env python3
"""
A Sample for Controlling the motion of the rover

This sample calculates the linear and angular error of the rover
and publishes linear and angular velocities accordingly
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from transforms3d.euler import quat2euler
from math import atan2, degrees, cos, sin, sqrt

class PIDControllerNode(Node):
    """
    ROS2 Node for controlling the motion of the rover usin PID

    This node subscribes to (/odom and /path) and publishes Twist message to /cmd_vel

    Attributes:
        dt (float): Time Period
        previous_cte_error (float): Stores the previous CTE error
        integral_cte (float): Accumilates the CTE error
        prev_error_heading (float): Stores the previous heading error
        integral_heading (float): Accumilates the heading error
        rover_x (float): Current X Position of the Rover
        rover_y (float): Current Y Position of the Rover
        rover_yaw (float): Current Yaw of the Rover
        desired_heading (float): Desired Yaw of the Rover
        rover_linear_vx (float): Current LinearX velocity of the Rover
        rover_angular_vz (float): Current Angular Z Position of the Rover
        target_x (float): Desired X Position of the Rover
        target_y (float): Desired Y position of the Rover
        timer (timer): loops the compute_velocity function every dt

        ROS Parameters:
        ~rate (int):
        ~linear_kp (float): Kp Value for Linear velocity
        ~linear_ki (float): Ki Value for Linear velocity
        ~linear_kd (float): Kd Value for Linear velocity
        ~angular_kp (float): Kp Value for Angular velocity
        ~angular_ki (float): Ki Value for Angular velocity
        ~angular_kd (float): Kd Value for Angular velocity

        ROS Subscribers:
        /path (nav_msgs/Path)
        /odom (nav_msgs/Odometry)

        ROS Publishers:
        /cmd_vel (geometry_msgs/Twist)
    """
    def __init__(self):
        """
        Initialize the PIDControllerNode.

        Sets up ROS publishers, subscribers.
        """
        super().__init__('motion_control')
               
        # Declare Parameters
        self.declare_parameter("rate", 50)
        self.declare_parameter("linear_kp", 1.0)
        self.declare_parameter("linear_ki", 0.5)
        self.declare_parameter("linear_kd", 0.2)
        self.declare_parameter("angular_kp", 1.2)
        self.declare_parameter("angular_ki", 0.3)
        self.declare_parameter("angular_kd", 0.4)

        # Attributes
        self.dt = 1 / self.get_parameter("rate").value
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0
        self.desired_heading = 0.0
        self.rover_linear_vx = 0.0
        self.rover_angular_vz = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.previous_cte_error = 0.0
        self.integral_cte = 0.0
        self.prev_error_heading = 0.0
        self.integral_heading = 0.0
        self.timer = self.create_timer(self.dt, self._compute_velocity)

        # Create Publishers
        self.twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create Subscribers
        self.path_sub = self.create_subscription(
            Path,
            "/path",
            self._path_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # Log Initialization
        self.get_logger().info("Motion Control Node Initialized")

    def _compute_velocity(self):
        """
        Main Function which computes the velocity using PID
        """
        cte = self._calculate_cte()
        heading_error = self._calculate_heading()

        self.linear_pid_value, self.previous_cte_error, self.integral_cte = \
        self._calculate_PID(
            cte,
            self.previous_cte_error,
            self.integral_cte,
            self.get_parameter("linear_kp").value,
            self.get_parameter("linear_ki").value,
            self.get_parameter("linear_kd").value,
            self.dt
        )
            
        self.angular_pid_value, self.prev_error_heading, self.integral_heading = \
        self._calculate_PID(
            heading_error,
            self.prev_error_heading,
            self.integral_heading,
            self.get_parameter("angular_kp").value,
            self.get_parameter("angular_ki").value,
            self.get_parameter("angular_kd").value,
            self.dt
        )

        self._send_vel_command()

    def _send_vel_command(self):
        """
        Creates the Twist message and publishs to /cmd_vel
        """
        # Ensure linear speed is always positive
        linear_speed = max(0.0, self.linear_pid_value)

        if abs(self._calculate_heading()) > 0.5:
            linear_speed *= 0.5

        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = self.angular_pid_value

        self.twist_pub.publish(msg)

    def _calculate_PID(self, error, prev_error, integral_err, kp, ki, kd, dt):
        """
        PID main function
        
        Args:
            error (float): current error
            prev_error (float): previous error
            integral_error (float): error accumilator
            kp (float): Kp value for calculation
            ki (float): Ki value for calculation
            kd (float): Kd value for calculation
            dt (float): Time Period

        Returns:
            output (float): PID output value
            error (float): current error
            integral_err (float): new accumilated error
        """ 
        integral_err += error * dt
        derivative = (error - prev_error) / dt

        output = (kp * error) + (ki * integral_err) + (kd * derivative)
        return output, error, integral_err

    def _calculate_cte(self):
        """
        Calculates CTE each loop iteration

        Returns:
            CTE (float): CTE value
        """
        cte = sqrt(((self.rover_y - self.target_y) ** 2) + \
                   ((self.rover_x - self.target_x) ** 2))  
        return cte

    def _calculate_heading(self):
        """
        Calculates Heading error each loop iteration

        Returns:
            heading_error (float): Heading Error value
        """
        # Calculate heading_error using atan2 to stay within -pi & pi
        heading_error = atan2(
        sin(self.desired_heading - self.rover_yaw),
        cos(self.desired_heading - self.rover_yaw)
        )
        return heading_error

    def _odom_callback(self, msg: Odometry):
        """
        Stores the current position and orientation of the Rover
        """
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        quatMsg = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, 
                   msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        _, _, self.rover_yaw = quat2euler(quatMsg)
        self.rover_linear_vx = msg.twist.twist.linear.x
        self.rover_angular_vz = msg.twist.twist.angular.z

        self.get_logger().info(f"Odometry: X={self.rover_x}, Y={self.rover_y},\
                                Yaw={degrees(self.rover_yaw)}°")
        self.get_logger().info(f"Linear Velocity: {self.rover_linear_vx},\
                                Angular Velocity: {self.rover_angular_vz}")

    def _path_callback(self, msg:Path):
        """
        Stores the desired path of the Rover
        """
        if not msg.poses:
            self.get_logger().warn("Received empty path message!")
            return
        
        self.target_poses = msg.poses
        self.target_x = self.target_poses[0].pose.position.x
        self.target_y = self.target_poses[0].pose.position.y
        self.desired_heading = atan2(
            self.target_y - self.rover_y,
            self.target_x - self.rover_x
        )
        self.get_logger().info(f"Target Waypoint: X={self.target_x},\
                                Y={self.target_y}")
        self.get_logger().info(f"Desired Heading: \
                               {degrees(self.desired_heading)}°")

def main(args=None):
    """
    Main entry point for the localization node.
    """
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
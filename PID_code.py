#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math 
from std_msgs import Int16
from functools import partial
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from rclpy.action.client import ClientGoalHandle

class PID():
    def __init__(self,Kp,Kd,Ki):
        self.lKp = 0.0
        self.lKd = 0.0
        self.lKi = 0.0

        self.AKp = 0.0
        self.AKd = 0.0
        self.AKi = 0.0

    def update_pid(self):
        pass 

    def calculate_cte(self):
        pass

    def calculate_heading(self):
        pass 

    def Control(self):
        pass


        
class ControlNode(Node):
    def __init__(self):
        super().__init__('Trajectory')
        self.odom_sub = self.create_subscription(Odometry,'odom',10,self.odom_call_back)   # subcribe to 'odom'  topic 
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',10) # publish to 'cmd_vel' topic
        self.client = ActionClient(self,FollowJointTrajectory,'Trajectory')
        
    def send_goal(self):

        self.count_until_client.wait_for_server() #wait to the action server 

        goal = FollowJointTrajectory.Goal() # create a goal 

        goal.trajectory.joint_names = ['wheel_right_joint','wheel_left_joint']  #specfiy joint names 

        point = JointTrajectoryPoint()  #create point 

        point.positions = [1.0,2.0]  # give postion to the wheel 
        point.velocities = [3.0,4.0]  #give veolicty to the wheel 
        point.accelerations = [2.0,2.0]  #give accleration  to the wheel 

        goal.trajectory.points = [point]  

        self.get_logger().info("sending goal")  

        self.count_until_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)  

    def goal_response_callback(self, future):
            self.goal_handle_: ClientGoalHandle = future.result() # type: ignore
            if self.goal_handle_.accepted:
                self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
            result = future.result().result
            self.get_logger().info(f"result : {str(result.error_string)}")

    
    def feedback(self,feedback_msg):
         actual_positions = feedback_msg.feedback.actual.position    # taking real postion
         actual_velocities = feedback_msg.feedback.actual.velocity   # taking real veolicty 
         desired_positions = feedback_msg.feedback.desired.position  # taking desired postion
         desired_velocities = feedback_msg.feedback.desired.velocity  # taking desired veolicty


         self.get_logger().info(f'desired postion {desired_positions} desired velocities {desired_velocities}')
         self.get_logger().info(f'actual postion {actual_positions} actual velocities {actual_velocities}')
         
        


    def odom_call_back(self,msg:Odometry):
       self.robot_x = msg.pose.pose.postion.x
       self.robot_y = msg.pose.pose.postion.y

       self.robot_linear_veolicty = self.twist.twist.linear.x
       self.robot_linear_veolicty = self.twist.twist.linear.y
       self.robot_linear_veolicty = self.twist.twist.linear.z


       self.robot_angular_veolicty = self.twist.twist.angular.x
       self.robot_angular_veolicty = self.twist.twist.angular.y
       self.robot_angular_veolicty = self.twist.twist.angular.z




       

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
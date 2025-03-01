#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math 


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

    def Control(self);
        pass

class Trajectory(Node):
    def __init__(self):
        super().__init__('Trajectory')
        self.odom_sub = self.create_subscription(Odometry,'odom',10,self.odom_call_back)
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',10)



    def odom_call_back(self,msg:Odometry):
        pass


        
        

        

def main(args=None):
    rclpy.init(args=args)
     

    node = Trajectory()
    rclpy.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
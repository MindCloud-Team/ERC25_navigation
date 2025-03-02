#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math 
from std_msgs import Int16
from functools import partial


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

class Encoder(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.create_subscription(Int16,'/encoder_ticks',10,self.call_back_encoder)

    def call_back_encoder(self,msg:Int16):
        self.ticks = msg.data

        
class Trajectory(Node):
    def __init__(self):
        super().__init__('Trajectory')
        self.odom_sub = self.create_subscription(Odometry,'odom',10,self.odom_call_back)
        self.cmd_pub = self.create_publisher(Twist,'cmd_vel',10)




    def odom_call_back(self,msg:Odometry):
       self.robot_x = msg.pose.pose.postion.x
       self.robot_y = msg.pose.pose.postion.y

       


    def cmd_vel_service(self):
        client = self.create_client(Twist,'cmd_vel')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service")

        request =Twist.Request()
        request.linear.x = # liner veolcity from pid 


        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))



    def callback_set_pen(self,future):
        try:
             response = future.result()
        except Exception as e:
            self.get_logger().error(f"service call failed :{e}")


       

def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
    rclpy.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from rovers_interfaces.msg import Ticks

class Test(Node):
    def __init__(self):
        super().__init__('navigation')
        # Subscribed topics
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_cb,
            10)
        # Published topics
        self.pub_ticks = self.create_publisher(Ticks, 'encoder_ticks', 10)        

        self.create_timer(0.1, self.ticks_publisher)

        # Constants:
        self.rate = 10

        # Variables:
        self.right_ticks = 0
        self.left_ticks = 0
        self.linear_vel = 0
        self.angular_vel = 0


    def vel_cb(self, msg:Twist):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def ticks_publisher(self):
        self.encoder = Ticks()
        if self.linear_vel > 0.2: # forward
            self.encoder.ticks_right+= 50
            self.encoder.ticks_left += 50

        if self.linear_vel < -0.2: # backward
           self.encoder.ticks_right -= 50
           self.encoder.ticks_left -= 50

        if self.angular_vel < -0.2: # right
           self.encoder.ticks_right -= 50
           self.encoder.ticks_left += 50

        if self.angular_vel > 0.2: # left
           self.encoder.ticks_right += 50
           self.encoder.ticks_left -= 50

        if self.angular_vel == 0 and self.linear_vel == 0:
            self.encoder.ticks_right = self.encoder.ticks_right
            self.encoder.ticks_left = self.encoder.ticks_left

        if self.encoder.ticks_right > 32700:
            self.encoder.ticks_right = -32700
        if self.encoder.ticks_right < -32700:
            self.encoder.ticks_right = 32700

        if self.encoder.ticks_left > 32700:
            self.encoder.ticks_left = -32700
        if self.encoder.ticks_left < -32700:
            self.encoder.ticks_left = 32700
        
        self.pub_ticks.publish(self.encoder)


def main(args=None):
    rclpy.init(args=args)
    test_node = Test()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
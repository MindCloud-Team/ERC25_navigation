import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

class JoyToMotionNode(Node):
    def __init__(self):
        super().__init__('joy_to_motion_node')
        
        # Create a publisher to send TwistStamped data
        self.pub_twist = self.create_publisher(TwistStamped, "cmd_vel", 10)        
        # Create a subscription to the joystick (joy) data
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg: Joy):
        twist_msg = TwistStamped()
        
        # Set the header with current time
        twist_msg.header = Header()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'  # or whatever frame is appropriate
        
        # Set the twist values from joystick axes
        twist_msg.twist.linear.x = msg.axes[1]  # typically axis 1 is forward/backward
        twist_msg.twist.angular.z = msg.axes[0]  # typically axis 0 is left/right
        
        # Publish the TwistStamped message
        self.pub_twist.publish(twist_msg)
        self.get_logger().info(
            f"Published TwistStamped: linear.x={twist_msg.twist.linear.x}, "
            f"angular.z={twist_msg.twist.angular.z}, "
            f"frame_id='{twist_msg.header.frame_id}'"
        )

def main(args=None):
    rclpy.init(args=args)
    joy_to_motion_node = JoyToMotionNode()
    
    # Spin the node to keep it active and listening to the joy topic
    rclpy.spin(joy_to_motion_node)
    
    joy_to_motion_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
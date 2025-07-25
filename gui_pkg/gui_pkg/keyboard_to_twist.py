#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header

from pynput import keyboard
import threading


class KeyboardToMotionNode(Node):
    def __init__(self):
        super().__init__("keyboard_to_motion_node")

        self.pub_twist = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.linear = 0.0
        self.angular = 0.0

        # Start keyboard listener in separate thread
        self.listener_thread = threading.Thread(target=self.keyboard_listener)
        self.listener_thread.start()

        # Timer to publish at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_twist)

    def keyboard_listener(self):
        def on_press(key):
            try:
                if key.char == "w":
                    self.linear = 1.0
                elif key.char == "s":
                    self.linear = -1.0
                elif key.char == "a":
                    self.angular = 1.0
                elif key.char == "d":
                    self.angular = -1.0
            except AttributeError:
                pass

        def on_release(key):
            # Stop movement on key release
            self.linear = 0.0
            self.angular = 0.0

        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()

    def publish_twist(self):
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = self.linear
        twist_msg.twist.angular.z = self.angular

        self.pub_twist.publish(twist_msg)
        self.get_logger().info(
            f"Publishing: linear.x={self.linear}, angular.z={self.angular}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardToMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

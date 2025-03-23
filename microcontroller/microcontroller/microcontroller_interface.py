#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# Serial port settings - update with your actual port names
TWIST_ARDUINO_PORT = "/dev/ttyACM0"   # Arduino for motor control
LEFT_ENCODER_PORT = "/dev/ttyACM1"    # Arduino for left encoder
RIGHT_ENCODER_PORT = "/dev/ttyACM2"   # Arduino for right encoder
BAUD_RATE = 9600

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publishers for encoder ticks
        self.left_ticks_pub = self.create_publisher(Int16, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Int16, 'right_ticks', 10)
        self.left_rpm_pub = self.create_publisher(Int16, 'rpm_left', 10)
        self.right_rpm_pub = self.create_publisher(Int16, 'rpm_right', 10)

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Initialize serial connections
        try:
            self.twist_serial = serial.Serial(TWIST_ARDUINO_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info("Connected to twist Arduino")
            time.sleep(2)  # Wait for Arduino reset

            self.left_encoder_serial = serial.Serial(LEFT_ENCODER_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info("Connected to left encoder Arduino")
            time.sleep(2)  # Wait for Arduino reset

            self.right_encoder_serial = serial.Serial(RIGHT_ENCODER_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info("Connected to right encoder Arduino")
            time.sleep(2)  # Wait for Arduino reset

        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise

        # Create threads for reading from encoders
        self.left_encoder_thread = threading.Thread(target=self.read_left_encoder)
        self.right_encoder_thread = threading.Thread(target=self.read_right_encoder)

        # Set as daemon so threads exit when main program exits
        self.left_encoder_thread.daemon = True
        self.right_encoder_thread.daemon = True

        # Start threads
        self.left_encoder_thread.start()
        self.right_encoder_thread.start()

        # Set up timer for heartbeat (1Hz)
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat)

        self.get_logger().info("Robot controller initialized")

    def cmd_vel_callback(self, msg):
        """Process incoming cmd_vel messages and send to twist Arduino"""
        try:
            # Format command for the Arduino
            # We'll send the linear.x and angular.z values directly
            # The Arduino's code already handles the conversion to motor speeds
            command = f"\"linear_x\":{int(msg.linear.x * 1000)},\"angular_z\":{int(msg.angular.z * 1000)}\n"
            self.twist_serial.write(command.encode())
            self.get_logger().info(f"Sent twist: {command.strip()}")
            time.sleep(0.1)

        except Exception as e:
            self.get_logger().error(f"Failed to send twist command: {e}")

    def read_left_encoder(self):
        """Thread function to read from left encoder Arduino"""
        left_ticks_msg = Int16()
        left_rpm_msg = Int16()

        while rclpy.ok():
            try:
                if self.left_encoder_serial.in_waiting > 0:
                    line = self.left_encoder_serial.readline().decode().strip()
                    # Look for tick data
                    if line.startswith("TICKS:"):
                        try:
                            ticks = int(line.split(":")[1])
                            left_ticks_msg.data = ticks
                            self.left_ticks_pub.publish(left_ticks_msg)
                        except (ValueError, IndexError):
                            pass
                    # Look for RPM data
                    elif line.startswith("RPM:"):
                        try:
                            rpm = int(line.split(":")[1])
                            left_rpm_msg.data = rpm
                            self.left_rpm_pub.publish(left_rpm_msg)
                        except (ValueError, IndexError):
                            pass
            except Exception as e:
                self.get_logger().error(f"Error reading from left encoder: {e}")

            # Short sleep to prevent CPU overload
            time.sleep(0.01)

    def read_right_encoder(self):
        """Thread function to read from right encoder Arduino"""
        right_ticks_msg = Int16()
        right_rpm_msg = Int16()

        while rclpy.ok():
            try:
                if self.right_encoder_serial.in_waiting > 0:
                    line = self.right_encoder_serial.readline().decode().strip()
                    # Look for tick data
                    if line.startswith("TICKS:"):
                        try:
                            ticks = int(line.split(":")[1])
                            right_ticks_msg.data = ticks
                            self.right_ticks_pub.publish(right_ticks_msg)
                        except (ValueError, IndexError):
                            pass
                    # Look for RPM data
                    elif line.startswith("RPM:"):
                        try:
                            rpm = int(line.split(":")[1])
                            right_rpm_msg.data = rpm
                            self.right_rpm_pub.publish(right_rpm_msg)
                        except (ValueError, IndexError):
                            pass
            except Exception as e:
                self.get_logger().error(f"Error reading from right encoder: {e}")

            # Short sleep to prevent CPU overload
            time.sleep(0.01)

    def heartbeat(self):
        """Send a periodic heartbeat message"""
        self.get_logger().info("Robot controller is running...")

    def close_connections(self):
        """Close all serial connections"""
        try:
            self.twist_serial.close()
            self.left_encoder_serial.close()
            self.right_encoder_serial.close()
            self.get_logger().info("Closed all Arduino connections")
        except:
            pass

def main(args=None):
    rclpy.init(args=args)

    try:
        robot_controller = RobotController()

        try:
            rclpy.spin(robot_controller)
        except KeyboardInterrupt:
            pass
        finally:
            # Shutdown and cleanup
            robot_controller.close_connections()
            robot_controller.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error initializing robot controller: {e}")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

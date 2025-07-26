import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rovers_interfaces.msg import BBoxList
from builtin_interfaces.msg import Duration

class BBoxToMarkerNode(Node):
    def __init__(self):
        super().__init__('bbox_to_marker')
        self.subscription = self.create_subscription(
            BBoxList,
            'bbox_list_topic',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Marker,
            'visualization_marker',
            10
        )

    def listener_callback(self, msg: BBoxList):
        for i, box in enumerate(msg.boxes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            center_x = (box.top_left.x + box.bottom_right.x) / 2.0
            center_y = (box.top_left.y + box.bottom_right.y) / 2.0

            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.05

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = abs(box.bottom_right.x - box.top_left.x)
            marker.scale.y = abs(box.bottom_right.y - box.top_left.y)
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker.id = i
            marker.lifetime = Duration(seconds=0).to_msg()

            self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = BBoxToMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

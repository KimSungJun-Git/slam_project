import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapListener(Node):
    def __init__(self):
        super().__init__('map_listener')
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.get_logger().info(
            f"map size: {msg.info.width} x {msg.info.height}"
        )

def main():
    rclpy.init()
    node = MapListener()
    rclpy.spin(node)
    rclpy.shutdown()

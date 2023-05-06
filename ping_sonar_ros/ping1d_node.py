import rclpy
from .ping1d_component import Ping1dComponent

def main(args=None):
    rclpy.init(args=args)
    range_publisher = Ping1dComponent()
    rclpy.spin(range_publisher)
    range_publisher.destroy_node()
    rclpy.shutdown()
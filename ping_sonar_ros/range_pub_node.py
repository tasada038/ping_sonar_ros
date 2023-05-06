import rclpy

from .range_pub_component import RangePublisher

def main(args=None):
    rclpy.init(args=args)
    range_publisher = RangePublisher()
    rclpy.spin(range_publisher)
    range_publisher.destroy_node()
    rclpy.shutdown()

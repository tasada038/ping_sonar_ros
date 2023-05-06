from rclpy.node import Node
from sensor_msgs.msg import Range

class RangePublisher(Node):
    def __init__(self):
        super().__init__("range_publisher")
        self.publisher_ = self.create_publisher(Range, "range_topic", 10)
        self.timer_ = self.create_timer(1.0, self.publish_range)

    def publish_range(self):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = "range_sensor"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.1
        range_msg.min_range = 0.2
        range_msg.max_range = 4.0
        range_msg.range = 1.5
        self.publisher_.publish(range_msg)
        self.get_logger().info("Publishing range: {}".format(range_msg.range))
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(Odometry, '/sub_odom', self.cb, 10)

    def cb(self, msg):
        now = self.get_clock().now().to_msg()
        delay = (rclpy.time.Time.from_msg(now) - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        self.get_logger().info(f"Delay: {delay:.3f} s")

rclpy.init()
rclpy.spin(Listener())

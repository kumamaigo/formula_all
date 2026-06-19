#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/aiformula_sensing/zed_node/left_image/undistorted',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.save_dir = '/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/datasets'
        os.makedirs(self.save_dir, exist_ok=True)
        self.counter = 0
        self.get_logger().info(f"Saving images to: {self.save_dir}")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename = os.path.join(self.save_dir, f"frame_{self.counter:06d}.jpg")
            cv2.imwrite(filename, cv_image)
            self.counter += 1
            if self.counter % 50 == 0:
                self.get_logger().info(f"Saved {self.counter} images")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import csv
import math
from datetime import datetime

class PIDLogger(Node):
    def __init__(self):
        super().__init__('pid_logger')
        self.cmd_sub = self.create_subscription(Twist, '/aiformula_control/twist_mux/cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom', self.odom_callback, 10)

        self.current_cmd = Twist()
        self.current_odom = Odometry()

        # CSVファイル名をタイムスタンプで
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = f"/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/pid_eval_{now}.csv"
        self.file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(["time", "cmd_linear", "odom_linear", "cmd_angular", "odom_angular", "error_linear", "error_angular"])
        self.get_logger().info(f"Logging to {self.csv_path}")

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def cmd_callback(self, msg):
        self.current_cmd = msg

    def odom_callback(self, msg):
        self.current_odom = msg

    def timer_callback(self):
        vx_cmd = self.current_cmd.linear.x
        wz_cmd = self.current_cmd.angular.z

        vx_odom = self.current_odom.twist.twist.linear.x
        wz_odom = self.current_odom.twist.twist.angular.z

        err_v = vx_cmd - vx_odom
        err_w = wz_cmd - wz_odom

        t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

        self.writer.writerow([t, vx_cmd, vx_odom, wz_cmd, wz_odom, err_v, err_w])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

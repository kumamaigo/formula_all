import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class SubOdomSaver(Node):
    def __init__(self):
        super().__init__('sub_odom_saver')

        # 保存先ファイル
        log_path = '/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/sub_odom.csv'
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        self.csvf = open(log_path, 'w', newline='')
        self.writer = csv.writer(self.csvf)

        # CSVヘッダ
        header = ['time','x','y','z','qx','qy','qz','qw']
        self.writer.writerow(header)

        # /sub_odom購読
        self.create_subscription(Odometry, '/sub_odom', self.cb, 50)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        row = [t, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        self.writer.writerow(row)
        self.csvf.flush()  # 逐次書き込み

def main(args=None):
    rclpy.init(args=args)
    node = SubOdomSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csvf.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

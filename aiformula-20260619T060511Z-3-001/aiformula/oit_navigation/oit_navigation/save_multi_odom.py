# save_multi_odom.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
from message_filters import Subscriber, ApproximateTimeSynchronizer

class MultiOdomSaver(Node):
    def __init__(self):
        super().__init__('multi_odom_saver')

        # CSV 出力準備
        self.csvf = open('/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/multi_odom.csv', 'w', newline='')
        self.writer = csv.writer(self.csvf)
        header = [
            'time',
            # gyro_odom
            'gyro_x','gyro_y','gyro_z','gyro_qx','gyro_qy','gyro_qz','gyro_qw',
            # sub_odom
            'sub_x','sub_y','sub_z','sub_qx','sub_qy','sub_qz','sub_qw',
            # fusion_odom
            'fusion_x','fusion_y','fusion_z','fusion_qx','fusion_qy','fusion_qz','fusion_qw'
        ]
        self.writer.writerow(header)

        # サブスクライバ（message_filters）
        self.sub_gyro = Subscriber(self, Odometry, '/aiformula_sensing/gyro_odometry_publisher/odom')
        self.sub_sub  = Subscriber(self, Odometry, '/sub_odom')
        self.sub_fusion = Subscriber(self, Odometry, '/odom_fusion')

        ats = ApproximateTimeSynchronizer(
            [self.sub_gyro, self.sub_sub, self.sub_fusion],
            queue_size=50,
            slop=0.05  # 50ms 以内なら同じタイムスタンプとして同期
        )
        ats.registerCallback(self.cb)

    def cb(self, msg_gyro, msg_sub, msg_fusion):
        # 基準時刻（gyroのheaderを採用）
        t = msg_gyro.header.stamp.sec + msg_gyro.header.stamp.nanosec * 1e-9

        def extract(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]

        row = [t] + extract(msg_gyro) + extract(msg_sub) + extract(msg_fusion)
        self.writer.writerow(row)
        self.csvf.flush()

def main(args=None):
    rclpy.init(args=args)
    node = MultiOdomSaver()
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

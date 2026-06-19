import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
import math

class PointCloudFollower(Node):
    def __init__(self):
        super().__init__('pointcloud_follower')

        # パラメータ宣言
        self.declare_parameter('pointcloud_topic', '/pointcloud')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        # パラメータ取得
        pc_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value

        # Publisher/Subscriber
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            self.pc_callback,
            10
        )

    def pc_callback(self, msg: PointCloud2):
        # 点群から前方の (x,y) 点を収集
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            if x > 0.0:
                points.append((x, y))

        twist = Twist()
        if not points:
            # 点がなければ停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 平均位置を計算
            avg_x = sum(p[0] for p in points) / len(points)
            avg_y = sum(p[1] for p in points) / len(points)
            # 目標方向
            angle_to_target = math.atan2(avg_y, avg_x)
            # 比例制御
            linear = self.linear_gain * avg_x
            angular = self.angular_gain * angle_to_target
            # 上限クリッピング
            twist.linear.x = max(min(linear, self.max_linear), -self.max_linear)
            twist.angular.z = max(min(angular, self.max_angular), -self.max_angular)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

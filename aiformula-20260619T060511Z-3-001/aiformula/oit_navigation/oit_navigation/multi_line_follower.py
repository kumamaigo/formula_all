import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import math
import numpy as np
import time

class MultiLineFollower(Node):
    def __init__(self):
        super().__init__('multi_line_follower')
        
        #パラメータ宣言
        self.declare_parameter('center_topic', '/aiformula_perception/lane_line_publisher/lane_lines/center')
        self.declare_parameter('left_topic', '/aiformula_perception/lane_line_publisher/lane_lines/left')
        self.declare_parameter('right_topic', '/aiformula_perception/lane_line_publisher/lane_lines/right')
        self.declare_parameter('cmd_vel_topic', '/aiformula_control/lane_line_controller/cmd_vel')
        
        self.declare_parameter('linear_gain', 0.3)
        self.declare_parameter('angular_gain', 0.7)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('lookahead_min_x', 0.0)
        self.declare_parameter('lookahead_max_x', 10.0)
        self.declare_parameter('far_weight_gain', 1.5)
        self.declare_parameter('fallback_speed', 2.0)
        self.declare_parameter('data_timeout', 10.0)
        
        #パラメータ取得
        self.center_topic = self.get_parameter('center_topic').get_parameter_value().string_value
        self.left_topic = self.get_parameter('left_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.lookahead_min = self.get_parameter('lookahead_min_x').get_parameter_value().double_value
        self.lookahead_max = self.get_parameter('lookahead_max_x').get_parameter_value().double_value
        self.far_weight_gain = self.get_parameter('far_weight_gain').get_parameter_value().double_value
        self.fallback_speed = self.get_parameter('fallback_speed').get_parameter_value().double_value
        self.data_timeout = self.get_parameter('data_timeout').get_parameter_value().double_value
        
        #Publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.marker_pub = self.create_publisher(Marker, '/lane_target_marker', 10)
        
        self.pc_center = None
        self.pc_left = None
        self.pc_right = None
        self.last_center_time = 0.0
        self.last_left_time = 0.0
        self.last_right_time = 0.0
        
        #Subscriber
        self.create_subscription(PointCloud2, self.center_topic, self.center_cb, 10)
        self.create_subscription(PointCloud2, self.left_topic, self.left_cb, 10)
        self.create_subscription(PointCloud2, self.right_topic, self.right_cb, 10)
        
        #Timer
        self.timer = self.create_timer(0.05, self.control_loop) #20Hz
        
        self.get_logger().info("MultiLineFollower started.")
        
    def center_cb(self, msg):
        self.pc_center = msg
        #self.get_logger().info(f"msg:{self.pc_center}")
        self.last_center_time = time.time()

        for f in msg.fields:
            self.get_logger().info(f"  name={f.name}, offset={f.offset}, datatype={f.datatype}, count={f.count}")

        # PointCloud2のメタ情報を出力
        #self.get_logger().info(f"Received center line msg: width={msg.width}, height={msg.height}, fields={len(msg.fields)}")

        # データが空でないか確認
        if len(msg.data) == 0:
            self.get_logger().warn("Received PointCloud2 message, but data is empty!")
        else:
            self.get_logger().info(f"Data length: {len(msg.data)} bytes")
        
    def left_cb(self, msg):
        self.pc_left = msg
        self.last_left_time = time.time()
        
    def right_cb(self, msg):
        self.pc_right = msg
        self.last_right_time = time.time()
        
    def is_recent(self, last_time):
        return (time.time() - last_time) < self.data_timeout
        
    def compute_target(self, msg, prefer_far=False):
        points = []
    
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            x, y, z = p
            points.append((x, y))
            
        if not points:
            return None

        xs = np.array([p[0] for p in points])
        ys = np.array([p[1] for p in points])

        if prefer_far:
            weights = np.exp(self.far_weight_gain * (xs / self.lookahead_max))
        else:
            weights = np.ones_like(xs)

        avg_x = np.average(xs, weights=weights)
        avg_y = np.average(ys, weights=weights)

        return avg_x, avg_y

        
    def publish_maker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lane_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        
    def control_loop(self):
        twist = Twist()
        target = None
        
        has_center = self.pc_center and self.is_recent(self.last_center_time)
        has_left = self.pc_left and self.is_recent(self.last_left_time)
        has_right = self.pc_right and self.is_recent(self.last_right_time)
        
        #中央ラインを最優先(遠方重視)
        if has_center:
            center = self.compute_target(self.pc_center, prefer_far = True)
            if center:
                target = center
                self.get_logger().info("Following CENTER line")
                
        #中央ラインがない場合->左ライン
        if target is None and has_left:
            left = self.compute_target(self.pc_left)
            if left:
                target = left
                self.get_logger().info("Following LEFT line")

        #左ラインもない場合->右ライン
        if target is None and has_right:
            right = self.compute_target(self.pc_right)
            if right:
                target = right
                self.get_logger().info("Following RIGHT line")

        if target is None:
            twist.linear.x = self.fallback_speed * 0.5
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().warn("No lines detected")
            return
        
        #Pure Pursuit制御
        tx, ty = target
        angle_to_target = math.atan2(ty, tx)
        
        linear = self.linear_gain * tx
        angular = self.angular_gain * angle_to_target

        if angular > 0.2:
            linear = linear * 0.5
        
        #上限処理
        twist.linear.x = max(min(linear, self.max_linear), -self.max_linear)
        twist.angular.z = max(min(angular, self.max_angular), -self.max_angular)
        
        self.cmd_pub.publish(twist)
        self.publish_maker(tx, ty)
        
def main(args = None):
    rclpy.init(args = args)
    node = MultiLineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
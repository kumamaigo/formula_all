import math
from typing import List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from aiformula_interfaces.msg import ObjectInfoMultiArray

class Cluster:
    def __init__(self):
        self.xs = []
        self.ys = []
        
    def add(self, x, y):
        self.xs.append(x)
        self.ys.append(y)
        
    def center(self):
        return (sum(self.xs) / len(self.xs), sum(self.ys) / len(self.ys))
    
    def min_x(self):
        return min(self.xs) if self.xs else 0.0
    
    def width_y(self):
        return (max(self.ys) - min(self.ys)) if self.ys else 0.0
    
class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__("obstacle_avoider")
        
        #パラメータ宣言
        self.declare_parameter("obstacle_topic", "/aiformula_perception/object_publisher/object_info")
        self.declare_parameter("cmd_vel_topic", "/aiformula_control/red_cone_controller/cmd_vel")
        self.declare_parameter("cmd_vel_topic_line", "/aiformula_control/lane_line_controller/cmd_vel")
        self.declare_parameter("max_angular_speed", 0.3)
        
        #パラメータ取得
        obj_topic = self.get_parameter("obstacle_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        cmd_line_topic = self.get_parameter("cmd_vel_topic_line").get_parameter_value().string_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        
        self.last_objects = []
        self.last_stamp = -1e9
        self.linear_now = 0.0
        
        #Publisher
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        
        #Subscriber
        self.create_subscription(ObjectInfoMultiArray, obj_topic, self.obj_cb, 10)
        self.create_subscription(Twist, cmd_line_topic, self.cmd_cb, 10)
        
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("ObstacleAvoider started")
        
    def now_s(self):
        t = self.get_clock().now().to_msg()
        return float(t.sec) + float(t.nanosec) * 1e-9
    
    def obj_cb(self, msg:ObjectInfoMultiArray):
        self.last_objects = [(o.x, o.y, o.width, o.confidence) for o in msg.objects if o.x > 0.0 and o.confidence > 0.2 and abs(o.y) < 2.0]
        self.last_stamp = self.now_s()

    def cmd_cb(self, msg:Twist):
        self.linear_now = msg.linear.x
    
    def _is_fresh(self, timeout = 0.5):
        return (self.now_s() - self.last_stamp) < timeout
    
    def loop(self):
        tw = Twist()
        fresh = self._is_fresh()
        objs = self.last_objects if fresh else []
        
        if not objs:
                return
        
        #回避対象を一番近い物体にする
        x, y, width, _ = min(objs, key=lambda t: t[0])
        
        clearance = width * 0.5 + 0.5
        target_y = -clearance if y > 0 else +clearance
        
        e = target_y - y
        x_eff = max(x, 0.1)
        
        tw.angular.z = max(min(math.atan2(e, x_eff), self.max_angular_speed), -self.max_angular_speed)
        
        tw.linear.x = max(0.0, self.linear_now * 0.5)
        
        self.cmd_pub.publish(tw)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
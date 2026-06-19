#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math, time

class StraightTest(Node):
    def __init__(self):
        super().__init__('straight_test')
        
        #パラメータ
        self.declare_parameter('cmd_topic', '/aiformula_control/twist_mux/cmd_vel')
        #self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('target_speed', 4.5) #[m/s] 走行開始時の目標速度
        self.declare_parameter('drive_time', 2.7) #[s] 指令速度を維持する時間
        
        p = self.get_parameter
        self.cmd_topic = p('cmd_topic').value
        self.target_speed = float(p('target_speed').value)
        self.drive_time = float(p('drive_time').value)
        
        #Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        
        #制御タイマー
        self.start_time = time.time()
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("StraightTest started")
        
    def control_loop(self):
        t = time.time() - self.start_time
        twist = Twist()
        
        if t < self.drive_time:
            twist.linear.x = self.target_speed
            twist.angular.z = 0.0
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_pub.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = StraightTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass 
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv, os, math, time

class BrakeTest(Node):
    def __init__(self):
        super().__init__('brake_test')
        
        #パラメータ
        self.declare_parameter('cmd_topic', '/aiformula_control/twist_mux/cmd_vel')
        self.declare_parameter('odom_topic', '/sub_odom')
        self.declare_parameter('log_path', '/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/brake_log.csv')
        self.declare_parameter('target_speed', 1.0) #[m/s] 走行開始時の目標速度
        self.declare_parameter('drive_time', 5.0) #[s] 指令速度を維持する時間
        
        p = self.get_parameter
        self.cmd_topic = p('cmd_topic').value
        self.odom_topic = p('odom_topic').value
        self.log_path = p('log_path').value
        self.target_speed = float(p('target_speed').value)
        self.drive_time = float(p('drive_time').value)
        
        #Publisher/Subscriber
        self.cmd_pub = self.create_publisher(Twist, self.cmd_pub, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        
        #CSV初期化
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.csv = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['t[s]', 'v_meas[m/s]', 'a_est[m/s^2]'])
        
        self.prev_time = None
        self.prev_v = None
        self.max_decel = 0.0
        
        #制御タイマー
        self.start_time = time.time()
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("BrakeTest started!")
        
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
        
    def odom_cb(self, msg:Odometry):
        now = time.time()
        v = msg.twist.twist.linear.x
        
        if self.prev_time is not None:
            dt = now - self.prev_time
            
            if dt > 0:
                a = (v - self.prev_v) / dt
                
                self.max_decel = max(self.max_decel, -a)
                self.writer.writerow([round(now - self.start_time, 3), round(v, 3), round(a, 3)])
                self.prev_time = now
                self.prev_v = v
                
    def on_shutdown(self):
        self.csv.close()
        if self.max_decel > 0:
            mu_est = self.max_decel / 9.81
            self.get_logger().info(f'Estimated friction coefficient μ = {mu_est:.2f}')
        else:
            self.get_logger().warn('No deceleration recorded μ not computed.')
            
def main(args=None):
    rclpy.init(args=args)
    node = BrakeTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
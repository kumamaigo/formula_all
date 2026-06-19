#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AccelStraight(Node):
    def __init__(self):
        super().__init__('accel_straight')
        
        #パラメータ
        self.declare_parameter('cmd_topic', '/aiformula_control/twist_mux/cmd_vel')
        self.declare_parameter('target_speed', 2.7) #最高速度[m/s]
        self.declare_parameter('target_angular', 2.5) #旋回半径[m]
        self.declare_parameter('drive_time', 5.0) #走行時間[s]
        self.declare_parameter('accel_time', 1.0) #加速・減速にかける時間[s]
        
        p = self.get_parameter
        self.cmd_topic = p('cmd_topic').value
        self.target_speed = float(p('target_speed').value)
        self.target_angular = float(p('target_angular').value)
        self.drive_time = float(p('drive_time').value)
        self.accel_time = float(p('accel_time').value)
        
        #Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        
        #時間管理
        self.start_time = time.time()
        self.timer = self.create_timer(0.01, self.control_loop)
        
    def control_loop(self):
        t = time.time() - self.start_time
        twist = Twist()
        
        #加速区間
        if t < self.accel_time:
            speed = self.target_speed * (t / self.accel_time)
            angular = speed / self.target_angular
            
        #定速区間
        elif t < self.drive_time - self.accel_time:
            speed = self.target_speed
            angular = speed / self.target_angular
            
        #減速区間
        elif t < self.drive_time:
            remain = self.drive_time - t
            speed = self.target_speed * (remain / self.accel_time)
            angular = speed / self.target_angular

        #完全停止
        else:
            speed = 0.0
            angular = 0.0
            
        twist.linear.x = speed
        twist.angular.z = 0.0
        
        self.cmd_pub.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = AccelStraight()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
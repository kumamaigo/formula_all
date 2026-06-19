#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy.parameter

class FeedbackController(Node):
    def __init__(self):
        super().__init__('feedback_controller')
        
        self.declare_parameter('raw_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/sub_odom')
        self.declare_parameter('feedback_cmd_vel_topic', '/aiformula_control/twist_mux/cmd_vel')
        
        self.declare_parameter('wheel_radius', 0.13) #車輪半径[m]
        self.declare_parameter('tread', 0.7) #トレッド幅[m]
        
        self.declare_parameter('omega_wheel_max', 300.0) #最大車輪速度[rad/s]
        self.declare_parameter('mu', 0.7) #摩擦係数
        self.declare_parameter('v_min', 0.1) #最小線速度[m/s]
        
        self.declare_parameter('Kp_v_PI', 0.6)
        self.declare_parameter('Ki_v_PI', 0.1)
        self.declare_parameter('Kp_v_PD', 0.4)
        self.declare_parameter('Kd_v_PD', 0.02)
        
        self.declare_parameter('Kp_w_PI', 0.8)
        self.declare_parameter('Ki_w_PI', 0.1)
        self.declare_parameter('Kp_w_PD', 0.9)
        self.declare_parameter('Kd_w_PD', 0.05)
        
        self.declare_parameter('omega_low', 0.05)
        self.declare_parameter('omega_high', 0.2)
        self.declare_parameter('cycle_time', 0.01) #100Hz
        
        self.declare_parameter('integrator_limit', 0.5) #最大積分量
        self.declare_parameter('derivative_alpha', 0.2) #ローパスフィルタ
        
        p = self.get_parameter
        
        self.raw_topic = p('raw_cmd_vel_topic').value
        self.odom_topic = p('odom_topic').value
        self.feedback_topic = p('feedback_cmd_vel_topic').value
        
        self.r = p('wheel_radius').value
        self.L = p('tread').value
        
        self.omega_wheel_max = p('omega_wheel_max').value
        self.mu = p('mu').value
        self.v_min= p('v_min').value
        
        self.Kp_v_PI = p('Kp_v_PI').value
        self.Ki_v_PI = p('Ki_v_PI').value
        self.Kp_v_PD = p('Kp_v_PD').value
        self.Kd_v_PD = p('Kd_v_PD').value
        
        self.Kp_w_PI = p('Kp_w_PI').value
        self.Ki_w_PI = p('Ki_w_PI').value
        self.Kp_w_PD = p('Kp_w_PD').value
        self.Kd_w_PD = p('Kd_w_PD').value
        
        self.omega_low = p('omega_low').value
        self.omega_high = p('omega_high').value
        self.dt = p('cycle_time').value
        
        self.int_limit = p('integrator_limit').value
        self.alpha_d = p('derivative_alpha').value
        
        self.v_ref = 0.0
        self.w_ref = 0.0
        self.v_meas = 0.0
        self.w_meas = 0.0
        self.I_v = 0.0
        self.I_w = 0.0
        self.prev_e_v = 0.0
        self.prev_e_w = 0.0
        self.D_v_filt = 0.0
        self.D_w_filt = 0.0
        
        self.pub_cmd = self.create_publisher(Twist, self.feedback_topic, 10)
        self.sub_ref = self.create_subscription(Twist, self.raw_topic, self.ref_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_step)
        
        self.add_on_set_parameters_callback(self.dynamic_reconfigure_cb)
        
    def dynamic_reconfigure_cb(self, params):
        for p in params:
            if hasattr(self, p.name):
                setattr(self, p.name, p.value)
        
        return rclpy.parameter.SetParameterResult(successful=True)
    
    def ref_callback(self, msg: Twist):
        self.v_ref = msg.linear.x
        self.w_ref = msg.angular.z
        
    def odom_callback(self, msg: Odometry):
        self.v_meas = msg.twist.twist.linear.x
        self.w_meas = msg.twist.twist.angular.z
        
    def control_step(self):
        e_v = self.v_ref - self.v_meas
        e_w = self.w_ref - self.w_meas
        
        self.I_v = max(min(self.I_v + e_v * self.dt, self.int_limit), -self.int_limit)
        self.I_w = max(min(self.I_w + e_w * self.dt, self.int_limit), -self.int_limit)
        
        raw_Dv = (e_v - self.prev_e_v) / max(self.dt, 1e-6)
        raw_Dw = (e_w - self.prev_e_w) / max(self.dt, 1e-6)
        self.D_v_filt += self.alpha_d * (raw_Dv -self.D_v_filt)
        self.D_w_filt += self.alpha_d * (raw_Dw - self.D_w_filt)
        self.prev_e_v = e_v
        self.prev_e_w = e_w
        
        ratio = (abs(self.w_meas) - self.omega_low) / max(self.omega_high - self.omega_low, 1e-6)
        ratio = min(max(ratio, 0.0), 1.0)
        
        u_v_PI = self.Kp_v_PI * e_v + self.Ki_v_PI * self.I_v
        u_v_PD = self.Kp_v_PD * e_v + self.Kd_v_PD * self.D_v_filt
        u_v = (1 - ratio) * u_v_PI + ratio * u_v_PD
        
        u_w_PI = self.Kp_w_PI * e_w + self.Ki_w_PI * self.I_w
        u_w_PD = self.Kp_w_PD * e_w + self.Kd_w_PD * self.D_w_filt
        u_w = (1 - ratio) * u_w_PI + ratio * u_w_PD
        
        v_cmd = self.v_ref + u_v
        w_cmd = self.w_ref + u_w
        
        a_long = (v_cmd - self.v_meas) / max(self.dt, 1e-6)
        rem = max((self.mu * 9.81) ** 2 - a_long ** 2, 0.0)
        w_limit = math.sqrt(rem) / max(abs(self.v_meas), self.v_min)
        w_cmd = max(min(w_cmd, w_limit), -w_limit)
        
        w_r = (v_cmd + (self.L / 2.0) * w_cmd) / self.r
        w_l = (v_cmd - (self.L / 2.0) * w_cmd) / self.r
        max_wheel = max(abs(w_r), abs(w_l))
        
        if(max_wheel > self.omega_wheel_max):
            scale = self.omega_wheel_max / max_wheel
            v_cmd *= scale
            w_cmd *= scale
            
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = FeedbackController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
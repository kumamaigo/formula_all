#!/usr/bin/env python3

import math
import csv
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import SetParametersResult
from time import time

class PIDFeedbackController(Node):
    def __init__(self):
        super().__init__('pid_feedback_controller')
        
        self.declare_parameter('raw_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/sub_odom')
        self.declare_parameter('feedback_cmd_vel_topic', '/aiformula_control/twist_mux/cmd_vel')
        self.declare_parameter('log_path', '/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/feedback_log.csv')
        
        self.declare_parameter('wheel_radius', 0.13)
        self.declare_parameter('tread', 0.7)
        
        self.declare_parameter('omega_wheel_max', 90.0)
        self.declare_parameter('mu', 0.7)
        self.declare_parameter('v_min', 0.05)
        
        self.declare_parameter('Kp_v_st', 0.6)
        self.declare_parameter('Ki_v_st', 0.0)
        self.declare_parameter('Kd_v_st', 0.02)
        
        self.declare_parameter('Kp_w_st', 0.8)
        self.declare_parameter('Ki_w_st', 0.02)
        self.declare_parameter('Kd_w_st', 0.02)
        
        self.declare_parameter('Kp_v_turn', 0.4)
        self.declare_parameter('Ki_v_turn', 0.0)
        self.declare_parameter('Kd_v_turn', 0.01)
        
        self.declare_parameter('Kp_w_turn', 0.9)
        self.declare_parameter('Ki_w_turn', 0.05)
        self.declare_parameter('Kd_w_turn', 0.05)
        
        self.declare_parameter('omega_low', 0.4)
        self.declare_parameter('omega_high', 0.8)
        
        self.declare_parameter('cycle_time', 0.01)
        self.declare_parameter('integrator_limit', 0.5)
        self.declare_parameter('derivative_alpha', 0.2)
        
        p = self.get_parameter
        
        self.raw_topic = p('raw_cmd_vel_topic').value
        self.odom_topic = p('odom_topic').value
        self.feedback_topic = p('feedback_cmd_vel_topic').value
        self.log_path = p('log_path').value
        
        self.r = float(p('wheel_radius').value)
        self.L = float(p('tread').value)
        
        self.omega_wheel_max = float(p('omega_wheel_max').value)
        self.mu = float(p('mu').value)
        self.v_min = float(p('v_min').value)
        
        self.Kp_v_st = float(p('Kp_v_st').value)
        self.Ki_v_st = float(p('Ki_v_st').value)
        self.Kd_v_st = float(p('Kd_v_st').value)
        
        self.Kp_w_st = float(p('Kp_w_st').value)
        self.Ki_w_st = float(p('Ki_w_st').value)
        self.Kd_w_st = float(p('Kd_w_st').value)
        
        self.Kp_v_turn = float(p('Kp_v_turn').value)
        self.Ki_v_turn = float(p('Ki_v_turn').value)
        self.Kd_v_turn = float(p('Kd_v_turn').value)
        
        self.Kp_w_turn = float(p('Kp_w_turn').value)
        self.Ki_w_turn = float(p('Ki_w_turn').value)
        self.Kd_w_turn = float(p('Kd_w_turn').value)
        
        self.omega_low = float(p('omega_low').value)
        self.omega_high = float(p('omega_high').value)
        
        self.dt = float(p('cycle_time').value)
        self.int_limit = float(p('integrator_limit').value)
        self.alpha_d = float(p('derivative_alpha').value)
        
        if self.omega_high <= self.omega_low:
            self.get_logger().warn("omega_high <= omega_low: adjusting omega_high")
            self.omega_high = self.omega_high + 1e-3
            
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
        
        self.mode_straight = True
        
        self.pub_cmd = self.create_publisher(Twist, self.feedback_topic, 10)
        self.sub_ref = self.create_subscription(Twist, self.raw_topic, self.ref_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.control_step)
        
        self.add_on_set_parameters_callback(self.dynamic_reconfigure_cb)
        
        os.makedirs(os.path.dirname(self.log_path), exist_ok = True)
        self.csv_file = open(self.log_path, 'w', newline = '')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            'time[s]', 'v_ref', 'w_ref', 'v_meas', 'w_meas', 'e_v', 'e_w', 'u_v', 'u_w'
        ])
        
        self.get_logger().info("PIDFeedbackController started")
        
    def dynamic_reconfigure_cb(self, params):
        for param in params:
            name = param.name
            if hasattr(self, name):
                try:
                    setattr(self, name, param.value)
                except Exception as e:
                    self.get_logger().warn(f"param set fail {name}: {e}")
        return SetParametersResult(successful=True)
    
    def ref_callback(self, msg: Twist):
        self.v_ref = float(msg.linear.x)
        self.w_ref = float(msg.angular.z)
        
    def odom_callback(self, msg: Odometry):
        self.v_meas = float(msg.twist.twist.linear.x)
        self.w_meas = float(msg.twist.twist.angular.z)
        
    @staticmethod
    def clip(x, lo, hi):
        return max(lo, min(hi, x))
    
    def select_gains(self):
        abs_w = abs(self.w_meas)
        
        if self.mode_straight:
            if abs_w >= self.omega_high:
                self.mode_straight = False
                
        else:
            if abs_w <= self.omega_low:
                self.mode_straight = True
                
        if self.mode_straight:
            return (self.Kp_v_st, self.Ki_v_st, self.Kd_v_st, self.Kp_w_st, self.Ki_w_st, self.Kd_w_st, 'STRAIGHT')
        
        else:
            return (self.Kp_v_turn, self.Ki_v_turn, self.Kd_v_turn, self.Kp_w_turn, self.Ki_w_turn, self.Kd_w_turn, 'TURN')
        
    def control_step(self):
        e_v = self.v_ref - self.v_meas
        e_w = self.w_ref - self.w_meas
        
        raw_Dv = (e_v - self.prev_e_v) / max(self.dt, 1e-9)
        raw_Dw = (e_w - self.prev_e_w) / max(self.dt, 1e-9)
        
        self.D_v_filt += self.alpha_d * (raw_Dv - self.D_v_filt)
        self.D_w_filt += self.alpha_d * (raw_Dw - self.D_w_filt)
        
        self.prev_e_v = e_v
        self.prev_e_w = e_w
        
        (Kp_v, Ki_v, Kd_v, Kp_w, Ki_w, Kd_w, mode_name) = self.select_gains()
        
        self.I_v = self.clip(self.I_v + e_v * self.dt, -self.int_limit, self.int_limit)
        self.I_w = self.clip(self.I_w + e_w * self.dt, -self.int_limit, self.int_limit)
        
        u_v = Kp_v * e_v + Ki_v * self.I_v + Kd_v * self.D_v_filt
        u_w = Kp_w * e_w + Ki_w * self.I_w + Kd_w * self.D_w_filt
        
        v_cmd = float(self.v_ref + u_v)
        w_cmd = float(self.w_ref + u_w)
        
        a_long = (v_cmd - self.v_meas) / max(self.dt, 1e-9)
        rem = max((self.mu * 9.81) ** 2 - a_long ** 2, 0.0)
        denom = max(abs(self.v_meas), self.v_min)
        w_limit = math.sqrt(rem) / denom if denom > 0.0 else float('inf')
        w_cmd = self.clip(w_cmd, -w_limit, w_limit)
        
        if abs(self.r) < 1e-9:
            self.get_logger().error("wheel_radius too small or zero")
            return
        
        w_r = (v_cmd + (self.L / 2.0) * w_cmd) / self.r
        w_l = (v_cmd - (self.L / 2.0) * w_cmd) / self.r
        max_wheel = max(abs(w_r), abs(w_l))
        if max_wheel > 1e-9 and max_wheel > self.omega_wheel_max:
            scale = self.omega_wheel_max / max_wheel
            v_cmd *= scale
            w_cmd *= scale
            
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(w_cmd)
        self.pub_cmd.publish(twist)
        
        self.csv_writer.writerow([
            round(time(), 3), round(self.v_ref, 3), round(self.w_ref, 3), round(self.v_meas, 3), round(self.w_meas, 3), round(e_v, 3), round(e_w, 3), round(u_v, 3), round(u_w, 3) 
        ])
        
        self.csv_file.flush()
        
    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = PIDFeedbackController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
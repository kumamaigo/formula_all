import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class PIFeedbackController(Node):
    def __init__(self):
        super().__init__('pi_feedback_controller')
        
        #トピック
        self.declare_parameter('raw_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('omega_topic', '/omega_topic')
        self.declare_parameter('feedback_cmd_vel_topic', 'aiformula_control/twist_mux/cmd_vel')
        
        #基本パラメータ
        self.declare_parameter('x_c', -0.30)  # 従属輪のx座標[m]
        self.declare_parameter('y_c', 0.0)    # 従属輪のy座標[m]
        self.declare_parameter('wheel_radius', 0.0508) # [m]
        self.declare_parameter('tread', 0.30) # [m]
        self.declare_parameter('omega_wheel_max', 300.0) # [rad/s]
        self.declare_parameter('mu', 0.7) #道路の摩擦係数
        self.declare_parameter('v_min', 0.1) #最低線速度[m/s]
        
        #ゲイン
        self.declare_parameter('Kp_v', 0.6)
        self.declare_parameter('Ki_v', 0.1)
        self.declare_parameter('Kp_ang', 0.8)
        self.declare_parameter('Ki_ang', 0.1)
        
        #フィルタ, 周期
        self.declare_parameter('alpha_vx', 0.85)
        self.declare_parameter('alpha_vy', 0.85)
        self.declare_parameter('alpha_omega', 0.8)
        self.declare_parameter('cycle_time', 0.01)
        self.declare_parameter('radius', 0.0508)
        
        # パラメータ取得
        raw_topic = self.get_parameter('raw_cmd_vel_topic').get_parameter_value().string_value
        omega_topic = self.get_parameter('omega_topic').get_parameter_value().string_value
        feedback_topic = self.get_parameter('feedback_cmd_vel_topic').get_parameter_value().string_value

        self.x_c = self.get_parameter('x_c').get_parameter_value().double_value
        self.y_c = self.get_parameter('y_c').get_parameter_value().double_value
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.L = self.get_parameter('tread').get_parameter_value().double_value
        self.omega_wheel_max = self.get_parameter('omega_wheel_max').get_parameter_value().double_value
        self.mu = self.get_parameter('mu').get_parameter_value().double_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value

        self.Kp_v = self.get_parameter('Kp_v').get_parameter_value().double_value
        self.Ki_v = self.get_parameter('Ki_v').get_parameter_value().double_value
        self.Kp_ang = self.get_parameter('Kp_ang').get_parameter_value().double_value
        self.Ki_ang = self.get_parameter('Ki_ang').get_parameter_value().double_value

        self.alpha_vx = self.get_parameter('alpha_vx').get_parameter_value().double_value
        self.alpha_vy = self.get_parameter('alpha_vy').get_parameter_value().double_value
        self.alpha_omega = self.get_parameter('alpha_omega').get_parameter_value().double_value
        self.cycle_time = self.get_parameter('cycle_time').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        
        #初期化
        self.v_ref = 0.0
        self.omega_ref = 0.0
        self.vx_f = 0.0
        self.vy_f = 0.0
        self.omega_est_f = 0.0
        self.v_body_f = 0.0
        self.vy_body_f = 0.0
        self.e_v_int = 0.0
        self.e_ang_int = 0.0
        
        # Publisher/Subscriber
        self.pub_cmd = self.create_publisher(Twist, feedback_topic, 10)
        self.sub_ref = self.create_subscription(Twist, raw_topic, self.ref_callback, 10)
        self.sub_omega = self.create_subscription(Twist, omega_topic, self.omega_callback, 10)
        
        # timer
        self.timer = self.create_timer(self.cycle_time, self.control_step)
        
    def lowpass(self, x_prev, x_meas, alpha):
        return alpha * x_prev + (1.0 - alpha) * x_meas
    
    def ref_callback(self, msg:Twist):
        self.v_ref = msg.linear.x
        self.omega_ref = msg.angular.z
        
    def omega_callback(self, msg:Twist):
        vx_c = msg.linear.x * self.r
        vy_c = msg.linear.y * self.r
        
        omega_raw = vy_c / self.x_c
        
        self.omega_est_f = self.lowpass(self.omega_est_f, omega_raw, self.alpha_omega)
        self.vx_f = self.lowpass(self.vx_f, vx_c, self.alpha_vx)
        self.vy_f = self.lowpass(self.vy_f, vy_c, self.alpha_vy)
        
        v_body = self.vx_f + self.omega_est_f * self.y_c
        vy_body = self.vy_f - self.omega_est_f * self.x_c
        
        self.v_body_f = self.lowpass(self.v_body_f, v_body, self.alpha_vx)
        self.vy_body_f = self.lowpass(self.vy_body_f, vy_body, self.alpha_vy)
        
    def control_step(self):
        v_meas = self.v_body_f
        omega_meas = self.omega_est_f
        
        #誤差計算
        e_v = self.v_ref - v_meas
        e_ang = self.omega_ref - omega_meas
        
        #積分
        self.e_v_int += e_v * self.cycle_time
        self.e_ang_int += e_ang * self.cycle_time
        
        self.e_v_int = max(min(self.e_v_int, 10.0), -10.0)
        self.e_ang_int = max(min(self.e_ang_int, 10.0), -10.0)
        
        u_v = self.Kp_v * e_v + self.Ki_v * self.e_v_int
        u_ang = self.Kp_ang * e_ang + self.Ki_ang * self.e_ang_int
        
        v_cmd = self.v_ref + u_v
        omega_cmd = self.omega_ref + u_ang
        
        a_long = (v_cmd - v_meas) / max(self.cycle_time, 1e-6)
        rem = max((self.mu * 9.81) ** 2 - a_long ** 2, 0.0)
        omega_limit_mu = math.sqrt(rem) / max(abs(v_meas), self.v_min)
        
        if omega_cmd > omega_limit_mu:
            omega_cmd = omega_limit_mu
        elif omega_cmd < -omega_limit_mu:
            omega_cmd = -omega_limit_mu
            
        omega_r = (v_cmd + (self.L / 2.0) * omega_cmd) / self.r
        omega_l = (v_cmd - (self.L / 2.0) * omega_cmd) / self.r
        max_wheel = max(abs(omega_r), abs(omega_l))
        
        if max_wheel > self.omega_wheel_max and max_wheel > 1e-6:
            scale = self.omega_wheel_max / max_wheel
            v_cmd *= scale
            omega_cmd *= scale
            
        twist = Twist()
        twist.linear.x = float(v_cmd)
        twist.angular.z = float(omega_cmd)
        self.pub_cmd.publish(twist)
        
def main(args=None):
    rclpy.init(args=args)
    node = PIFeedbackController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PDFeedbackController(Node):
    def __init__(self):
        super().__init__('pd_feedback_controller')
        
        #パラメータ宣言
        self.declare_parameter('raw_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('omega_topic', '/omega')
        self.declare_parameter('feedback_cmd_vel_topic', 'aiformula_control/twist_mux/cmd_vel')
        self.declare_parameter('proportional_linear_gain', 1.0)
        self.declare_parameter('proportional_angular_gain', 1.0)
        self.declare_parameter('differential_linear_gain', 1.0)
        self.declare_parameter('differential_angular_gain', 1.0)
        self.declare_parameter('lpf_alpha', 0.2)
        self.declare_parameter('cycle_time', 0.01)
        
        #パラメータ取得
        raw_topic = self.get_parameter('raw_cmd_vel_topic').get_parameter_value().string_value
        omega_topic = self.get_parameter('omega_topic').get_parameter_value().string_value
        feedback_topic = self.get_parameter('feedback_cmd_vel_topic').get_parameter_value().string_value
        self.proportional_linear_gain = self.get_parameter('proportional_linear_gain').get_parameter_value().double_value
        self.proportional_angular_gain = self.get_parameter('proportional_angular_gain').get_parameter_value().double_value
        self.differential_linear_gain = self.get_parameter('differential_linear_gain').get_parameter_value().double_value
        self.differential_angular_gain = self.get_parameter('differential_angular_gain').get_parameter_value().double_value
        self.lpf_alpha = self.get_parameter('lpf_alpha').get_parameter_value().double_value
        self.cycle_time = self.get_parameter('cycle_time').get_parameter_value().double_value
        
        #Publisher/Subscriber
        self.cmd_pub = self.create_publisher(Twist, feedback_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, raw_topic, self.raw_callback, 10)
        self.omega_sub = self.create_subscription(Twist, omega_topic, self.omega_callback, 10)
        
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.error_linear_vel_prev = 0.0
        self.error_angular_vel_prev = 0.0
        self.measured_linear_vel = 0.0
        self.measured_angular_vel = 0.0
        self.filtered_linear_vel = 0.0
        self.filtered_angular_vel = 0.0
        
        self.timer = self.create_timer(self.cycle_time, self.timer_callback)
        
    def raw_callback(self, msg:Twist):
        #目標速度の取得
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z
        
    def omega_callback(self, msg:Twist):
        #計測速度の取得
        raw_linear = msg.linear.x
        raw_angular = msg.linear.y
        
        #ローパスフィルタ適用
        self.filtered_linear_vel = self.lpf_alpha * raw_linear + (1 - self.lpf_alpha) * self.filtered_linear_vel
        self.filtered_angular_vel = self.lpf_alpha * raw_angular + (1 - self.lpf_alpha) * self.filtered_angular_vel
        
        self.measured_linear_vel = self.filtered_linear_vel
        self.measured_angular_vel = self.filtered_angular_vel
        
    def timer_callback(self):
        error_linear_vel = self.target_linear_vel - self.measured_linear_vel
        error_angular_vel = self.target_angular_vel - self.measured_angular_vel
        
        control_linear = self.proportional_linear_gain * error_linear_vel + self.differential_linear_gain * (error_linear_vel - self.error_linear_vel_prev) / self.cycle_time
        control_angular = self.proportional_angular_gain * error_angular_vel + self.differential_angular_gain * (error_angular_vel - self.error_angular_vel_prev) /self.cycle_time
        
        twist = Twist()
        
        twist.linear.x = self.target_linear_vel + control_linear
        twist.angular.z = self.target_angular_vel + control_angular
        
        self.cmd_pub.publish(twist)
        
        self.error_linear_vel_prev = error_linear_vel
        self.error_angular_vel_prev = error_angular_vel
        
def main(args=None):
    rclpy.init(args=args)
    node = PDFeedbackController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
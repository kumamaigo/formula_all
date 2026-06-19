import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 # パネル幅(pixel)を受け取る
import time

class PanelDetectionNavigator(Node):
    def __init__(self):
        super().__init__('panel_navigator')

        #パラメータ宣言
        #self.declare_parameter('cmd_vel_topic', '/aiformula_control/lane_line_controller/cmd_vel')
        self.declare_parameter('real_panel_width', 0.19)#変える
        self.declare_parameter('focal_length_px', 189.5)#変える
        # 制御用パラメータ
        #self.declare_parameter('distance_threshold', 10.0)  # 何メートルまで近づくか
        #self.declare_parameter('max_linear_speed', 0.5)    # 前進最大速度
        #self.declare_parameter('turning_speed', 0.3)       # 回避時の旋回速度
        #self.declare_parameter('data_timeout', 1.0)        # 検出中断時のタイムアウト(秒)
        #パラメータ取得
        #cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.real_panel_width = self.get_parameter('real_panel_width').get_parameter_value().double_value
        self.focal_length = self.get_parameter('focal_length_px').get_parameter_value().double_value
        #self.dist_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        #self.max_linear = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        #self.turn_speed = self.get_parameter('turning_speed').get_parameter_value().double_value
        #self.data_timeout = self.get_parameter('data_timeout').get_parameter_value().double_value
        #変数初期化
        self.current_pixel_width = 0.0
        self.last_detection_time = 0.0
        #通信の設定
        # distance_estimation_from_pixel_width_pub_formula_nodeから送られてくるパネルの「ピクセル幅」をサブスクライブ
        self.create_subscription(Float32, '/detected_panel_width_px', self.panel_callback, 10)
        #self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        # 制御ループ (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Panel Navigator Started')

    def panel_callback(self, msg):
        self.current_pixel_width = msg.data
        if self.current_pixel_width > 0:
            self.last_detection_time = time.time()
        
        if self.current_pixel_width <= 0:
            return
    
    #検出データが新鮮かどうか
    def is_recent(self):
        return (time.time() - self.last_detection_time) < self.data_timeout
    
    #メイン制御ループ
    def control_loop(self):
        if self.current_pixel_width <= 0:
            # self.get_logger().info("Waiting for panel detection...", throttle_duration_sec=2.0)
            return
        #twist = Twist()
        #データが新鮮でない、または検出されていない場合の処理
        #if not self.is_recent() or self.current_pixel_width <= 0:
            #twist.linear.x = 0.0
            #twist.angular.z = 0.0
            #self.cmd_pub.publish(twist)
            # self.get_logger().warn("Panel not detected or data timeout - Stopping", once=True)
            #return
        #距離の推定
        estimated_dist = (self.real_panel_width * self.focal_length) / self.current_pixel_width
        self.get_logger().info(f'Estimated Distance: {estimated_dist:.2f}m', throttle_duration_sec=1.0)

        #制御命令の生成
        #if estimated_dist > self.dist_threshold:
            # 遠い場合はターゲットに向かって前進
            #twist.linear.x = self.max_linear
            #twist.angular.z = 0.0
            #self.get_logger().info("yet")
        #else:
            # 近づきすぎたら回避行動 (左折など)
            #twist.linear.x = self.max_linear * 0.5
            #twist.angular.z = self.turn_speed
            #self.get_logger().warn("Threshold reached! Turning")

        #上限処理とパブリッシュ
        #twist.linear.x = max(min(twist.linear.x, self.max_linear), -self.max_linear)
        #twist.angular.z = max(min(twist.angular.z, self.turn_speed), -self.turn_speed)
        
        #self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = PanelDetectionNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

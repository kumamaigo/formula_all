import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RealTimeLightPanelNode(Node):
    def __init__(self):
        super().__init__('realtime_light_panel_node')
        self.get_logger().info('Real-Time Light Panel Detection Node with Publisher Started')
        
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # 画像のサブスクライバ
        self.subscription = self.create_subscription(Image, '/aiformula_sensing/zed_node/rgb/image_rect_color', self.image_callback, 10)
        #self.subscription = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, qos_profile)

        # パネル幅(pixel)のパブリッシャ
        self.width_pub = self.create_publisher(Float32, '/detected_panel_width_px', 10)

        cv2.namedWindow("Light Panel Detection")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        processed_image, max_width = self.process_image_realtime(cv_image)
        
        # 横幅をパブリッシュ (検出されなかった場合は 0.0 を送る)
        width_msg = Float32()
        width_msg.data = float(max_width)
        self.width_pub.publish(width_msg)
        
        cv2.imshow("Light Panel Detection", processed_image)
        cv2.waitKey(1)

    def process_image_realtime(self, bgr_image):
        draw_image = bgr_image.copy()
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        
        max_w = 0.0
        max_area = 0.0

        # 色範囲の定義
        lower_green = np.array([60, 150, 140])
        upper_green = np.array([85, 255, 255])
        lower_red = np.array([0, 200, 170])
        upper_red = np.array([10, 255, 255])
        
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        
        # 緑と赤のマスクを合成（どちらかの色が検出されればOKとする場合）
        combined_mask = cv2.bitwise_or(mask_green, mask_red)
        
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                
                #描画
                cv2.rectangle(draw_image, (x, y), (x+w, y+h), (255, 255, 0), 3)
                cv2.putText(draw_image, f"W: {w}px", (x, y-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                # 最も面積が大きいものを「ターゲットのパネル」として横幅を保持
                if area > max_area:
                    max_area = area
                    max_w = float(w)

        return draw_image, max_w

def main(args=None):
    rclpy.init(args=args)
    node = RealTimeLightPanelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

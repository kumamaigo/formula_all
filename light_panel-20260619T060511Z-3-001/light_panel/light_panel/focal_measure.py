import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#領域の大きさで区別

class RealTimeLightPanelNode(Node):
    def __init__(self):
        super().__init__('realtime_light_panel_node')
        self.get_logger().info('Real-Time Light Panel Detection Node Started')
        
        # ROS <-> OpenCV 変換用のブリッジ
        self.bridge = CvBridge()

        # USBカメラからの画像トピック（通常は /image_raw または /usb_cam/image_raw）
        # `usb_cam` ノードを起動しておく必要があります。
        self.subscription = self.create_subscription(
            Image,
            '/aiformula_sensing/zed_node/rgb/image_rect_color', 
            self.image_callback,
            10)
        self.subscription # prevent unused variable warning

        # 判定結果を表示するためのOpenCVウィンドウ
        cv2.namedWindow("Light Panel Detection")

    def image_callback(self, msg):
        """カメラ画像を受信したときに呼ばれるコールバック"""
        try:
            # ROS Image メッセージを OpenCV の BGR 画像に変換
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # --- [画像処理による色検出] ---
        # OpenCVで解析（HSV変換、マスク処理）
        processed_image = self.process_image_realtime(cv_image)
        
        # --- [結果の表示] ---
        # 処理後の画像をOpenCVのウィンドウに表示
        cv2.imshow("Light Panel Detection", processed_image)
        cv2.waitKey(1) # これがないとウィンドウが更新されません

    def process_image_realtime(self, bgr_image):
        """BGR画像を入力とし、色検出して矩形を描画した画像を返す"""
        
        # 描画用の複製
        draw_image = bgr_image.copy()

        # HSV色空間に変換
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        
        # --- [色の定義] ---
        # モニターに映した画像に対するHSV範囲（※調整が必要になることが多いです）
        # 前回の定義より少し広め（明るさVを少し落とすなど）に設定します。

        # 緑色(H:69-73,S:160-224,V:150-220)
        lower_green = np.array([60, 150, 140])
        upper_green = np.array([85, 255, 255])
        
        # 赤色(H:0-1,S:237-241,V:191-216)
        lower_red = np.array([0, 200, 170])
        upper_red = np.array([10, 255, 255])
        
        # --- [マスク処理] ---
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        
        # --- [領域の検出と描画（緑）] ---
        # マスク画像から「輪郭（Contours）」を検索
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 1000: # ある程度の大きさの領域だけを処理
                # 輪郭を囲む外接矩形を計算
                x, y, w, h = cv2.boundingRect(contour)
                self.get_logger().info(f'Detected GREEN: Width={w}, Area={area}')
                # 元画像に矩形を描画（緑色、太さ3）
                cv2.rectangle(draw_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
                # テキストも描画
                cv2.putText(draw_image, "GREEN", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        # --- [領域の検出と描画（赤）] ---
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(contour)
                self.get_logger().info(f'Detected RED: Width={w}, Area={area}')
                # 元画像に矩形を描画（赤色、太さ3）
                cv2.rectangle(draw_image, (x, y), (x+w, y+h), (0, 0, 255), 3)
                #cv2.putText(draw_image, "RED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                cv2.putText(draw_image, f"W: {w}px", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        return draw_image

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
        # OpenCVのウィンドウを閉じる
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

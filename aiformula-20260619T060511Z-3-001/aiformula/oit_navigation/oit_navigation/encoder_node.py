import sys
import time
import serial
import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

DEFAULT_PORT = '/dev/ttyACM0'
BAUD = 115200

#Windows PC側のIPとポート
UDP_IP = "192.168.10.135" #適宜書き換え
UDP_PORT = 5005

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        
        #Publisher
        self.omega_pub = self.create_publisher(Twist, '/omega', 10)
        
        #UDPソケット
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        #初期化
        self.snapshot1 = [0, 0, 0, 0, 0]
        self.snapshot2 = [0, 0, 0, 0, 0]
        self.labels = ['count_total', 'rad', 'omega', 'rps', 'rpm']
        
    def spin(self):
        self.get_logger().info(f"Opening {DEFAULT_PORT} at {BAUD}bps. Ctrl-C to exit.")
        
        try:
            with serial.Serial(DEFAULT_PORT, baudrate = BAUD, timeout = 0.01) as ser:
                time.sleep(0.1)
                
                while rclpy.ok():
                    start = time.time()
                    
                    try:
                        line = ser.readline()
                        
                        if line:
                            try:
                                text = line.decode('utf-8', errors = 'replace').rstrip('\r\n')
                            
                            except Exception:
                                text = str(line)
                                
                            s = text.lstrip('>').strip()
                            
                            if ':' in s:
                                key, val_str = s.split(':', 1)
                                key = key.strip()
                                val_str = val_str.strip()
                                kl = key.lower()
                                
                                #どちらのエンコーダか判断
                                target = None
                                if kl.startswith('e1') or kl.startswith('l') or 'left' in kl:
                                    target = 1
                                    
                                elif kl.startswith('e2') or kl.startswith('r') or 'right' in kl:
                                    target = 2
                                    
                                #キー名正規化
                                norm = kl
                                if '_' in norm:
                                    parts = norm.split('_', 1)
                                    
                                    if parts[0] in ('e1', 'e2', 'l', 'r'):
                                        norm = parts[1]
                                        
                                else:
                                    if norm.startswith('e1'):
                                        norm = norm[2:].lstrip('_')
                                        
                                    elif norm.startswith('e2'):
                                        norm = norm[2:].lstrip('_')
                                        
                                    elif norm.startswith('l') and len(norm) > 1:
                                        norm = norm[1:].lstrip('_')
                                    
                                    elif norm.startswith('r') and len(norm) > 1:
                                        norm = norm[1:].lstrip('_')
                                        
                                #値を格納
                                try:
                                    dst = self.snapshot1 if target == 1 else self.snapshot2 if target == 2 else None
                                    
                                    if dst is not None:
                                        if 'count_total' in norm:
                                            dst[0] = int(float(val_str))
                                            
                                        elif 'rad/s' in kl or 'ω' in key or 'omega' in norm:
                                            dst[2] = float(val_str)
                                            
                                        elif norm.startswith('rad'):
                                            dst[1] = float(val_str)
                                            
                                        elif 'rps' in norm:
                                            dst[3] = float(val_str)
                                            
                                        elif 'rpm' in norm:
                                            dst[4] = float(val_str)
                                            
                                except Exception as e:
                                    self.get_logger().warn(f'Parse value error: {e}')
                                    
                        #UDP送信(E1+E2)
                        data = self.snapshot1 + self.snapshot2
                        message = struct.pack('i f f f f i f f f f', *data)
                        self.sock.sendto(message, (UDP_IP, UDP_PORT))
                        
                        #ROSでomegaをpublish
                        twist = Twist()
                        twist.linear.x = float(self.snapshot1[2]) #E1のomega
                        twist.linear.y = float(self.snapshot2[2]) #E2のomega
                        twist.angular.x = float(self.snapshot1[1])
                        twist.angular.y = float(self.snapshot2[1])
                        self.omega_pub.publish(twist)
                        
                        #周期調整(10ms送信)
                        elapsed = time.time() - start
                        
                        if elapsed < 0.01:
                            time.sleep(0.01 - elapsed)
                            
                    except KeyboardInterrupt:
                        self.get_logger().info("Exiting...")
                        break
                    
                    except Exception as e:
                        self.get_logger().info(f"Read error: {e}")
                        break
                    
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdRemap(Node):
    def __init__(self):
        super().__init__('cmd_remap')

        self.cmd_pub = self.create_publisher(Twist, '/aiformula_control/twist_mux/cmd_vel', 10)

        #self.cmd_sub = self.create_subscription(Twist, '/aiformula_control/key_board_controller/cmd_vel', self.cmd_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

    def cmd_callback(self, msg: Twist):
        twist = Twist()
        twist.linear.x = msg.linear.x
        twist.angular.z = msg.angular.z

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CmdRemap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()     

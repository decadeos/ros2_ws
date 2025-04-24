import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # публикуем каждые 0.5 секунды
        self.timer = self.create_timer(timer_period, self.publish_cmd)

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.05
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

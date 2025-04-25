import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.obstacle_detected = False

    def lidar_callback(self, msg):
        # Берём центральный сектор (например +/- 10 градусов)
        center_idx = len(msg.ranges) // 2
        window = 5  # ширина окна вокруг центра
        center_ranges = msg.ranges[center_idx - window : center_idx + window]

        # Фильтруем NaN/inf и берём минимум
        center_ranges = [r for r in center_ranges if r > 0.0 and r < float('inf')]
        if center_ranges:
            min_distance = min(center_ranges)
            self.obstacle_detected = min_distance < 0.5
        else:
            self.obstacle_detected = False

    def control_loop(self):
        cmd = Twist()
        if self.obstacle_detected:
            # Обнаружили стену → крутимся влево
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().info('СТЕНА')
        else:
            # Путь чист → едем вперёд
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
            self.get_logger().info('ЧИСТО')

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

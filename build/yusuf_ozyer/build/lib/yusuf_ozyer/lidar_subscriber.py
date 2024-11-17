import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # /scan konusuna abone olun
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # Aboneliği saklayın

    def scan_callback(self, msg):
        # LIDAR verisini işleyin (örneğin, minimum mesafeyi alın)
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Minimum distance: {min_distance} meters')

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

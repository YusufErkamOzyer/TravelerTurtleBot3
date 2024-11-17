import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from sensor_msgs.msg import BatteryState
def quaternion_to_euler(x, y, z, w):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw




class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.battery_state_subscriber=self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.current_yaw = 0.0
        self.target_yaw = None
        self.tolerance = 0.01 
        self.lidar_ranges=[]
        self.timer = self.create_timer(0.1, self.go_turtlebot3) 
    def laser_callback(self, msg):
        self.lidar_ranges=msg.ranges
        
    def battery_callback(self, msg):
        if msg.percentage<20:
            self.get_logger().info(f"Battery Percentage {msg.percentage} stopping")
            self.stop_robot()
    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
    
    
    def go_turtlebot3(self):
            if not self.lidar_ranges:
                self.get_logger().warn("LIDAR data is not available yet!")
                return []
            front_data = self.lidar_ranges[len(self.lidar_ranges) // 4: 3 * len(self.lidar_ranges) // 4]
            min_distance = min(front_data)
            self.move_robot(self.calculate_speed(min_distance))
            if min_distance <= 0.4:
                self.stop_robot()
                self.rotate_90_degrees()
                
            else:
                new_speed = self.calculate_speed(min_distance)
                self.move_robot(new_speed)
    def normalize_yaw(self, yaw):
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

    def rotate_90_degrees(self):
        epsilon = 1e-3 
        current_yaw = self.current_yaw
        target_yaw = current_yaw + math.pi / 2 # 90 derece donus

        target_yaw = self.normalize_yaw(target_yaw)

        
        angular_speed = 0.5  

        twist = Twist()
        twist.angular.z = -angular_speed  

        self.get_logger().info(f"Rotating 90 degrees to the right from {current_yaw} to {target_yaw}.")

       
        while True:
            yaw_diff = self.normalize_yaw(self.current_yaw - target_yaw)
            self.get_logger().info(f"Current yaw: {self.current_yaw}, Target yaw: {target_yaw}, Yaw difference: {yaw_diff}")
            if abs(yaw_diff) < epsilon:
                self.get_logger().info("Yaw difference is within epsilon. Stopping rotation.")
                break

            self.cmd_vel_publisher.publish(twist)

        self.stop_robot()
        self.get_logger().info(f"Rotation completed. Current yaw: {self.current_yaw}. Moving forward again.")
        
            



    def move_robot(self,new_speed):
        twist = Twist()
        twist.linear.x = new_speed
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("stopped")
        
    def calculate_speed(self,min_distance):
        max_speed=0.22
        if min_distance>=1.0:
            return max_speed
        else:
            return max_speed*min_distance


def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtleBotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

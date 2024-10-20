import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math
import time
import tf_transformations


class TurtleBotMover(Node):

    def __init__(self):
        super().__init__('turtlebot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odometry, 10)
        self.time_to_move = self.create_timer(0.1, self.move)
        self.current_pose = None
        self.goal()

        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.distance_threshold = 0.1
        self.constant_linear_speed = 0.7
        self.max_angular_speed = 1.0

    def odometry(self,msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation 

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        self.current_pose = (position.x, position.y, yaw)

    
    def goal(self):
        self.x_loc = random.uniform(-3.0,3.0)
        self.y_loc = random.uniform(-3.0,3.0)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.get_logger().info(f'New target: ({self.x_loc:.2f}, {self.y_loc:.2f})')

    
    def angle_change(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi 
        while angle < -math.pi:
            angle += 2.0*math.pi 
        return angle 

    def move(self):
        if self.current_pose is None:
            return 

        curx, cury, curth = self.current_pose 
        dx = self.x_loc - curx 
        dy = self.y_loc - cury 
        distance = math.sqrt(dx **2 + dy **2)
        target_angle = math.atan2(dy,dx)

        angular_err = self.angle_change(target_angle - curth)
        angular_speed = max(min(angular_err * 2.0, self.max_angular_speed), -self.max_angular_speed)

        if distance > self.distance_threshold:
            linear_speed = self.constant_linear_speed
        else:
            linear_speed = 0.0

        twist = Twist()
        twist.linear.x = linear_speed 
        twist.angular.z = angular_speed 
        
        self.publisher.publish(twist)

        if distance < self.distance_threshold: 
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            elapsed_time = current_time - self.start_time 

            if elapsed_time >= 10.0:
                self.get_logger().info("Waiting")
                self.wait(1)

            else:
                self.get_logger().info("Waiting")
                self.wait(10.0 - elapsed_time)

    
    def wait(self, wait_t):
        stop = Twist()
        self.publisher.publish(stop)

        time.sleep(wait_t)
        self.goal()


def main(args= None):
    rclpy.init(args = args)
    node = TurtleBotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
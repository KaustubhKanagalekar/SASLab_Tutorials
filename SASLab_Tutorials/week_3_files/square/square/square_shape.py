# File: square_turtlebot_node.py
'''import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquareTurtleBotNode(Node):

    def __init__(self):
        super().__init__('square_turtlebot_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist_msg = Twist()
        self.state = 'moving_forward'  # Initial state
        self.side_length = 1.0  # Length of one side of the square
        self.current_length = 0.0
        self.current_angle = 0.0

    def timer_callback(self):
        if self.state == 'moving_forward':
            self.move_forward()
        elif self.state == 'rotating':
            self.rotate()

    def move_forward(self):
        speed = 0.2
        self.twist_msg.linear.x = speed
        self.publisher.publish(self.twist_msg)

        self.current_length += speed * 0.1  # Assuming 0.1 seconds elapsed per timer callback
        self.get_logger().info(f'Moving forward: {self.current_length:.2f} / {self.side_length:.2f}')

        if self.current_length >= self.side_length:
            self.twist_msg.linear.x = 0.0
            self.publisher.publish(self.twist_msg)
            self.current_length = 0.0
            self.state = 'rotating'

    def rotate(self):
        speed = 0.5
        self.twist_msg.angular.z = speed
        self.publisher.publish(self.twist_msg)

        self.current_angle += speed * 0.1  # Assuming 0.1 seconds elapsed per timer callback

        if self.current_angle >= 1.57:  # 90 degrees in radians
            self.twist_msg.angular.z = 0.0
            self.publisher.publish(self.twist_msg)
            self.current_angle = 0.0
            self.state = 'moving_forward'

def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtleBotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareTurtleBotNode(Node):

    def __init__(self):
        super().__init__('square_turtlebot_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist_msg = Twist()
        self.state = 'moving_forward'  # Initial state
        self.side_length = 1.0  # Length of one side of the square
        self.current_length = 0.0
        self.current_angle = 0.0
        self.step_count = 0  # Number of completed sides
        self.total_steps = 4  # Total sides to move

    def timer_callback(self):
        if self.step_count >= self.total_steps:
            self.stop_robot()  # Stop if total steps are reached
            return

        if self.state == 'moving_forward':
            self.move_forward()
        elif self.state == 'rotating':
            self.rotate()

    def move_forward(self):
        speed = 0.2
        self.twist_msg.linear.x = speed
        self.publisher.publish(self.twist_msg)

        self.current_length += speed * 0.1  # Assuming 0.1 seconds elapsed per timer callback
        self.get_logger().info(f'Step {self.step_count + 1}: Moving forward: {self.current_length:.2f} / {self.side_length:.2f}')

        if self.current_length >= self.side_length:
            self.twist_msg.linear.x = 0.0  # Stop forward movement
            self.publisher.publish(self.twist_msg)
            self.current_length = 0.0
            self.state = 'rotating'
            self.get_logger().info(f'Step {self.step_count + 1}: Reached side length, switching to rotating.')

    def rotate(self):
        speed = 0.5
        self.twist_msg.angular.z = speed
        self.publisher.publish(self.twist_msg)

        self.current_angle += speed * 0.1  # Assuming 0.1 seconds elapsed per timer callback

        if self.current_angle >= 1.57:  # 90 degrees in radians
            self.twist_msg.angular.z = 0.0  # Stop rotation
            self.publisher.publish(self.twist_msg)
            self.current_angle = 0.0
            self.state = 'moving_forward'
            self.step_count += 1  # Increment step count
            self.get_logger().info(f'Step {self.step_count}: Completed rotation, switching to moving forward.')

    def stop_robot(self):
        # Stop the robot by sending zero velocities
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)
        self.get_logger().info("Robot stopped after completing the square.")

def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtleBotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()  # Stop the robot on Ctrl+C
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 
import tf_transformations 
import math 

class SquareTurtleBotNode(Node):

    def __init__(self):
        super().__init__('square_turtlebot_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.twist_msg = Twist()
        
        self.state = 'moving_forward'  # Initial state
        self.side_length = 1.0  # Length of one side of the square
        self.current_length = 0.0
        self.current_angle = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.step_count = 0  # Number of completed sides
        self.total_steps = 4  # Total sides to move
        self.start_position = None

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation 
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # Update current pose values
        self.current_x = position.x
        self.current_y = position.y
        self.current_yaw = yaw
        self.get_logger().info(f"Updated current position: x={self.current_x}, y={self.current_y}, yaw={self.current_yaw}")

    def timer_callback(self):
        if self.step_count >= self.total_steps:
            self.stop_robot()  # Stop if total steps are reached
            return

        if self.state == 'moving_forward':
            self.move_forward()
        elif self.state == 'rotating':
            self.rotate()

    def move_forward(self):
        speed = 0.2

        # Initialize start_position only once
        if self.start_position is None:
            self.start_position = self.current_x  # Set the starting position

        self.twist_msg.linear.x = speed
        self.publisher.publish(self.twist_msg)

        # Update current_length based on the current pose
        self.current_length = self.current_x - self.start_position
        self.get_logger().info(f'Step {self.step_count + 1}: Moving forward: {self.current_length:.2f} / {self.side_length:.2f}')

        # Check if the length of the side has been reached
        if abs(self.current_length) >= self.side_length:
            self.twist_msg.linear.x = 0.0  # Stop forward movement
            self.publisher.publish(self.twist_msg)
            self.start_position = None  # Reset start_position for the next side
            self.current_length = 0.0  # Reset current_length for the next side
            self.state = 'rotating'
            self.get_logger().info(f'Step {self.step_count + 1}: Reached side length, switching to rotating.')

    def rotate(self):
        speed = 0.5
        self.twist_msg.angular.z = speed
        self.publisher.publish(self.twist_msg)

        # Check for completion of rotation
        if abs(self.current_yaw) >= (math.pi / 2):  # 90 degrees in radians
            self.twist_msg.angular.z = 0.0  # Stop rotation
            self.publisher.publish(self.twist_msg)
            
            # Resetting yaw to 0 only if the robot completed the rotation
            self.current_yaw = 0.0  
            self.start_position = None  # Reset the start position for the next side
            self.state = 'moving_forward'
            self.step_count += 1  # Increment step count
            self.get_logger().info(f'Step {self.step_count}: Completed rotation, switching to moving forward.')


    def stop_robot(self):
        # Stop the robot by sending zero velocities
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)
        self.get_logger().info("Robot stopped after completing the square.")

def main(args=None):
    rclpy.init(args=args)
    node = SquareTurtleBotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()  # Stop the robot on Ctrl+C
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


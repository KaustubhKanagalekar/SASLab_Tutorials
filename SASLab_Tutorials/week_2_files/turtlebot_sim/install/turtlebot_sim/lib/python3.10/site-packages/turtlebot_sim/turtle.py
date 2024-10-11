# File: teleport_turtlebot_node.py
import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist

class TeleportTurtleBotNode(Node):

    def __init__(self):
        super().__init__('teleport_turtlebot_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 10.0  # Time interval to teleport the TurtleBot
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Triggering TurtleBot teleport...')
        self.teleport_turtlebot()

    def teleport_turtlebot(self):
        # Create a Twist message
        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(-1.0, 1.0)  # Random linear velocity
        twist_msg.angular.z = random.uniform(-1.0, 1.0)  # Random angular velocity

        self.publisher.publish(twist_msg)
        self.get_logger().info('Published velocity: linear.x: %.2f, angular.z: %.2f' % (twist_msg.linear.x, twist_msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = TeleportTurtleBotNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# File: teleport_turtle_node.py
import rclpy
from rclpy.node import Node
import random 
from turtlesim.srv import TeleportAbsolute


class TeleportTurtleNode(Node):

    def __init__(self):
        super().__init__('teleport_turtle_node')
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.teleport_turtle()

    def teleport_turtle(self):
        request = TeleportAbsolute.Request()
        request.x = random.uniform(0.0,10.0)   # Specify the x coordinate
        request.y = random.uniform(0.0,10.0)  # Specify the y coordinate
        request.theta = random.uniform(0,6.28)  # Specify the angle in radians

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle teleported to x: %.1f, y: %.1f, theta: %.2f' % (request.x, request.y, request.theta))
        else:
            self.get_logger().error('Failed to teleport turtle')

class TimerNode(Node):

    def __init__(self):
        super().__init__('timer_node')
        self.timer_period = 10.0  # Timer period in seconds (e.g., 2 seconds)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.teleport_turtle_node = TeleportTurtleNode()

    def timer_callback(self):
        self.get_logger().info('Triggering turtle teleport...')
        self.teleport_turtle_node.teleport_turtle()


def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ArchimedesSpiral(Node):

    def __init__(self):
        super().__init__('archimedes_spiral')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        
        # Parameters for the spiral
        self.initial_linear_velocity = 7.0  # m/s (constant linear velocity)
        self.initial_angular_velocity = 10  # rad/s (initial angular velocity)
        self.angular_velocity_decrease = 0.4  # Rate at which angular velocity decreases
        self.min_angular_velocity = 1.75  # Minimum angular velocity to avoid too slow turns
        self.time_elapsed = 0.0  # Time elapsed since start

        # Create a timer to publish commands periodically
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Update time elapsed
        self.time_elapsed += 0.1  # Timer period is 0.1s

        # Create a Twist message
        twist = Twist()

        # Set linear velocity to a constant value
        twist.linear.x = self.initial_linear_velocity

        # Calculate angular velocity (decreases smoothly over time)
        angular_velocity = self.initial_angular_velocity - self.angular_velocity_decrease * self.time_elapsed
        twist.angular.z = max(angular_velocity, self.min_angular_velocity)

        # Log the velocities for debugging
        self.get_logger().info(f"Linear velocity: {twist.linear.x}, Angular velocity: {twist.angular.z}")

        # Publish the Twist message
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    spiral_turtle = ArchimedesSpiral()
    try:
        rclpy.spin(spiral_turtle)
    except KeyboardInterrupt:
        pass
    finally:
        spiral_turtle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

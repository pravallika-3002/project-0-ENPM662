import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('node_2')

        # Publisher to publish velocity commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Parameters for the motion
        self.acceleration = 0.2  # Acceleration (m/s^2)
        self.target_velocity = 0.5  # Maximum velocity (m/s)
        self.deceleration = -0.2  # Deceleration (m/s^2)
        self.distance_constant_velocity = 2.0  # Distance to travel at constant velocity (m)

        self.current_velocity = 0.0  # Current velocity (m/s)
        self.phase = "acceleration"  # Current phase of motion
        self.start_time = time()  # Start time of the motion
        self.last_time = self.start_time  # Time since the last update

        self.acceleration_time = self.target_velocity / self.acceleration  # Time to reach target velocity
        self.deceleration_time = -self.target_velocity / self.deceleration  # Time to stop from target velocity

        self.constant_velocity_start_time = None  # When the constant velocity phase starts
        self.constant_velocity_end_time = None  # When the deceleration phase should start
        self.distance_traveled_constant_velocity = 0.0  # Distance traveled at constant velocity

        # Create a timer to update the velocity commands at 10Hz
        self.timer = self.create_timer(0.1, self.update_velocity)

    def update_velocity(self):
        current_time = time()
        time_elapsed = current_time - self.start_time
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # Create the Twist message for velocity
        velocity_msg = Twist()

        if self.phase == "acceleration":
            # Accelerate to the target velocity
            self.current_velocity = self.acceleration * time_elapsed

            if self.current_velocity >= self.target_velocity:
                # Switch to constant velocity phase
                self.current_velocity = self.target_velocity
                self.phase = "constant_velocity"
                self.constant_velocity_start_time = time_elapsed
                self.constant_velocity_end_time = self.constant_velocity_start_time + (self.distance_constant_velocity / self.target_velocity)

            velocity_msg.linear.x = self.current_velocity

        elif self.phase == "constant_velocity":
            # Move at constant velocity
            velocity_msg.linear.x = self.target_velocity
            self.distance_traveled_constant_velocity += self.target_velocity * delta_time

            if self.distance_traveled_constant_velocity >= self.distance_constant_velocity:
                # Switch to deceleration phase
                self.phase = "deceleration"
                self.start_time = current_time  # Reset the timer for deceleration

        elif self.phase == "deceleration":
            # Decelerate to stop
            time_in_deceleration = time_elapsed
            self.current_velocity = self.target_velocity + self.deceleration * time_in_deceleration

            if self.current_velocity <= 0.0:
                self.current_velocity = 0.0
                self.get_logger().info("TurtleBot3 has stopped.")
                self.timer.cancel()  # Stop the timer when the robot has come to rest

            velocity_msg.linear.x = self.current_velocity

        # Publish the velocity command
        self.cmd_vel_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




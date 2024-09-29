import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')

#parameters assumed for the motion
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
#target distance is given in in meters and velocity in meters per second
        self.x_target = 2.0  #bot is moved 2 m
        self.velocity_value = 0.2  #constant velocity 0.2 m/s
#the amount of time required to reach the target is caluculated
        self.time_to_reach_target = self.x_target / self.velocity_value

        self.get_logger().info(f'Moving towards the target distance: {self.x_target} meters at {self.velocity_value} m/s')
        self.get_logger().info(f'Estimated time to reach the target: {self.time_to_reach_target} seconds')

        self.move_turtlebot()

    def move_turtlebot(self):
# Creating a velocity message 
        velocity_msg = Twist()
        velocity_msg.linear.x = self.velocity_value
        velocity_msg.angular.z = 0.0  # No rotation
#start time is obtained
        start_time = time.time()
#turtlebot is moved for a calculated time
        while time.time() - start_time < self.time_to_reach_target:
            self.publisher_.publish(velocity_msg)
            self.get_logger().info(f'Moving... Time: {time.time() - start_time:.2f} seconds')
            time.sleep(0.1)  #delayed to avoid flooding of topic
#turtlebot stops after reaching the target
        velocity_msg.linear.x = 0.0
        self.publisher_.publish(velocity_msg)
        self.get_logger().info('Reached the target distance. Stopping.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



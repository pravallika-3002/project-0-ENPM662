import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import time

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('node_2')

#creating a publisher to publish velocity to the bot 
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

#parameters assumed for the given scenario
        self.acceleration = 0.2#acceleration in m/s^2
        self.target_velocity = 0.5#maximum velocity in m/s
        self.deceleration = -0.2#deceleration in m/s^2
        self.distance_constant_velocity = 2.0#distance to travel at constant velocity in m

        self.current_velocity = 0.0#current velocity in m/s
        self.phase = "acceleration"#current phase of motion
        self.start_time = time()#time at which the motion starts
        self.last_time = self.start_time#time since the last update is made

        self.acceleration_time = self.target_velocity / self.acceleration#time to reach the target velocity
        self.deceleration_time = -self.target_velocity / self.deceleration#time to stop from the target velocity

        self.constant_velocity_start_time = None#start of constant velocity phase
        self.constant_velocity_end_time = None#start of deceleration phase 
        self.distance_traveled_constant_velocity = 0.0#amount of distance traveled  by the bot at a constant velocity

 #creating a timer to update the velocity commands at the rate of 10Hz
        self.timer = self.create_timer(0.1, self.update_velocity)

    def update_velocity(self):
        current_time = time()
        time_elapsed = current_time - self.start_time
        delta_time = current_time - self.last_time
        self.last_time = current_time

 #twist message for velocity
        velocity_msg = Twist()

        if self.phase == "acceleration":
            #bot accelerates to target velocity
            self.current_velocity = self.acceleration * time_elapsed

            if self.current_velocity >= self.target_velocity:
                #switching to constant velocity phase
                self.current_velocity = self.target_velocity
                self.phase = "constant_velocity"
                self.constant_velocity_start_time = time_elapsed
                self.constant_velocity_end_time = self.constant_velocity_start_time + (self.distance_constant_velocity / self.target_velocity)

            velocity_msg.linear.x = self.current_velocity

        elif self.phase == "constant_velocity":
            #bot moves with constant velocity
            velocity_msg.linear.x = self.target_velocity
            self.distance_traveled_constant_velocity += self.target_velocity * delta_time

            if self.distance_traveled_constant_velocity >= self.distance_constant_velocity:
                #deceleration phase of bot
                self.phase = "deceleration"
                self.start_time = current_time#the timer should be resetted for deceleration

        elif self.phase == "deceleration":
 #decelerates and stops
            time_in_deceleration = time_elapsed
            self.current_velocity = self.target_velocity + self.deceleration * time_in_deceleration

            if self.current_velocity <= 0.0:
                self.current_velocity = 0.0
                self.get_logger().info("TurtleBot3 has stopped.")
                self.timer.cancel()#timer stops when the robot comes to rest

            velocity_msg.linear.x = self.current_velocity

 #publishing the velocity command
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




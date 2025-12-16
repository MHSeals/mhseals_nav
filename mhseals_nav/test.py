import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class BoatController(Node):
    def __init__(self):
        super().__init__("boat_control")

        self.get_logger().info("Boat Controller Node Initialized")

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, "/mavros/setpoint_velocity/cmd_vel", 10
        )

        self.cmd_vel = TwistStamped()
        self.cmd_vel.twist.linear.x = 0.0
        self.cmd_vel.twist.linear.y = 0.0
        self.cmd_vel.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.cmd_vel)
        
        self.timer = self.create_timer(0.1, self.run)

    def set_x_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.x = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def set_y_velocity(self, velocity: float):
        self.cmd_vel.twist.linear.y = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def set_angular_velocity(self, velocity: float):
        self.cmd_vel.twist.angular.z = velocity
        self.cmd_vel_publisher.publish(self.cmd_vel)
        
    def run(self):
        self.set_x_velocity(-5.0)

def main(args=None):
    rclpy.init(args=args)

    boat_controller = BoatController()

    try:
        rclpy.spin(boat_controller)
    except KeyboardInterrupt:
        boat_controller.get_logger().info("Shutting down")
    finally:
        boat_controller.destroy_node()
        rclpy.shutdown()
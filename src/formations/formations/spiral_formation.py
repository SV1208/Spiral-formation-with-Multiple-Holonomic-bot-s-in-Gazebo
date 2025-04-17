import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        # Use a private attribute with an underscore
        self._publishers = {
            # 'bot1': self.create_publisher(Twist, '/bot1/cmd_vel', 10),
            'bot2': self.create_publisher(Twist, '/bot2/cmd_vel', 10),
            'bot3': self.create_publisher(Twist, '/bot3/cmd_vel', 10),
            'bot4': self.create_publisher(Twist, '/bot4/cmd_vel', 10),
            'bot5': self.create_publisher(Twist, '/bot5/cmd_vel', 10)
        }
        self.timer = self.create_timer(0.1, self.publish_velocities)  # Publish at 10 Hz

        self.bot_twists = {
            # 'bot1': {'linear': {'x': -1.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 1.0}},
            'bot2': {'linear': {'x': 0.0, 'y': 4.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 2.0}},
            'bot3': {'linear': {'x': 0.0, 'y': -4.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 2.0}},
            'bot4': {'linear': {'x': -4.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 2.0}},
            'bot5': {'linear': {'x': 4.0, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': 2.0}}
        }

    def publish_velocities(self):
        for bot_name, twist_data in self.bot_twists.items():
            twist_msg = Twist()
            twist_msg.linear.x = twist_data['linear']['x']
            twist_msg.linear.y = twist_data['linear']['y']
            twist_msg.linear.z = twist_data['linear']['z']
            twist_msg.angular.x = twist_data['angular']['x']
            twist_msg.angular.y = twist_data['angular']['y']
            twist_msg.angular.z = twist_data['angular']['z']
            self._publishers[bot_name].publish(twist_msg)
            self.get_logger().info(f'Published Twist for {bot_name}')

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

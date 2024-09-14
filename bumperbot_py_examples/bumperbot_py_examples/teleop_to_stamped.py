import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TeleopToStamped(Node):
    def __init__(self):

        super().__init__('teleop_to_stamped')
        self.subscriber = self.create_subscription(Twist, '/key_vel', self.listener_callback, 10)
        self.publisher = self.create_publisher(TwistStamped, '/bumperbot_controller/cmd_vel', 10)


    def listener_callback(self, msg):

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"  # Set your frame_id accordingly
        twist_stamped.twist = msg
        self.publisher.publish(twist_stamped)


def main(args=None):

    rclpy.init(args=args)
    teleop_to_stamped = TeleopToStamped()
    rclpy.spin(teleop_to_stamped)
    teleop_to_stamped.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

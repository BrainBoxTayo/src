import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class SimpleTFKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")

        # to publish a fixed transform we can use the static transform broadcaster
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)

        self.static_transform_stamped = TransformStamped()
        self.static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped.header.frame_id = "bumperbot_base"
        self.static_transform_stamped.child_frame_id = "bumperbot_top"
        self.static_transform_stamped.transform.translation.x = 0.0
        self.static_transform_stamped.transform.translation.y = 0.0
        self.static_transform_stamped.transform.translation.z = 0.3
        self.static_transform_stamped.transform.rotation.x = 0.0
        self.static_transform_stamped.transform.rotation.y = 0.0
        self.static_transform_stamped.transform.rotation.z = 0.0
        self.static_transform_stamped.transform.rotation.w = 1.0

        self.dynamic_transform_stamped = TransformStamped()
        self.x_increment = 0.05 # to simulate movement
        self.last_x = 0.0

        self.static_tf_broadcaster.sendTransform(self.static_transform_stamped)

        self.get_logger().info("publishing static transform between {} and {}".format(self.static_transform_stamped.header.frame_id, self.static_transform_stamped.child_frame_id))
        self.timer = self.create_timer(0.1, self.timerCallback)

    def timerCallback(self):
        
        self.dynamic_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped.transform.translation.x = self.last_x + self.x_increment
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.x = 0.0
        self.dynamic_transform_stamped.transform.rotation.y = 0.0
        self.dynamic_transform_stamped.transform.rotation.z = 0.0
        self.dynamic_transform_stamped.transform.rotation.w = 1.0

        self.dynamic_tf_broadcaster.sendTransform(self.dynamic_transform_stamped)
        self.last_x = self.dynamic_transform_stamped.transform.translation.x


def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTFKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
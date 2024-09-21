import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform


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
        self.x_increment = 0.05  # to simulate movement
        self.last_x = 0.0
        self.rotations_counter = 0
        self.last_orientation = quaternion_from_euler(0, 0, 0)
        self.orientation_increment = quaternion_from_euler(0, 0, 0.05)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.static_tf_broadcaster.sendTransform(self.static_transform_stamped)

        self.get_logger().info("publishing static transform between {} and {}".format(
            self.static_transform_stamped.header.frame_id, self.static_transform_stamped.child_frame_id))
        self.timer = self.create_timer(0.1, self.timerCallback)

        self.get_transform_srv = self.create_service(
            GetTransform, "get_transform", self.getTransformCallback)

    def getTransformCallback(self, req, res):
        self.get_logger().info("Requested Transform Between {} and {}".format(
            req.frame_id, req.child_frame_id))
        requested_transform = TransformStamped()
        try:
            requested_transform = self.tf_buffer.lookup_transform(
                req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An Error occured while transforming {} and {}".format(
                req.frame_id, req.child_frame_id))
            res.success = False
            return res
        res.transform = requested_transform
        res.success = True
        return res

    def timerCallback(self):

        self.dynamic_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped.header.frame_id = "odom"
        self.dynamic_transform_stamped.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped.transform.translation.x = self.last_x + self.x_increment
        self.dynamic_transform_stamped.transform.translation.y = 0.0
        self.dynamic_transform_stamped.transform.translation.z = 0.0
        q = quaternion_multiply(self.last_orientation,
                                self.orientation_increment)
        self.dynamic_transform_stamped.transform.rotation.x = q[0]
        self.dynamic_transform_stamped.transform.rotation.y = q[1]
        self.dynamic_transform_stamped.transform.rotation.z = q[2]
        self.dynamic_transform_stamped.transform.rotation.w = q[3]

        self.dynamic_tf_broadcaster.sendTransform(
            self.dynamic_transform_stamped)
        self.last_x = self.dynamic_transform_stamped.transform.translation.x
        self.rotations_counter += 1
        self.last_orientation = q
        self.get_logger().info("rotations: {}".format(self.rotations_counter))

        if self.rotations_counter >= 100:
            self.orientation_increment = quaternion_inverse(self.orientation_increment)
            self.rotations_counter = 0


def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTFKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

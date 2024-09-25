import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovariance, PoseStamped


class bumperbotPath(Node):
    def __init__(self):
        super().__init__("bumperbot_path")
        self.odom_sub = self.create_subscription(
            Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        self.trajectory_pub = self.create_publisher(
            Path, "bumperbot_controller/trajectory", 10
        )
        self.path_arr = Path()

        # I need to set the pose in the path to a poseStamped
        # form of the PoseWithCovariance of the Odom topic
        # basically poseStamped.pose = PoseWithCovariance.Pose

    def odomCallback(self, msg):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = msg.pose.pose

        self.path_arr.poses.append(pose)
        self.trajectory_pub.publish(self.path_arr)
        self.get_logger().info("Publishing to trajectory")


def main():
    rclpy.init()
    bumperbot_path = bumperbotPath()
    rclpy.spin(bumperbot_path)
    bumperbot_path.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
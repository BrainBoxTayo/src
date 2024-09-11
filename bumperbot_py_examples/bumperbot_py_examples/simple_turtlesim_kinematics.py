import rclpy
from rclpy.node import Node
from turtlesim.msg._pose import Pose
from math import sin, cos

class SimpleTurtleSimKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")

        self.turtle1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback, 10)
        self.turtle2_pose_sub = self.create_subscription(Pose, "/turtle2/pose", self.turtle2PoseCallback, 10)

        self.last_turtle1_pose = Pose()
        self.last_turtle2_pose = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose = msg

    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose = msg
        Tx = self.last_turtle2_pose.x - self.last_turtle1_pose.x
        Ty = self.last_turtle2_pose.y - self.last_turtle1_pose.y
        theta_rad = self.last_turtle2_pose.theta - self.last_turtle1_pose.theta
        theta_deg = 180 * theta_rad / 3.14
        self.get_logger().info("\n \
                               Translation Vector turtle1 -> turtle2 \n \
                               Tx: {} \
                               Ty: {}\n \
                               Rotation Matrix turtle1 -> turtle2 \
                               theta(rad: {}) \
                               theta(deg: {})\n \
                               [R11         R12] : [{}          {}]\n \
                               [R21         R22] : [{}          {}]\n".format(Tx,Ty, theta_rad, theta_deg, cos(theta_rad), -sin(theta_rad), sin(theta_rad), cos(theta_rad)))
        

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtleSimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from math import sin, cos

class simpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius = self.get_parameter(
            "wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            "wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius {}".format(self.wheel_radius))
        self.get_logger().info("Using Wheel Separation: {}".format(self.wheel_separation))
        
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = self.get_clock().now()
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0

        self.wheel_cmd_pub = self.create_publisher(
            Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub = self.create_subscription(
            TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10)

        self.speed_conversion_matrix = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                                                [self.wheel_radius / self.wheel_separation, -(self.wheel_radius / self.wheel_separation)]])

        self.get_logger().info("The conversion matrix is {}".format(self.speed_conversion_matrix))

    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(
            self.speed_conversion_matrix), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0],
                                wheel_speed[0, 0]]

        self.wheel_cmd_pub.publish(wheel_speed_msg)
    
    def jointCallback(self, msg):
        # for now in gazebo, the messages are gotten from the jointStates topic.
        # IRL, we would get the position data from an encoder
        dp_left = msg.position[1] - self.left_wheel_prev_pos # change in positions of wheel
        dp_right = msg.position[0] - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time # change in time to get to curr position
        
        #update all holders
        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        phi_left = dp_left / ( dt.nanoseconds / S_TO_NS )
        phi_right = dp_right / ( dt.nanoseconds / S_TO_NS ) # Velocities of the wheels
        
        # self.get_logger().info("msg.position[1]: {}  msg.position[0]: {}".format(msg.position[1],msg.position[0]))

        linear_vel = (self.wheel_radius * phi_right + self.wheel_radius * phi_left) / 2 # From Kinematics
        angular_vel = (self.wheel_radius * phi_right - self.wheel_radius * phi_left) / self.wheel_separation # From Kinematics
        ds = (self.wheel_radius * phi_right + self.wheel_radius * phi_left) / 2
        d_theta = (self.wheel_radius * phi_right - self.wheel_radius * phi_left) / self.wheel_separation
        self.pos_theta += d_theta
        self.pos_x += ds * cos(self.pos_theta)
        self.pos_y += ds * sin(self.pos_theta)

        self.get_logger().info("Linear_Velocity: {}, Angular_velocity: {}".format(linear_vel, angular_vel))
        self.get_logger().info("X Positon: {}, Y Position: {}, Theta: {}".format(self.pos_x, self.pos_y, self.pos_theta))


def main():
    rclpy.init()
    simple_controller = simpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rclpy.constants import S_TO_NS
from math import sin, cos
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")

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
        
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_theta = 0.0
        

       
        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, "bumperbot_controller/odom_noisy", 10)
        

      

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.x = 0.0 
        self.odom_msg.pose.pose.orientation.y = 0.0 
        self.odom_msg.pose.pose.orientation.z = 0.0 
        self.odom_msg.pose.pose.orientation.w = 1.0 
        
        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint_noisy"
        self.prev_time = self.get_clock().now()



  
    def jointCallback(self, msg):
        # Adding Noise
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos # change in positions of wheel
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time # change in time to get to curr position
        
        #update all previous variables
        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        phi_left = dp_left / ( dt.nanoseconds / S_TO_NS )
        phi_right = dp_right / ( dt.nanoseconds / S_TO_NS ) # Velocities of the wheels
        
        linear_vel = (self.wheel_radius * phi_right + self.wheel_radius * phi_left) / 2 # From Kinematics
        angular_vel = (self.wheel_radius * phi_right - self.wheel_radius * phi_left) / self.wheel_separation # From Kinematics
        ds = (self.wheel_radius * dp_right + self.wheel_radius * dp_left) / 2
        d_theta = (self.wheel_radius * dp_right - self.wheel_radius * dp_left) / self.wheel_separation
        self.pos_theta += d_theta
        self.pos_x += ds * cos(self.pos_theta)
        self.pos_y += ds * sin(self.pos_theta)

        q = quaternion_from_euler(0, 0, self.pos_theta)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.pos_x
        self.odom_msg.pose.pose.position.y = self.pos_y
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.angular.z = angular_vel

        self.transform_stamped.transform.translation.x = self.pos_x
        self.transform_stamped.transform.translation.y = self.pos_y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub.publish(self.odom_msg)
        self.br.sendTransform(self.transform_stamped)

def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

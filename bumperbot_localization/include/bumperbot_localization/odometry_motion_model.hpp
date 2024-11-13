#ifndef ODOM_MOTION_HPP
#define ODOM_MOTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class OdometryMotionModel : public rclcpp::Node 
{
    public:
        OdometryMotionModel(const std::string &name);
    private:
    
        void odomCallback(const nav_msgs::msg::Odometry &odom);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;    
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub;

        geometry_msgs::msg::PoseArray samples;
        double alpha1;
        double alpha2;
        double alpha3;
        double alpha4;
        int nr_samples;

        double last_odom_x;
        double last_odom_y;
        double last_odom_theta;
        bool is_first_odom;
};

#endif
#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node 
{
    public:
        KalmanFilter(const std::string &name);
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

        double __mean;
        double __variance;
        double __imu_angular_z;
        bool __is_first_odom;
        double __last_angular_z;
        double __motion;

        nav_msgs::msg::Odometry __kalman_odom;

        double __motion_variance;
        double __measurement_variance;

        void measurementUpdate();
        void statePrediction();
        void odomCallback(const nav_msgs::msg::Odometry &odom);
        void imuCallback(const sensor_msgs::msg::Imu &imu);
    

};

#endif
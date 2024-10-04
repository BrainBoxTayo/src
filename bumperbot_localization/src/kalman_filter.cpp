#include <kalman_filter.hpp>

KalmanFilter::KalmanFilter(const std::string &name)
    : Node(name),
      __mean(0.0),
      __variance(1000.0),
      __imu_angular_z(0.0),
      __is_first_odom(true),
      __last_angular_z(0.0),
      __motion(0.0),
      __motion_variance(4.0),
      __measurement_variance(0.5)
{
    odom_sub = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom_noisy", 10, std::bind(&KalmanFilter::odomCallback, this, std::placeholders::_1));
    imu_sub = create_subscription<sensor_msgs::msg::Imu>("imu/out", 10, std::bind(&KalmanFilter::imuCallback, this, std::placeholders::_1));
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("bumperbot_controller/odom_kalman", 10);
}

void KalmanFilter::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    __kalman_odom = odom;

    if (__is_first_odom)
    {
        // First Step of the kalman filter algorithm
        // Initial Estimate
        __mean = odom.twist.twist.angular.z;
        __last_angular_z = odom.twist.twist.angular.z;
        __is_first_odom = false;
        return;
    }

    __motion = odom.twist.twist.angular.z - __last_angular_z;
    statePrediction();
    measurementUpdate();

    __last_angular_z = odom.twist.twist.angular.z;
    __kalman_odom.twist.twist.angular.z = __mean;
    odom_pub->publish(__kalman_odom);
}

void KalmanFilter::imuCallback(const sensor_msgs::msg::Imu &imu)
{
    __imu_angular_z = imu.angular_velocity.z;
}

void KalmanFilter::measurementUpdate()
{
    // measurement variance is the variance of the measurements coming from the IMU sensor,
    __mean = (__measurement_variance * __mean + __variance * __imu_angular_z) / (__measurement_variance + __variance);
    __variance = (__variance * __measurement_variance) / (__variance + __measurement_variance);
}

void KalmanFilter::statePrediction()
{
    __mean = __mean + __motion;
    __variance = __variance + __motion_variance;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KalmanFilter>("kalman_filter");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
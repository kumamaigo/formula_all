#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

using namespace std::chrono_literals;

class FusionNode : public rclcpp::Node
{
public:
    FusionNode()
    : Node("fusion_node"),alpha_(0.98), yaw_angle_(0.0), yaw_rate_(0.0)
    {
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("/sub_odom", 50, std::bind(&FusionNode::odomCallback, this, std::placeholders::_1));

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("/aiformula_sensing/zed_node/imu", 200, std::bind(&FusionNode::imuCallback, this, std::placeholders::_1));

        pub_fusion_ = create_publisher<nav_msgs::msg::Odometry>("/odom_fusion", 10);

        timer_ = create_wall_timer(10ms, std::bind(&FusionNode::timerCallback, this));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_odom_ = *msg;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        yaw_rate_ = msg->angular_velocity.z;
    }

    void timerCallback()
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if (latest_odom_.header.stamp.sec == 0 && latest_odom_.header.stamp.nanosec == 0)
            return;

        rclcpp::Time now = this->get_clock()->now();
        if (last_time_.nanoseconds() == 0) 
        {
            last_time_ = now;
            return;
        }

        double dt = (now - last_time_).seconds();
        last_time_ = now;

        double yaw_imu = yaw_angle_ + yaw_rate_ * dt;

        double yaw_odom = tf2::getYaw(latest_odom_.pose.pose.orientation);

        yaw_angle_ = alpha_ * yaw_imu + (1.0 - alpha_) * yaw_odom;

        nav_msgs::msg::Odometry fused = latest_odom_;
        fused.header.stamp = now;
        fused.pose.pose.position = latest_odom_.pose.pose.position;
        fused.pose.pose.orientation = toQuaternionMsg(yaw_angle_);
        fused.twist.twist.angular.z = yaw_rate_;

        pub_fusion_->publish(fused);
    }

    geometry_msgs::msg::Quaternion toQuaternionMsg(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_fusion_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_;
    nav_msgs::msg::Odometry latest_odom_;
    rclcpp::Time last_time_;

    double alpha_;
    double yaw_angle_;
    double yaw_rate_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FusionNode>());
    rclcpp::shutdown();
    return 0;
}
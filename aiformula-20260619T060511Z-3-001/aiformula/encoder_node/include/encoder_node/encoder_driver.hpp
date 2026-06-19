#ifndef ENCODER_DRIVER_HPP_
#define ENCODER_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <serial/serial.h>
#include <memory>
#include <string>
#include <vector>
#include <array>

#include "encoder_node/udp_sender.hpp"

constexpr int SNAPSHOT_SIZE = 5; //[count_total, rad, omega, rps, rpm]

class EncoderDriver : public rclcpp::Node
{
public:
    EncoderDriver();
    void spin();

private:
    void parse_line(const std::string &line);
    void publish_odom();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::array<float, SNAPSHOT_SIZE> snapshot1_;
    std::array<float, SNAPSHOT_SIZE> snapshot2_;
    std::unique_ptr<UdpSender> udp_sender_;

    double x_, y_, yaw_;
    double prev_rad1_, prev_rad2_;
    double l_, radius_;
};

#endif
#include "encoder_node/encoder_driver.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <cmath>

#define DEFAULT_PORT "/dev/ttyACM0"
#define BAUD 115200
#define UDP_IP "192.168.10.3"
#define UDP_PORT 5005

EncoderDriver::EncoderDriver() : Node("encoder_driver"), x_(0.0), y_(0.0), yaw_(0.0)
{
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sub_odom", 10);
    snapshot1_.fill(0.0f);
    snapshot2_.fill(0.0f);
    udp_sender_ = std::make_unique<UdpSender>(UDP_IP, UDP_PORT);

    l_ = 0.815; //車体中心から従属輪までの距離
    radius_ = 0.0508; //従属輪の半径[m]
    
    //前回の角度を保持する変数
    prev_rad1_ = 0.0;
    prev_rad2_ = 0.0;
}

void EncoderDriver::parse_line(const std::string &line)
{
    if(line.empty()) return;

    std::string clean_line = line;
    if(clean_line[0] == '>')
    {
        clean_line.erase(0, 1);
    }

    auto pos = clean_line.find(":");
    if(pos == std::string::npos) return;

    std::string key = clean_line.substr(0, pos);
    std::string value_str = clean_line.substr(pos + 1);

    while(!key.empty() && (key.front() == '\t' || key.front() == ' '))
    {
        key.erase(0, 1);
    }
    while(!key.empty() && (key.back() == '\t' || key.back() == ' '))
    {
        key.pop_back();
    }
    while(!value_str.empty() && (value_str.front() == '\t' || value_str.front() == ' '))
    {
        value_str.erase(0, 1);
    }
    while(!value_str.empty() && (value_str.back() == '\t' || value_str.back() == ' '))
    {
        value_str.pop_back();
    }

    double value = 0.0;
    try
    {
        value = std::stod(value_str);
    }
    catch(const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Invalid value for key %s: '%s'", key.c_str(), value_str.c_str());
        return;
    }

    //E1
    if(key.find("E1_count_total") != std::string::npos)
    {
        snapshot1_[0] = value;
    }
    else if(key.find("E1_rad") != std::string::npos)
    {
        snapshot1_[1] = value;
    }
    else if(key.find("E1_ω") != std::string::npos || key.find("E1_w") != std::string::npos)
    {
        snapshot1_[2] = value;
    }
    else if(key.find("E1_RPS") != std::string::npos)
    {
        snapshot1_[3] = value;
    }
    else if(key.find("E1_RPM") != std::string::npos)
    {
        snapshot1_[4] = value;
    }

    //E2
    else if(key.find("E2_count_total") != std::string::npos)
    {
        snapshot2_[0] = value;
    }
    else if(key.find("E2_rad") != std::string::npos)
    {
        snapshot2_[1] = value;
    }
    else if(key.find("E2_ω") != std::string::npos || key.find("E2_w") != std::string::npos)
    {
        snapshot2_[2] = value;
    }
    else if(key.find("E2_RPS") != std::string::npos)
    {
        snapshot2_[3] = value;
    }
    else if(key.find("E2_RPM") != std::string::npos)
    {
        snapshot2_[4] = value;
    }
}

void EncoderDriver::publish_odom()
{
    //現在の角度
    double rad1 = snapshot1_[1];
    double rad2 = snapshot2_[1];

    //角度の差分
    double d_rad1 = rad1 - prev_rad1_;
    double d_rad2 = rad2 - prev_rad2_;

    //従属輪の移動距離
    double caster_x = d_rad1 * radius_;
    double caster_y = d_rad2 * radius_;

    //次回のために保存
    prev_rad1_ = rad1;
    prev_rad2_ = rad2;

    //従属輪の移動速度
    double caster_vel_x = snapshot1_[2] * radius_;
    double caster_vel_y = snapshot2_[2] * radius_;

    //車体の回転量
    double theta = caster_y / l_;
    yaw_ += theta;

    //車体の移動量
    double dx = caster_x * cos(yaw_);
    double dy = caster_y * sin(yaw_);
    x_ += dx;
    y_ += dy;

    //クォータニオン変換(yawのみ)
    geometry_msgs::msg::Quaternion q;
    q.z = sin(yaw_ / 2.0);
    q.w = cos(yaw_ / 2.0);

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = q;

    msg.twist.twist.linear.x = caster_vel_x;
    msg.twist.twist.angular.z = caster_vel_y / l_;

    odom_pub_->publish(msg);
}

void EncoderDriver::spin()
{
    try
    {
        serial::Serial ser(DEFAULT_PORT, BAUD, serial::Timeout::simpleTimeout(10));
        RCLCPP_INFO(this->get_logger(), "Opening %s at %d bps. Ctrl-c to exit.", DEFAULT_PORT, BAUD);

        while(rclcpp::ok())
        {
            std::string line = ser.readline(256, "\n");
            if(!line.empty())
            {
                parse_line(line);

                //UDP通信
                std::array<int, 2> ints = {static_cast<int>(snapshot1_[0]), static_cast<int>(snapshot2_[0])};
                std::array<float, 8> floats = {
                    snapshot1_[1], snapshot1_[2], snapshot1_[3], snapshot1_[4],
                    snapshot2_[1], snapshot2_[2], snapshot2_[3], snapshot2_[4]
                };
                udp_sender_->send(ints, floats);

                publish_odom();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }
    catch(serial::IOException &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Serial error: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderDriver>();
    node->spin();
    rclcpp::shutdown();
    return 0;
}
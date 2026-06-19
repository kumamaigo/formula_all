#ifndef AIFORMULA_KEYOP_HPP_
#define AIFORMULA_KEYOP_HPP_

#include <termios.h>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include "oit_interfaces/msg/keyboard_input.hpp"

namespace aiformula_keyop
{

class KeyOp final : public rclcpp::Node
{
public:
    explicit KeyOp(const rclcpp::NodeOptions & options);
    ~KeyOp() override;

    // Disable copy and move
    KeyOp(KeyOp &&) = delete;
    KeyOp & operator=(KeyOp &&) = delete;
    KeyOp(const KeyOp &) = delete;
    KeyOp & operator=(const KeyOp &) = delete;

private:
    // ROS entities
    rclcpp::Subscription<oit_interfaces::msg::KeyboardInput>::SharedPtr keyinput_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_publisher_;
    std::shared_ptr<rclcpp::TimerBase> timer_;

    // Control state
    bool last_zero_vel_sent_;
    std::mutex cmd_mutex_;
    std::shared_ptr<geometry_msgs::msg::Twist> cmd_;

    // Keyboard handling
    std::atomic<bool> quit_requested_;
    int key_file_descriptor_;  // stdin
    struct termios original_terminal_state_;
    std::thread thread_;

    // Main control loop
    void spin();

    // Velocity commands
    void enable();
    void disable();
    void incrementLinearVelocity();
    void decrementLinearVelocity();
    void incrementAngularVelocity();
    void decrementAngularVelocity();
    void resetVelocity();
    void publishLock(bool state);

    // Input handling
    void keyboardInputLoop();
    void processKeyboardInput(char c);
    void remoteKeyInputReceived(const std::shared_ptr<oit_interfaces::msg::KeyboardInput> key);

    // Terminal handling
    void restoreTerminal();
};

} // namespace aiformula_keyop

#endif // AIFORMULA_KEYOP_HPP_

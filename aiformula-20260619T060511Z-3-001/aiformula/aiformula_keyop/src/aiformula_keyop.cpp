#include "aiformula_keyop/keyop.hpp"

#include <sys/select.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

namespace aiformula_keyop
{
    KeyOp::KeyOp(const rclcpp::NodeOptions & options)
    : Node("aiformula_keyop", options), last_zero_vel_sent_(true), quit_requested_(false), key_file_descriptor_(0)
    {
        tcgetattr(key_file_descriptor_, &original_terminal_state_);
        cmd_ = std::make_shared<geometry_msgs::msg::Twist>();

        double linear_vel_step = this->declare_parameter("linear_vel_step", 0.1);
        double linear_vel_max = this->declare_parameter("linear_vel_max", 4.0);
        double angular_vel_step = this->declare_parameter("angular_vel_step", 0.1);
        double angular_vel_max = this->declare_parameter("angular_vel_max", 3.0);

        RCLCPP_INFO(get_logger(), "KeyOp : using linear vel step [%f], max[%f]", linear_vel_step, linear_vel_max);
        RCLCPP_INFO(get_logger(), "KeyOp : using angular vel step [%f], max[%f]", angular_vel_step, angular_vel_max);

        keyinput_subscriber_ = this->create_subscription<oit_interfaces::msg::KeyboardInput>(
            "teleop", 1,
            std::bind(&KeyOp::remoteKeyInputReceived, this, std::placeholders::_1)
        );

        //velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/aiformula_control/key_board_controller/cmd_vel", 1);
        stop_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/aiformula_control/twist_mux/cmd_vel", 1);
        lock_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/aiformula_control/twist_mux/key_board_controller/lock", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KeyOp::spin, this)
        );

        thread_ = std::thread(&KeyOp::keyboardInputLoop, this);

        RCLCPP_INFO(get_logger(), "KeyOp : initialized.");
    }

    KeyOp::~KeyOp()
    {
        disable();
        quit_requested_ = true;
        if(thread_.joinable()) thread_.join();
        restoreTerminal();
    }

    void KeyOp::restoreTerminal()
    {
        tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
    }

    void KeyOp::spin()
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        if((cmd_->linear.x != 0.0) || (cmd_->angular.z != 0.0))
        {
            velocity_publisher_->publish(*cmd_);
            last_zero_vel_sent_ = false;
        }
        else if(!last_zero_vel_sent_)
        {
            velocity_publisher_->publish(*cmd_);
            last_zero_vel_sent_ = true;
        }
    }

    void KeyOp::keyboardInputLoop()
    {
        struct termios raw;
        std::memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        tcsetattr(key_file_descriptor_, TCSANOW, &raw);

        puts("Reading from keyboard");
        puts("---------------------");
        puts("↑/↓ : linear velocity incr/decr.");
        puts("→/← : angular velocity incr/decr.");
        puts("SPACE : reset linear/angular velocity.");
        puts("d : disable motors.");
        puts("e : enable motors.");

        while(!quit_requested_)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(key_file_descriptor_, &readfds);
            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 50 * 1000;

            int ret = ::select(key_file_descriptor_ + 1, &readfds, nullptr, nullptr, &timeout);

            if(ret > 0 && FD_ISSET(key_file_descriptor_, &readfds))
            {
                char c;
                if(::read(key_file_descriptor_, &c, 1) > 0)
                {
                    processKeyboardInput(c);
                }
                else
                {
                    RCLCPP_ERROR(get_logger(), "Failed to read character: %s", ::strerror(errno));
                }
            }
            else if(ret < 0)
            {
                RCLCPP_ERROR(get_logger(), "Select failed: %s", ::strerror(errno));
            }
        }
    }

    void KeyOp::remoteKeyInputReceived(const std::shared_ptr<oit_interfaces::msg::KeyboardInput> key)
    {
        processKeyboardInput(key->pressed_key);
    }

    void KeyOp::processKeyboardInput(char c)
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);

        switch(c)
        {
            case oit_interfaces::msg::KeyboardInput::KEYCODE_LEFT:
                incrementAngularVelocity();
                break;
            case oit_interfaces::msg::KeyboardInput::KEYCODE_RIGHT:
                decrementAngularVelocity();
                break;
            case oit_interfaces::msg::KeyboardInput::KEYCODE_UP:
                incrementLinearVelocity();
                break;
            case oit_interfaces::msg::KeyboardInput::KEYCODE_DOWN:
                decrementLinearVelocity();
                break;
            case oit_interfaces::msg::KeyboardInput::KEYCODE_SPACE:
                resetVelocity();
                break;
            case 'd':
                publishLock(true);
                disable();
                break;
            case 'e':
                publishLock(false);
                enable();
                break;
            default:
                break;
        }
    }

    void KeyOp::disable()
    {
        cmd_->linear.x = 0.0;
        cmd_->angular.z = 0.0;
        velocity_publisher_->publish(*cmd_);
        RCLCPP_INFO(get_logger(), "KeyOp : Disable power to the device subsystem.");
    }
    
    void KeyOp::enable()
    {
        cmd_->linear.x = 0.0;
        cmd_->angular.z = 0.0;
        velocity_publisher_->publish(*cmd_);
        RCLCPP_INFO(get_logger(), "KeyOp : Enable power to the device subsystem.");
    }

    void KeyOp::incrementLinearVelocity()
    {
        double step = get_parameter("linear_vel_step").as_double();
        double max = get_parameter("linear_vel_max").as_double();
        if(cmd_->linear.x + step <= max) cmd_->linear.x += step;
        RCLCPP_INFO(get_logger(), "KeyOp : linear [%f], angular [%f]", cmd_->linear.x, cmd_->angular.z);
    }

    void KeyOp::decrementLinearVelocity()
    {
        double step = get_parameter("linear_vel_step").as_double();
        double max = get_parameter("linear_vel_max").as_double();
        if(cmd_->linear.x - step >= -max) cmd_->linear.x -= step;
        RCLCPP_INFO(get_logger(), "KeyOp : linear [%f], angular [%f]", cmd_->linear.x, cmd_->angular.z);
    }

    void KeyOp::incrementAngularVelocity()
    {
        double step = get_parameter("angular_vel_step").as_double();
        double max = get_parameter("angular_vel_max").as_double();
        if(cmd_->angular.z + step <= max) cmd_->angular.z += step;
        RCLCPP_INFO(get_logger(), "KeyOp : linear [%f], angular [%f]", cmd_->linear.x, cmd_->angular.z);
    }

    void KeyOp::decrementAngularVelocity()
    {
        double step = get_parameter("angular_vel_step").as_double();
        double max = get_parameter("angular_vel_max").as_double();
        if(cmd_->angular.z - step >= -max) cmd_->angular.z -= step;
        RCLCPP_INFO(get_logger(), "KeyOp : linear [%f], angular [%f]", cmd_->linear.x, cmd_->angular.z);
    }

    void KeyOp::resetVelocity()
    {
        cmd_->linear.x = 0.0;
        cmd_->angular.z = 0.0;
        stop_velocity_publisher_->publish(*cmd_);
        RCLCPP_INFO(get_logger(), "KeyOp : reset linear/angular velocities.");
    }

    void KeyOp::publishLock(bool state)
    {
        std_msgs::msg::Bool lock;
        lock.data = state;
        lock_publisher_->publish(lock);
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(aiformula_keyop::KeyOp)
#include <chrono>
#include <functional>
#include <memory>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "serial_msgs/msg/motor_currents.hpp"


class MotorControlNode: public rclcpp::Node {
public:

    MotorControlNode() : rclcpp::Node("motor_command_reader") {
        using std::placeholders::_1;
        _subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10, std::bind(&MotorControlNode::twist_callback, this, _1));
        _publisher = this->create_publisher<serial_msgs::msg::MotorCurrents>("motor_currents", 10);
        RCLCPP_INFO(this->get_logger(), "made publisher");


    }

private:

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr _subscription;
    rclcpp::Publisher<serial_msgs::msg::MotorCurrents>::SharedPtr _publisher;

    void twist_callback(const geometry_msgs::msg::TwistStamped& twist) {
        RCLCPP_INFO(this->get_logger(), "linear (x): %f, angular (z): %f", twist.twist.linear.x, twist.twist.angular.z);

        serial_msgs::msg::MotorCurrents msg;

        double linear = twist.twist.linear.x;
        double angular = twist.twist.angular.z;
        // angular = ((3 / (8 * 3.14)) * 127 * angular + 127);


        msg.left_wheels = speed_to_current(linear - angular / 2.5);
        msg.right_wheels = speed_to_current(linear + angular / 2.5);
        _publisher->publish(msg);
        
    }

    uint8_t speed_to_current(double speed) {
        double scaled_speed = ((speed + 1.0) / 2.0) * 254.0;
        if (scaled_speed < 0) {
            scaled_speed = 0;
        } else if (scaled_speed > 253) {
            scaled_speed = 253;
        }
        return static_cast<uint8_t>(scaled_speed);
    }

};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
}
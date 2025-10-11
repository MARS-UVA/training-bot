#include <chrono>
#include <functional>
#include <memory>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial_msgs/msg/motor_currents.hpp"


class MotorControlNode: public rclcpp::Node {
public:

    MotorControlNode() : rclcpp::Node("motor_control") {
        using std::placeholders::_1;
        _subscription = this->create_subscription<geometry_msgs::msg::Twist>("control/twist", 10, std::bind(&MotorControlNode::twist_callback, this, _1));
        _publisher = this->create_publisher<serial_msgs::msg::MotorCurrents>("motor_currents", 10);
    }

private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
    rclcpp::Publisher<serial_msgs::msg::MotorCurrents>::SharedPtr _publisher;

    void twist_callback(const geometry_msgs::msg::Twist& twist) {
        RCLCPP_INFO(this->get_logger(), "linear (x): %f, angular (z): %f", twist.linear.x, twist.angular.z);

        serial_msgs::msg::MotorCurrents msg;
        msg.left_wheels = speed_to_current(twist.linear.x - twist.angular.z);
        msg.right_wheels = speed_to_current(twist.linear.x + twist.angular.z);
        _publisher->publish(msg);
    }

    uint8_t speed_to_current(double speed) {
        double scaled_speed = ((speed + 1.0) / 2.0) * 254.0;
        if (scaled_speed < 0) {
            scaled_speed = 0;
        } else if (scaled_speed > 254) {
            scaled_speed = 254;
        }
        return static_cast<uint8_t>(scaled_speed);
    }

};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControlNode>());
    rclcpp::shutdown();
}

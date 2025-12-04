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
        RCLCPP_INFO(this->get_logger(), "linear (x): %f m/s, angular (z): %f rad/s", twist.twist.linear.x, twist.twist.angular.z);

        serial_msgs::msg::MotorCurrents msg;

        double robotLinearSpeed = twist.twist.linear.x;
        double robotAngularSpeed = twist.twist.angular.z;
        // angular = ((3 / (8 * 3.14)) * 127 * angular + 127);
        const double wheelRadius = 0.05;
        const double wheelDistance = 0.25;
        const double gearRatio = 24;
        const double motorRadius = wheelRadius / gearRatio;
        const double PI = 3.141595;
        const double MAX_WHEEL_ANGULAR_SPEED = 200.0 * 2.0 * PI / 60.0;

        double leftLinearVel = robotLinearSpeed - (wheelDistance / 2.0) * robotAngularSpeed;        
        double rightLinearVel = robotLinearSpeed + (wheelDistance / 2.0) * robotAngularSpeed;
        RCLCPP_INFO(this->get_logger(), "left linear speed: %f m/s, right linear speed: %f m/s", leftLinearVel, rightLinearVel);
        double leftAngularVel = leftLinearVel / wheelRadius;
        double rightAngularVel = rightLinearVel / wheelRadius;

        RCLCPP_INFO(this->get_logger(), "left angular speed: %f rad/s, right angular speed: %f m/s", leftLinearVel, rightLinearVel);
        double leftAngularVelNorm = leftAngularVel / MAX_WHEEL_ANGULAR_SPEED;
        double rightAngularVelNorm = rightAngularVel / MAX_WHEEL_ANGULAR_SPEED;

        RCLCPP_INFO(this->get_logger(), "left normalized angular speed: %f rad/s, right normalized angular speed: %f m/s", leftAngularVelNorm, rightAngularVelNorm);
        msg.left_wheels = speed_to_current(leftAngularVelNorm);
        msg.right_wheels = speed_to_current(rightAngularVelNorm);
        _publisher->publish(msg);
        
    }

    uint8_t speed_to_current(double speed) {
        // speed from -1 to 1 -> byte from 0 to 253
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
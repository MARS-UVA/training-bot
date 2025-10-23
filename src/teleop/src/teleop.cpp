#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include <SDL2/SDL.h>
#include "serial_msgs/msg/motor_currents.hpp"
// #include "serial_msgs/serial_msgs/msg/motor_currents.h"

using namespace std::chrono_literals;

class Teleop : public rclcpp::Node
{
public:
  Teleop()
  : Node("teleop"), count_(0)
  {
    if(SDL_Init(SDL_INIT_GAMECONTROLLER) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDL: %s", SDL_GetError());
      return;
    }
    // Open the first available controller
    if (SDL_NumJoysticks() < 1) {
      RCLCPP_WARN(this->get_logger(), "No controllers connected");
    } else {
      controller_ = SDL_GameControllerOpen(0);
      if (!controller_) RCLCPP_ERROR(this->get_logger(), "Failed to open controller: %s", SDL_GetError());
    }

    publisher_ = this->create_publisher<serial_msgs::msg::MotorCurrents>("motor_currents", 10);
    auto timer_callback =
      [this]() -> void {
        SDL_GameControllerUpdate();
        if(!controller_) return;

        float x_axis = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0f;
        float y_axis = -1 * SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x_axis, y_axis);
        // Get linear and angular component of robot based on max speed
        float linear_component = y_axis * FULL_FORWARD_MAGNITUDE;
        float angular_component = x_axis * (1 - FULL_FORWARD_MAGNITUDE);
        // Get wheel speeds based on linear and angular components (currently using arcade drive)
        auto convertToCurrent = [](float speed) -> Uint8 {
          float scaled_speed = ((speed + 1.0f) / 2.0f) * 254.0f;
          if(scaled_speed < 0) scaled_speed = 0;
          else if(scaled_speed > 254) scaled_speed = 254;
          return static_cast<Uint8>(scaled_speed);
        };
        Uint8 left_wheels_speed = convertToCurrent(linear_component - angular_component);
        Uint8 right_wheels_speed = convertToCurrent(linear_component + angular_component);
        // Send wheel speeds to serial node
        auto message = serial_msgs::msg::MotorCurrents();
        message.left_wheels = left_wheels_speed;
        message.right_wheels = right_wheels_speed;
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(10ms, timer_callback);
  }

private:
  float FULL_FORWARD_MAGNITUDE = 0.6;
  SDL_GameController* controller_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<serial_msgs::msg::MotorCurrents>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop>());
  rclcpp::shutdown();
  return 0;
}
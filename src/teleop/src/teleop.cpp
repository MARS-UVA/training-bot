#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include <SDL2/SDL.h>
#include "serial_msgs/msg/motor_currents.hpp"

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
        float y_axis = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f;
        RCLCPP_INFO(this->get_logger(), "left: %f, right: %f", x_axis, y_axis);
        // Get linear and angular component of robot based on max speed
        float linear_component = x_axis * FULL_FORWARD_MAGNITUDE;
        float angular_component = y_axis * (1 - FULL_FORWARD_MAGNITUDE);
        // Get wheel speeds based on linear and angular components (currently using arcade drive)
        Uint8 left_wheels_speed = (Uint8) ((linear_component - angular_component) * 127) + 127;
        Uint8 right_wheels_speed = (Uint8) ((linear_component + angular_component) * 127) + 127;
        // Send wheel speeds to serial node
        auto message = serial_msgs::msg::MotorCurrents();
        message.front_left = left_wheels_speed;
        message.front_right = right_wheels_speed;
        message.back_left = left_wheels_speed;
        message.back_right = right_wheels_speed;
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
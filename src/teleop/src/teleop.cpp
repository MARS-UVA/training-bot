#include <chrono>
#include <memory>
#include <string>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include <SDL2/SDL.h>
#include "geometry_msgs/msg/twist.hpp"

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

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("motor_currents", 10);
    auto timer_callback =
      [this]() -> void {
        SDL_GameControllerUpdate();
        if(!controller_) return;

        float x_axis = SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTX) / 32767.0f;
        float y_axis = -1 * SDL_GameControllerGetAxis(controller_, SDL_CONTROLLER_AXIS_LEFTY) / 32767.0f;
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x_axis, y_axis);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = y_axis * FULL_FORWARD_MAGNITUDE;
        msg.angular.z = -x_axis * (1 - FULL_FORWARD_MAGNITUDE);
        this->publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(10ms, timer_callback);
  }

  ~Teleop() {
    SDL_GameControllerClose(controller_);
  }

private:
  float FULL_FORWARD_MAGNITUDE = 0.6;
  SDL_GameController* controller_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop>());
  rclcpp::shutdown();
  return 0;
}
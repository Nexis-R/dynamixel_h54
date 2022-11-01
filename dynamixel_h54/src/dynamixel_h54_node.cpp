#include "dynamixel_h54/dynamixel_h54.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamixel::Dynamixel>());
  rclcpp::shutdown();
}
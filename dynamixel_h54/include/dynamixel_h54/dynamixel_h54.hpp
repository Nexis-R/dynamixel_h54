#pragma once

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace dynamixel {

constexpr uint16_t ADDR_TOURQUE_ENABLE = 562;
constexpr uint16_t ADDR_PRESENT_POSITION = 611;
constexpr uint16_t ADDR_GOAL_POSITION = 596;
constexpr uint16_t ADDR_GOAL_VELOCITY = 600;
constexpr uint32_t PULSE_PER_REV = 501923;

struct Motor {
  uint16_t id;
  PortHandler* portHandler;
  PacketHandler* packetHandler;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_angle_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_velocity_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub;

  void handle_command_angle(const std_msgs::msg::Float32::SharedPtr angle);
  void handle_command_velocity(const std_msgs::msg::Float32::SharedPtr angle);
};

class Dynamixel : public rclcpp::Node {
 public:
  Dynamixel();

 private:
  void handle_pub_data();

  PortHandler* portHandler;
  PacketHandler* packetHandler;

  rclcpp::TimerBase::SharedPtr timer;

  std::map<int64_t, Motor> motors;
};
}  // namespace dynamixel
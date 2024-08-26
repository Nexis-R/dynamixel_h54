#include "dynamixel_h54/dynamixel_h54.hpp"

using namespace dynamixel;

using std::placeholders::_1;
using std::placeholders::_2;

Dynamixel::Dynamixel() : Node("dynamixel_h54") {
  auto logger = this->get_logger();

  this->declare_parameter("portname", "/dev/ttyUSB0");
  this->declare_parameter("baudrate", 1'000'000);
  this->declare_parameter("ids", std::vector<int64_t>{{1}});

  portHandler = PortHandler::getPortHandler(
      this->get_parameter("portname").as_string().c_str());
  packetHandler = PacketHandler::getPacketHandler();

  if (portHandler->openPort()) {
    RCLCPP_INFO(logger, "Successfully opened port!");
  } else {
    RCLCPP_ERROR(logger, "Failed to open port!");
  }

  if (portHandler->setBaudRate(this->get_parameter("baudrate").as_int())) {
    RCLCPP_INFO(logger, "Successfully set baudrate!");
  } else {
    RCLCPP_ERROR(logger, "Failed to set baudrate!");
  }

  const auto ids = this->get_parameter("ids").as_integer_array();
  for (const auto& id : ids) {
    packetHandler->write1ByteTxOnly(portHandler, id, ADDR_TOURQUE_ENABLE, 1);
    const auto result = packetHandler->ping(portHandler, id);
    RCLCPP_INFO_STREAM(logger, "id: " << id << "  ping result: " << result);

    Motor motor;
    motor.portHandler = portHandler;
    motor.packetHandler = packetHandler;
    motor.id = id;
    motors[id] = motor;

    const std::string motor_namespace = "~/motor" + std::to_string(id);
    motors[id].command_angle_sub =
        this->create_subscription<std_msgs::msg::Float32>(
            motor_namespace + "/command_angle", 1,
            std::bind(&Motor::handle_command_angle, &motors[id], _1));

    motors[id].command_velocity_sub =
        this->create_subscription<std_msgs::msg::Float32>(
            motor_namespace + "/command_velocity", 1,
            std::bind(&Motor::handle_command_velocity, &motors[id], _1));

    motors[id].angle_pub = this->create_publisher<std_msgs::msg::Float32>(
        motor_namespace + "/angle", 1);
  }

  this->declare_parameter("dataout_freq", 10);
  const auto dataout_freq = this->get_parameter("dataout_freq").as_int();
  timer =
      this->create_wall_timer(std::chrono::milliseconds(1000 / dataout_freq),
                              std::bind(&Dynamixel::handle_pub_data, this));
}

void Dynamixel::handle_pub_data() {
  for (auto& [id, motor] : motors) {
    int32_t pulse;
    motor.packetHandler->read4ByteTxRx(
        motor.portHandler, id, ADDR_PRESENT_POSITION, (uint32_t*)&pulse);
    const float angle = (pulse / PULSE_PER_REV) * M_PI * 2;

    std_msgs::msg::Float32 msg;
    msg.data = angle;
    motor.angle_pub->publish(msg);
  }
}

void Motor::handle_command_angle(
    const std_msgs::msg::Float32::SharedPtr angle) {
  packetHandler->write4ByteTxRx(portHandler, this->id, ADDR_GOAL_POSITION,
                                angle->data * PULSE_PER_REV / (2 * M_PI));
}

void Motor::handle_command_velocity(
    const std_msgs::msg::Float32::SharedPtr angle) {
  packetHandler->write4ByteTxRx(
      portHandler, this->id, ADDR_GOAL_VELOCITY,
      (int32_t)(angle->data / 0.104719755 / 0.00199234));
}
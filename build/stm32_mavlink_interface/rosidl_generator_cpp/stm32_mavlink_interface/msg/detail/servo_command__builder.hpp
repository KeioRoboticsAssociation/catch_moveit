// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/servo_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_ServoCommand_enable
{
public:
  explicit Init_ServoCommand_enable(::stm32_mavlink_interface::msg::ServoCommand & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::ServoCommand enable(::stm32_mavlink_interface::msg::ServoCommand::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoCommand msg_;
};

class Init_ServoCommand_pulse_us
{
public:
  explicit Init_ServoCommand_pulse_us(::stm32_mavlink_interface::msg::ServoCommand & msg)
  : msg_(msg)
  {}
  Init_ServoCommand_enable pulse_us(::stm32_mavlink_interface::msg::ServoCommand::_pulse_us_type arg)
  {
    msg_.pulse_us = std::move(arg);
    return Init_ServoCommand_enable(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoCommand msg_;
};

class Init_ServoCommand_angle_deg
{
public:
  explicit Init_ServoCommand_angle_deg(::stm32_mavlink_interface::msg::ServoCommand & msg)
  : msg_(msg)
  {}
  Init_ServoCommand_pulse_us angle_deg(::stm32_mavlink_interface::msg::ServoCommand::_angle_deg_type arg)
  {
    msg_.angle_deg = std::move(arg);
    return Init_ServoCommand_pulse_us(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoCommand msg_;
};

class Init_ServoCommand_servo_id
{
public:
  explicit Init_ServoCommand_servo_id(::stm32_mavlink_interface::msg::ServoCommand & msg)
  : msg_(msg)
  {}
  Init_ServoCommand_angle_deg servo_id(::stm32_mavlink_interface::msg::ServoCommand::_servo_id_type arg)
  {
    msg_.servo_id = std::move(arg);
    return Init_ServoCommand_angle_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoCommand msg_;
};

class Init_ServoCommand_header
{
public:
  Init_ServoCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoCommand_servo_id header(::stm32_mavlink_interface::msg::ServoCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ServoCommand_servo_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::ServoCommand>()
{
  return stm32_mavlink_interface::msg::builder::Init_ServoCommand_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__BUILDER_HPP_

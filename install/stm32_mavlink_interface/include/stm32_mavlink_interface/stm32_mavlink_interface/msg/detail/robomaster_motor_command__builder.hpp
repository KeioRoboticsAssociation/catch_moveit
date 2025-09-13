// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_RobomasterMotorCommand_emergency_stop
{
public:
  explicit Init_RobomasterMotorCommand_emergency_stop(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand emergency_stop(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_reset_watchdog
{
public:
  explicit Init_RobomasterMotorCommand_reset_watchdog(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_emergency_stop reset_watchdog(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_reset_watchdog_type arg)
  {
    msg_.reset_watchdog = std::move(arg);
    return Init_RobomasterMotorCommand_emergency_stop(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_enabled
{
public:
  explicit Init_RobomasterMotorCommand_enabled(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_reset_watchdog enabled(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return Init_RobomasterMotorCommand_reset_watchdog(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_target_position_rad
{
public:
  explicit Init_RobomasterMotorCommand_target_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_enabled target_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_target_position_rad_type arg)
  {
    msg_.target_position_rad = std::move(arg);
    return Init_RobomasterMotorCommand_enabled(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_target_velocity_rps
{
public:
  explicit Init_RobomasterMotorCommand_target_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_target_position_rad target_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_target_velocity_rps_type arg)
  {
    msg_.target_velocity_rps = std::move(arg);
    return Init_RobomasterMotorCommand_target_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_target_current_ma
{
public:
  explicit Init_RobomasterMotorCommand_target_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_target_velocity_rps target_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_target_current_ma_type arg)
  {
    msg_.target_current_ma = std::move(arg);
    return Init_RobomasterMotorCommand_target_velocity_rps(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_control_mode
{
public:
  explicit Init_RobomasterMotorCommand_control_mode(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_target_current_ma control_mode(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_RobomasterMotorCommand_target_current_ma(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_motor_id
{
public:
  explicit Init_RobomasterMotorCommand_motor_id(::stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorCommand_control_mode motor_id(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_RobomasterMotorCommand_control_mode(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

class Init_RobomasterMotorCommand_header
{
public:
  Init_RobomasterMotorCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobomasterMotorCommand_motor_id header(::stm32_mavlink_interface::msg::RobomasterMotorCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobomasterMotorCommand_motor_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::RobomasterMotorCommand>()
{
  return stm32_mavlink_interface::msg::builder::Init_RobomasterMotorCommand_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__BUILDER_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_RobomasterMotorState_overheat_count
{
public:
  explicit Init_RobomasterMotorState_overheat_count(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::RobomasterMotorState overheat_count(::stm32_mavlink_interface::msg::RobomasterMotorState::_overheat_count_type arg)
  {
    msg_.overheat_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_error_count
{
public:
  explicit Init_RobomasterMotorState_error_count(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_overheat_count error_count(::stm32_mavlink_interface::msg::RobomasterMotorState::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return Init_RobomasterMotorState_overheat_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_timeout_count
{
public:
  explicit Init_RobomasterMotorState_timeout_count(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_error_count timeout_count(::stm32_mavlink_interface::msg::RobomasterMotorState::_timeout_count_type arg)
  {
    msg_.timeout_count = std::move(arg);
    return Init_RobomasterMotorState_error_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_saturation_count
{
public:
  explicit Init_RobomasterMotorState_saturation_count(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_timeout_count saturation_count(::stm32_mavlink_interface::msg::RobomasterMotorState::_saturation_count_type arg)
  {
    msg_.saturation_count = std::move(arg);
    return Init_RobomasterMotorState_timeout_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_last_command_time_ms
{
public:
  explicit Init_RobomasterMotorState_last_command_time_ms(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_saturation_count last_command_time_ms(::stm32_mavlink_interface::msg::RobomasterMotorState::_last_command_time_ms_type arg)
  {
    msg_.last_command_time_ms = std::move(arg);
    return Init_RobomasterMotorState_saturation_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_status
{
public:
  explicit Init_RobomasterMotorState_status(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_last_command_time_ms status(::stm32_mavlink_interface::msg::RobomasterMotorState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_RobomasterMotorState_last_command_time_ms(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_enabled
{
public:
  explicit Init_RobomasterMotorState_enabled(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_status enabled(::stm32_mavlink_interface::msg::RobomasterMotorState::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return Init_RobomasterMotorState_status(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_control_mode
{
public:
  explicit Init_RobomasterMotorState_control_mode(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_enabled control_mode(::stm32_mavlink_interface::msg::RobomasterMotorState::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_RobomasterMotorState_enabled(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_target_current_ma
{
public:
  explicit Init_RobomasterMotorState_target_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_control_mode target_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorState::_target_current_ma_type arg)
  {
    msg_.target_current_ma = std::move(arg);
    return Init_RobomasterMotorState_control_mode(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_target_velocity_rps
{
public:
  explicit Init_RobomasterMotorState_target_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_target_current_ma target_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorState::_target_velocity_rps_type arg)
  {
    msg_.target_velocity_rps = std::move(arg);
    return Init_RobomasterMotorState_target_current_ma(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_target_position_rad
{
public:
  explicit Init_RobomasterMotorState_target_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_target_velocity_rps target_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorState::_target_position_rad_type arg)
  {
    msg_.target_position_rad = std::move(arg);
    return Init_RobomasterMotorState_target_velocity_rps(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_temperature_celsius
{
public:
  explicit Init_RobomasterMotorState_temperature_celsius(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_target_position_rad temperature_celsius(::stm32_mavlink_interface::msg::RobomasterMotorState::_temperature_celsius_type arg)
  {
    msg_.temperature_celsius = std::move(arg);
    return Init_RobomasterMotorState_target_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_current_milliamps
{
public:
  explicit Init_RobomasterMotorState_current_milliamps(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_temperature_celsius current_milliamps(::stm32_mavlink_interface::msg::RobomasterMotorState::_current_milliamps_type arg)
  {
    msg_.current_milliamps = std::move(arg);
    return Init_RobomasterMotorState_temperature_celsius(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_current_velocity_rps
{
public:
  explicit Init_RobomasterMotorState_current_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_current_milliamps current_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorState::_current_velocity_rps_type arg)
  {
    msg_.current_velocity_rps = std::move(arg);
    return Init_RobomasterMotorState_current_milliamps(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_current_position_rad
{
public:
  explicit Init_RobomasterMotorState_current_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_current_velocity_rps current_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorState::_current_position_rad_type arg)
  {
    msg_.current_position_rad = std::move(arg);
    return Init_RobomasterMotorState_current_velocity_rps(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_motor_id
{
public:
  explicit Init_RobomasterMotorState_motor_id(::stm32_mavlink_interface::msg::RobomasterMotorState & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorState_current_position_rad motor_id(::stm32_mavlink_interface::msg::RobomasterMotorState::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_RobomasterMotorState_current_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

class Init_RobomasterMotorState_header
{
public:
  Init_RobomasterMotorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobomasterMotorState_motor_id header(::stm32_mavlink_interface::msg::RobomasterMotorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobomasterMotorState_motor_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::RobomasterMotorState>()
{
  return stm32_mavlink_interface::msg::builder::Init_RobomasterMotorState_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__BUILDER_HPP_

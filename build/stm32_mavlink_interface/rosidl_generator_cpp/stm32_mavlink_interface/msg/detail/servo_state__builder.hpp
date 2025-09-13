// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/ServoState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/servo_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_ServoState_error_count
{
public:
  explicit Init_ServoState_error_count(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::ServoState error_count(::stm32_mavlink_interface::msg::ServoState::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_status
{
public:
  explicit Init_ServoState_status(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_error_count status(::stm32_mavlink_interface::msg::ServoState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ServoState_error_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_enabled
{
public:
  explicit Init_ServoState_enabled(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_status enabled(::stm32_mavlink_interface::msg::ServoState::_enabled_type arg)
  {
    msg_.enabled = std::move(arg);
    return Init_ServoState_status(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_pulse_us
{
public:
  explicit Init_ServoState_pulse_us(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_enabled pulse_us(::stm32_mavlink_interface::msg::ServoState::_pulse_us_type arg)
  {
    msg_.pulse_us = std::move(arg);
    return Init_ServoState_enabled(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_target_angle_deg
{
public:
  explicit Init_ServoState_target_angle_deg(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_pulse_us target_angle_deg(::stm32_mavlink_interface::msg::ServoState::_target_angle_deg_type arg)
  {
    msg_.target_angle_deg = std::move(arg);
    return Init_ServoState_pulse_us(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_current_angle_deg
{
public:
  explicit Init_ServoState_current_angle_deg(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_target_angle_deg current_angle_deg(::stm32_mavlink_interface::msg::ServoState::_current_angle_deg_type arg)
  {
    msg_.current_angle_deg = std::move(arg);
    return Init_ServoState_target_angle_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_servo_id
{
public:
  explicit Init_ServoState_servo_id(::stm32_mavlink_interface::msg::ServoState & msg)
  : msg_(msg)
  {}
  Init_ServoState_current_angle_deg servo_id(::stm32_mavlink_interface::msg::ServoState::_servo_id_type arg)
  {
    msg_.servo_id = std::move(arg);
    return Init_ServoState_current_angle_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

class Init_ServoState_header
{
public:
  Init_ServoState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoState_servo_id header(::stm32_mavlink_interface::msg::ServoState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ServoState_servo_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::ServoState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::ServoState>()
{
  return stm32_mavlink_interface::msg::builder::Init_ServoState_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__BUILDER_HPP_

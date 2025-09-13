// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetServoConfig_Request_save_to_flash
{
public:
  explicit Init_SetServoConfig_Request_save_to_flash(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetServoConfig_Request save_to_flash(::stm32_mavlink_interface::srv::SetServoConfig_Request::_save_to_flash_type arg)
  {
    msg_.save_to_flash = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_fail_safe_behavior
{
public:
  explicit Init_SetServoConfig_Request_fail_safe_behavior(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_save_to_flash fail_safe_behavior(::stm32_mavlink_interface::srv::SetServoConfig_Request::_fail_safe_behavior_type arg)
  {
    msg_.fail_safe_behavior = std::move(arg);
    return Init_SetServoConfig_Request_save_to_flash(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_watchdog_timeout_ms
{
public:
  explicit Init_SetServoConfig_Request_watchdog_timeout_ms(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_fail_safe_behavior watchdog_timeout_ms(::stm32_mavlink_interface::srv::SetServoConfig_Request::_watchdog_timeout_ms_type arg)
  {
    msg_.watchdog_timeout_ms = std::move(arg);
    return Init_SetServoConfig_Request_fail_safe_behavior(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_max_acceleration_deg_per_s2
{
public:
  explicit Init_SetServoConfig_Request_max_acceleration_deg_per_s2(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_watchdog_timeout_ms max_acceleration_deg_per_s2(::stm32_mavlink_interface::srv::SetServoConfig_Request::_max_acceleration_deg_per_s2_type arg)
  {
    msg_.max_acceleration_deg_per_s2 = std::move(arg);
    return Init_SetServoConfig_Request_watchdog_timeout_ms(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_max_velocity_deg_per_s
{
public:
  explicit Init_SetServoConfig_Request_max_velocity_deg_per_s(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_max_acceleration_deg_per_s2 max_velocity_deg_per_s(::stm32_mavlink_interface::srv::SetServoConfig_Request::_max_velocity_deg_per_s_type arg)
  {
    msg_.max_velocity_deg_per_s = std::move(arg);
    return Init_SetServoConfig_Request_max_acceleration_deg_per_s2(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_offset_deg
{
public:
  explicit Init_SetServoConfig_Request_offset_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_max_velocity_deg_per_s offset_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request::_offset_deg_type arg)
  {
    msg_.offset_deg = std::move(arg);
    return Init_SetServoConfig_Request_max_velocity_deg_per_s(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_direction_inverted
{
public:
  explicit Init_SetServoConfig_Request_direction_inverted(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_offset_deg direction_inverted(::stm32_mavlink_interface::srv::SetServoConfig_Request::_direction_inverted_type arg)
  {
    msg_.direction_inverted = std::move(arg);
    return Init_SetServoConfig_Request_offset_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_pulse_neutral_us
{
public:
  explicit Init_SetServoConfig_Request_pulse_neutral_us(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_direction_inverted pulse_neutral_us(::stm32_mavlink_interface::srv::SetServoConfig_Request::_pulse_neutral_us_type arg)
  {
    msg_.pulse_neutral_us = std::move(arg);
    return Init_SetServoConfig_Request_direction_inverted(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_pulse_max_us
{
public:
  explicit Init_SetServoConfig_Request_pulse_max_us(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_pulse_neutral_us pulse_max_us(::stm32_mavlink_interface::srv::SetServoConfig_Request::_pulse_max_us_type arg)
  {
    msg_.pulse_max_us = std::move(arg);
    return Init_SetServoConfig_Request_pulse_neutral_us(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_pulse_min_us
{
public:
  explicit Init_SetServoConfig_Request_pulse_min_us(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_pulse_max_us pulse_min_us(::stm32_mavlink_interface::srv::SetServoConfig_Request::_pulse_min_us_type arg)
  {
    msg_.pulse_min_us = std::move(arg);
    return Init_SetServoConfig_Request_pulse_max_us(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_angle_max_deg
{
public:
  explicit Init_SetServoConfig_Request_angle_max_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_pulse_min_us angle_max_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request::_angle_max_deg_type arg)
  {
    msg_.angle_max_deg = std::move(arg);
    return Init_SetServoConfig_Request_pulse_min_us(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_angle_min_deg
{
public:
  explicit Init_SetServoConfig_Request_angle_min_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetServoConfig_Request_angle_max_deg angle_min_deg(::stm32_mavlink_interface::srv::SetServoConfig_Request::_angle_min_deg_type arg)
  {
    msg_.angle_min_deg = std::move(arg);
    return Init_SetServoConfig_Request_angle_max_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

class Init_SetServoConfig_Request_servo_id
{
public:
  Init_SetServoConfig_Request_servo_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetServoConfig_Request_angle_min_deg servo_id(::stm32_mavlink_interface::srv::SetServoConfig_Request::_servo_id_type arg)
  {
    msg_.servo_id = std::move(arg);
    return Init_SetServoConfig_Request_angle_min_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetServoConfig_Request>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetServoConfig_Request_servo_id();
}

}  // namespace stm32_mavlink_interface


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetServoConfig_Response_message
{
public:
  explicit Init_SetServoConfig_Response_message(::stm32_mavlink_interface::srv::SetServoConfig_Response & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetServoConfig_Response message(::stm32_mavlink_interface::srv::SetServoConfig_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Response msg_;
};

class Init_SetServoConfig_Response_success
{
public:
  Init_SetServoConfig_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetServoConfig_Response_message success(::stm32_mavlink_interface::srv::SetServoConfig_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetServoConfig_Response_message(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetServoConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetServoConfig_Response>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetServoConfig_Response_success();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__BUILDER_HPP_

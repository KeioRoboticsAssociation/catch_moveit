// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_RobomasterMotorConfig_position_offset_rad
{
public:
  explicit Init_RobomasterMotorConfig_position_offset_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig position_offset_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_position_offset_rad_type arg)
  {
    msg_.position_offset_rad = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_direction_inverted
{
public:
  explicit Init_RobomasterMotorConfig_direction_inverted(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_position_offset_rad direction_inverted(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_direction_inverted_type arg)
  {
    msg_.direction_inverted = std::move(arg);
    return Init_RobomasterMotorConfig_position_offset_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_start_disabled
{
public:
  explicit Init_RobomasterMotorConfig_start_disabled(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_direction_inverted start_disabled(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_start_disabled_type arg)
  {
    msg_.start_disabled = std::move(arg);
    return Init_RobomasterMotorConfig_direction_inverted(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_startup_mode
{
public:
  explicit Init_RobomasterMotorConfig_startup_mode(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_start_disabled startup_mode(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_startup_mode_type arg)
  {
    msg_.startup_mode = std::move(arg);
    return Init_RobomasterMotorConfig_start_disabled(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_startup_position_rad
{
public:
  explicit Init_RobomasterMotorConfig_startup_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_startup_mode startup_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_startup_position_rad_type arg)
  {
    msg_.startup_position_rad = std::move(arg);
    return Init_RobomasterMotorConfig_startup_mode(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_velocity_kd
{
public:
  explicit Init_RobomasterMotorConfig_velocity_kd(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_startup_position_rad velocity_kd(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_velocity_kd_type arg)
  {
    msg_.velocity_kd = std::move(arg);
    return Init_RobomasterMotorConfig_startup_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_velocity_ki
{
public:
  explicit Init_RobomasterMotorConfig_velocity_ki(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_velocity_kd velocity_ki(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_velocity_ki_type arg)
  {
    msg_.velocity_ki = std::move(arg);
    return Init_RobomasterMotorConfig_velocity_kd(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_velocity_kp
{
public:
  explicit Init_RobomasterMotorConfig_velocity_kp(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_velocity_ki velocity_kp(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_velocity_kp_type arg)
  {
    msg_.velocity_kp = std::move(arg);
    return Init_RobomasterMotorConfig_velocity_ki(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_position_kd
{
public:
  explicit Init_RobomasterMotorConfig_position_kd(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_velocity_kp position_kd(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_position_kd_type arg)
  {
    msg_.position_kd = std::move(arg);
    return Init_RobomasterMotorConfig_velocity_kp(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_position_ki
{
public:
  explicit Init_RobomasterMotorConfig_position_ki(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_position_kd position_ki(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_position_ki_type arg)
  {
    msg_.position_ki = std::move(arg);
    return Init_RobomasterMotorConfig_position_kd(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_position_kp
{
public:
  explicit Init_RobomasterMotorConfig_position_kp(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_position_ki position_kp(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_position_kp_type arg)
  {
    msg_.position_kp = std::move(arg);
    return Init_RobomasterMotorConfig_position_ki(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_failsafe_behavior
{
public:
  explicit Init_RobomasterMotorConfig_failsafe_behavior(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_position_kp failsafe_behavior(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_failsafe_behavior_type arg)
  {
    msg_.failsafe_behavior = std::move(arg);
    return Init_RobomasterMotorConfig_position_kp(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_max_temperature_celsius
{
public:
  explicit Init_RobomasterMotorConfig_max_temperature_celsius(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_failsafe_behavior max_temperature_celsius(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_max_temperature_celsius_type arg)
  {
    msg_.max_temperature_celsius = std::move(arg);
    return Init_RobomasterMotorConfig_failsafe_behavior(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_watchdog_timeout_ms
{
public:
  explicit Init_RobomasterMotorConfig_watchdog_timeout_ms(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_max_temperature_celsius watchdog_timeout_ms(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_watchdog_timeout_ms_type arg)
  {
    msg_.watchdog_timeout_ms = std::move(arg);
    return Init_RobomasterMotorConfig_max_temperature_celsius(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_position_limits_enabled
{
public:
  explicit Init_RobomasterMotorConfig_position_limits_enabled(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_watchdog_timeout_ms position_limits_enabled(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_position_limits_enabled_type arg)
  {
    msg_.position_limits_enabled = std::move(arg);
    return Init_RobomasterMotorConfig_watchdog_timeout_ms(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_max_position_rad
{
public:
  explicit Init_RobomasterMotorConfig_max_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_position_limits_enabled max_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_max_position_rad_type arg)
  {
    msg_.max_position_rad = std::move(arg);
    return Init_RobomasterMotorConfig_position_limits_enabled(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_min_position_rad
{
public:
  explicit Init_RobomasterMotorConfig_min_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_max_position_rad min_position_rad(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_min_position_rad_type arg)
  {
    msg_.min_position_rad = std::move(arg);
    return Init_RobomasterMotorConfig_max_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_min_current_ma
{
public:
  explicit Init_RobomasterMotorConfig_min_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_min_position_rad min_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_min_current_ma_type arg)
  {
    msg_.min_current_ma = std::move(arg);
    return Init_RobomasterMotorConfig_min_position_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_max_current_ma
{
public:
  explicit Init_RobomasterMotorConfig_max_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_min_current_ma max_current_ma(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_max_current_ma_type arg)
  {
    msg_.max_current_ma = std::move(arg);
    return Init_RobomasterMotorConfig_min_current_ma(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_max_acceleration_rps2
{
public:
  explicit Init_RobomasterMotorConfig_max_acceleration_rps2(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_max_current_ma max_acceleration_rps2(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_max_acceleration_rps2_type arg)
  {
    msg_.max_acceleration_rps2 = std::move(arg);
    return Init_RobomasterMotorConfig_max_current_ma(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_max_velocity_rps
{
public:
  explicit Init_RobomasterMotorConfig_max_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_max_acceleration_rps2 max_velocity_rps(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_max_velocity_rps_type arg)
  {
    msg_.max_velocity_rps = std::move(arg);
    return Init_RobomasterMotorConfig_max_acceleration_rps2(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_motor_id
{
public:
  explicit Init_RobomasterMotorConfig_motor_id(::stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
  : msg_(msg)
  {}
  Init_RobomasterMotorConfig_max_velocity_rps motor_id(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_RobomasterMotorConfig_max_velocity_rps(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

class Init_RobomasterMotorConfig_header
{
public:
  Init_RobomasterMotorConfig_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobomasterMotorConfig_motor_id header(::stm32_mavlink_interface::msg::RobomasterMotorConfig::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobomasterMotorConfig_motor_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::RobomasterMotorConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::RobomasterMotorConfig>()
{
  return stm32_mavlink_interface::msg::builder::Init_RobomasterMotorConfig_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

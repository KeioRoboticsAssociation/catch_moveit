// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/encoder_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_EncoderConfig_wrap_around
{
public:
  explicit Init_EncoderConfig_wrap_around(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::EncoderConfig wrap_around(::stm32_mavlink_interface::msg::EncoderConfig::_wrap_around_type arg)
  {
    msg_.wrap_around = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_offset_counts
{
public:
  explicit Init_EncoderConfig_offset_counts(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_wrap_around offset_counts(::stm32_mavlink_interface::msg::EncoderConfig::_offset_counts_type arg)
  {
    msg_.offset_counts = std::move(arg);
    return Init_EncoderConfig_wrap_around(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_watchdog_timeout_ms
{
public:
  explicit Init_EncoderConfig_watchdog_timeout_ms(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_offset_counts watchdog_timeout_ms(::stm32_mavlink_interface::msg::EncoderConfig::_watchdog_timeout_ms_type arg)
  {
    msg_.watchdog_timeout_ms = std::move(arg);
    return Init_EncoderConfig_offset_counts(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_use_z
{
public:
  explicit Init_EncoderConfig_use_z(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_watchdog_timeout_ms use_z(::stm32_mavlink_interface::msg::EncoderConfig::_use_z_type arg)
  {
    msg_.use_z = std::move(arg);
    return Init_EncoderConfig_watchdog_timeout_ms(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_invert_b
{
public:
  explicit Init_EncoderConfig_invert_b(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_use_z invert_b(::stm32_mavlink_interface::msg::EncoderConfig::_invert_b_type arg)
  {
    msg_.invert_b = std::move(arg);
    return Init_EncoderConfig_use_z(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_invert_a
{
public:
  explicit Init_EncoderConfig_invert_a(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_invert_b invert_a(::stm32_mavlink_interface::msg::EncoderConfig::_invert_a_type arg)
  {
    msg_.invert_a = std::move(arg);
    return Init_EncoderConfig_invert_b(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_cpr
{
public:
  explicit Init_EncoderConfig_cpr(::stm32_mavlink_interface::msg::EncoderConfig & msg)
  : msg_(msg)
  {}
  Init_EncoderConfig_invert_a cpr(::stm32_mavlink_interface::msg::EncoderConfig::_cpr_type arg)
  {
    msg_.cpr = std::move(arg);
    return Init_EncoderConfig_invert_a(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

class Init_EncoderConfig_encoder_id
{
public:
  Init_EncoderConfig_encoder_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderConfig_cpr encoder_id(::stm32_mavlink_interface::msg::EncoderConfig::_encoder_id_type arg)
  {
    msg_.encoder_id = std::move(arg);
    return Init_EncoderConfig_cpr(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::EncoderConfig>()
{
  return stm32_mavlink_interface::msg::builder::Init_EncoderConfig_encoder_id();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__BUILDER_HPP_

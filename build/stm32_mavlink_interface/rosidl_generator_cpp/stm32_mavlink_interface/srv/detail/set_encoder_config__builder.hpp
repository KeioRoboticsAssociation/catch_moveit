// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetEncoderConfig_Request_save_to_flash
{
public:
  explicit Init_SetEncoderConfig_Request_save_to_flash(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request save_to_flash(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_save_to_flash_type arg)
  {
    msg_.save_to_flash = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_wrap_around
{
public:
  explicit Init_SetEncoderConfig_Request_wrap_around(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_save_to_flash wrap_around(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_wrap_around_type arg)
  {
    msg_.wrap_around = std::move(arg);
    return Init_SetEncoderConfig_Request_save_to_flash(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_offset_counts
{
public:
  explicit Init_SetEncoderConfig_Request_offset_counts(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_wrap_around offset_counts(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_offset_counts_type arg)
  {
    msg_.offset_counts = std::move(arg);
    return Init_SetEncoderConfig_Request_wrap_around(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_watchdog_timeout_ms
{
public:
  explicit Init_SetEncoderConfig_Request_watchdog_timeout_ms(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_offset_counts watchdog_timeout_ms(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_watchdog_timeout_ms_type arg)
  {
    msg_.watchdog_timeout_ms = std::move(arg);
    return Init_SetEncoderConfig_Request_offset_counts(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_use_z
{
public:
  explicit Init_SetEncoderConfig_Request_use_z(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_watchdog_timeout_ms use_z(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_use_z_type arg)
  {
    msg_.use_z = std::move(arg);
    return Init_SetEncoderConfig_Request_watchdog_timeout_ms(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_invert_b
{
public:
  explicit Init_SetEncoderConfig_Request_invert_b(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_use_z invert_b(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_invert_b_type arg)
  {
    msg_.invert_b = std::move(arg);
    return Init_SetEncoderConfig_Request_use_z(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_invert_a
{
public:
  explicit Init_SetEncoderConfig_Request_invert_a(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_invert_b invert_a(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_invert_a_type arg)
  {
    msg_.invert_a = std::move(arg);
    return Init_SetEncoderConfig_Request_invert_b(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_cpr
{
public:
  explicit Init_SetEncoderConfig_Request_cpr(::stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
  : msg_(msg)
  {}
  Init_SetEncoderConfig_Request_invert_a cpr(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_cpr_type arg)
  {
    msg_.cpr = std::move(arg);
    return Init_SetEncoderConfig_Request_invert_a(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

class Init_SetEncoderConfig_Request_encoder_id
{
public:
  Init_SetEncoderConfig_Request_encoder_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetEncoderConfig_Request_cpr encoder_id(::stm32_mavlink_interface::srv::SetEncoderConfig_Request::_encoder_id_type arg)
  {
    msg_.encoder_id = std::move(arg);
    return Init_SetEncoderConfig_Request_cpr(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetEncoderConfig_Request>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetEncoderConfig_Request_encoder_id();
}

}  // namespace stm32_mavlink_interface


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetEncoderConfig_Response_message
{
public:
  explicit Init_SetEncoderConfig_Response_message(::stm32_mavlink_interface::srv::SetEncoderConfig_Response & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Response message(::stm32_mavlink_interface::srv::SetEncoderConfig_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Response msg_;
};

class Init_SetEncoderConfig_Response_success
{
public:
  Init_SetEncoderConfig_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetEncoderConfig_Response_message success(::stm32_mavlink_interface::srv::SetEncoderConfig_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetEncoderConfig_Response_message(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetEncoderConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetEncoderConfig_Response>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetEncoderConfig_Response_success();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__BUILDER_HPP_

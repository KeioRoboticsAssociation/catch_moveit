// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/msg/detail/encoder_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace msg
{

namespace builder
{

class Init_EncoderState_error_count
{
public:
  explicit Init_EncoderState_error_count(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::msg::EncoderState error_count(::stm32_mavlink_interface::msg::EncoderState::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_status
{
public:
  explicit Init_EncoderState_status(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_error_count status(::stm32_mavlink_interface::msg::EncoderState::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_EncoderState_error_count(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_z_detected
{
public:
  explicit Init_EncoderState_z_detected(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_status z_detected(::stm32_mavlink_interface::msg::EncoderState::_z_detected_type arg)
  {
    msg_.z_detected = std::move(arg);
    return Init_EncoderState_status(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_revolutions
{
public:
  explicit Init_EncoderState_revolutions(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_z_detected revolutions(::stm32_mavlink_interface::msg::EncoderState::_revolutions_type arg)
  {
    msg_.revolutions = std::move(arg);
    return Init_EncoderState_z_detected(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_angle_deg
{
public:
  explicit Init_EncoderState_angle_deg(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_revolutions angle_deg(::stm32_mavlink_interface::msg::EncoderState::_angle_deg_type arg)
  {
    msg_.angle_deg = std::move(arg);
    return Init_EncoderState_revolutions(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_angle_rad
{
public:
  explicit Init_EncoderState_angle_rad(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_angle_deg angle_rad(::stm32_mavlink_interface::msg::EncoderState::_angle_rad_type arg)
  {
    msg_.angle_rad = std::move(arg);
    return Init_EncoderState_angle_deg(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_position
{
public:
  explicit Init_EncoderState_position(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_angle_rad position(::stm32_mavlink_interface::msg::EncoderState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_EncoderState_angle_rad(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_encoder_id
{
public:
  explicit Init_EncoderState_encoder_id(::stm32_mavlink_interface::msg::EncoderState & msg)
  : msg_(msg)
  {}
  Init_EncoderState_position encoder_id(::stm32_mavlink_interface::msg::EncoderState::_encoder_id_type arg)
  {
    msg_.encoder_id = std::move(arg);
    return Init_EncoderState_position(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

class Init_EncoderState_header
{
public:
  Init_EncoderState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderState_encoder_id header(::stm32_mavlink_interface::msg::EncoderState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EncoderState_encoder_id(msg_);
  }

private:
  ::stm32_mavlink_interface::msg::EncoderState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::msg::EncoderState>()
{
  return stm32_mavlink_interface::msg::builder::Init_EncoderState_header();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__BUILDER_HPP_

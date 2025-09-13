// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/encoder_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: encoder_id
  {
    out << "encoder_id: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_id, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: angle_rad
  {
    out << "angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_rad, out);
    out << ", ";
  }

  // member: angle_deg
  {
    out << "angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_deg, out);
    out << ", ";
  }

  // member: revolutions
  {
    out << "revolutions: ";
    rosidl_generator_traits::value_to_yaml(msg.revolutions, out);
    out << ", ";
  }

  // member: z_detected
  {
    out << "z_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.z_detected, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: error_count
  {
    out << "error_count: ";
    rosidl_generator_traits::value_to_yaml(msg.error_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: encoder_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "encoder_id: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_id, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: angle_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_rad, out);
    out << "\n";
  }

  // member: angle_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_deg, out);
    out << "\n";
  }

  // member: revolutions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "revolutions: ";
    rosidl_generator_traits::value_to_yaml(msg.revolutions, out);
    out << "\n";
  }

  // member: z_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.z_detected, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: error_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error_count: ";
    rosidl_generator_traits::value_to_yaml(msg.error_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace stm32_mavlink_interface

namespace rosidl_generator_traits
{

[[deprecated("use stm32_mavlink_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stm32_mavlink_interface::msg::EncoderState & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::EncoderState & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::EncoderState>()
{
  return "stm32_mavlink_interface::msg::EncoderState";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::EncoderState>()
{
  return "stm32_mavlink_interface/msg/EncoderState";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::EncoderState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::EncoderState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stm32_mavlink_interface::msg::EncoderState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__TRAITS_HPP_

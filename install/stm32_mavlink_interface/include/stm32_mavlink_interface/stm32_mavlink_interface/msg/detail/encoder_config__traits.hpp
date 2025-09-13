// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/encoder_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderConfig & msg,
  std::ostream & out)
{
  out << "{";
  // member: encoder_id
  {
    out << "encoder_id: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_id, out);
    out << ", ";
  }

  // member: cpr
  {
    out << "cpr: ";
    rosidl_generator_traits::value_to_yaml(msg.cpr, out);
    out << ", ";
  }

  // member: invert_a
  {
    out << "invert_a: ";
    rosidl_generator_traits::value_to_yaml(msg.invert_a, out);
    out << ", ";
  }

  // member: invert_b
  {
    out << "invert_b: ";
    rosidl_generator_traits::value_to_yaml(msg.invert_b, out);
    out << ", ";
  }

  // member: use_z
  {
    out << "use_z: ";
    rosidl_generator_traits::value_to_yaml(msg.use_z, out);
    out << ", ";
  }

  // member: watchdog_timeout_ms
  {
    out << "watchdog_timeout_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_timeout_ms, out);
    out << ", ";
  }

  // member: offset_counts
  {
    out << "offset_counts: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_counts, out);
    out << ", ";
  }

  // member: wrap_around
  {
    out << "wrap_around: ";
    rosidl_generator_traits::value_to_yaml(msg.wrap_around, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: encoder_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "encoder_id: ";
    rosidl_generator_traits::value_to_yaml(msg.encoder_id, out);
    out << "\n";
  }

  // member: cpr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpr: ";
    rosidl_generator_traits::value_to_yaml(msg.cpr, out);
    out << "\n";
  }

  // member: invert_a
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invert_a: ";
    rosidl_generator_traits::value_to_yaml(msg.invert_a, out);
    out << "\n";
  }

  // member: invert_b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "invert_b: ";
    rosidl_generator_traits::value_to_yaml(msg.invert_b, out);
    out << "\n";
  }

  // member: use_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_z: ";
    rosidl_generator_traits::value_to_yaml(msg.use_z, out);
    out << "\n";
  }

  // member: watchdog_timeout_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "watchdog_timeout_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_timeout_ms, out);
    out << "\n";
  }

  // member: offset_counts
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_counts: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_counts, out);
    out << "\n";
  }

  // member: wrap_around
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrap_around: ";
    rosidl_generator_traits::value_to_yaml(msg.wrap_around, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderConfig & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::msg::EncoderConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::EncoderConfig & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::EncoderConfig>()
{
  return "stm32_mavlink_interface::msg::EncoderConfig";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::EncoderConfig>()
{
  return "stm32_mavlink_interface/msg/EncoderConfig";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::EncoderConfig>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::EncoderConfig>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stm32_mavlink_interface::msg::EncoderConfig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/servo_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ServoCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: servo_id
  {
    out << "servo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_id, out);
    out << ", ";
  }

  // member: angle_deg
  {
    out << "angle_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_deg, out);
    out << ", ";
  }

  // member: pulse_us
  {
    out << "pulse_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_us, out);
    out << ", ";
  }

  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoCommand & msg,
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

  // member: servo_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_id, out);
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

  // member: pulse_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_us, out);
    out << "\n";
  }

  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoCommand & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::msg::ServoCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::ServoCommand & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::ServoCommand>()
{
  return "stm32_mavlink_interface::msg::ServoCommand";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::ServoCommand>()
{
  return "stm32_mavlink_interface/msg/ServoCommand";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::ServoCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::ServoCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stm32_mavlink_interface::msg::ServoCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobomasterMotorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: motor_id
  {
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: target_current_ma
  {
    out << "target_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.target_current_ma, out);
    out << ", ";
  }

  // member: target_velocity_rps
  {
    out << "target_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.target_velocity_rps, out);
    out << ", ";
  }

  // member: target_position_rad
  {
    out << "target_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position_rad, out);
    out << ", ";
  }

  // member: enabled
  {
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << ", ";
  }

  // member: reset_watchdog
  {
    out << "reset_watchdog: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_watchdog, out);
    out << ", ";
  }

  // member: emergency_stop
  {
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobomasterMotorCommand & msg,
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

  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << "\n";
  }

  // member: target_current_ma
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.target_current_ma, out);
    out << "\n";
  }

  // member: target_velocity_rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.target_velocity_rps, out);
    out << "\n";
  }

  // member: target_position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position_rad, out);
    out << "\n";
  }

  // member: enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << "\n";
  }

  // member: reset_watchdog
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset_watchdog: ";
    rosidl_generator_traits::value_to_yaml(msg.reset_watchdog, out);
    out << "\n";
  }

  // member: emergency_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobomasterMotorCommand & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::msg::RobomasterMotorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::RobomasterMotorCommand & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::RobomasterMotorCommand>()
{
  return "stm32_mavlink_interface::msg::RobomasterMotorCommand";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::RobomasterMotorCommand>()
{
  return "stm32_mavlink_interface/msg/RobomasterMotorCommand";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::RobomasterMotorCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::RobomasterMotorCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stm32_mavlink_interface::msg::RobomasterMotorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__TRAITS_HPP_

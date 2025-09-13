// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobomasterMotorState & msg,
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

  // member: current_position_rad
  {
    out << "current_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position_rad, out);
    out << ", ";
  }

  // member: current_velocity_rps
  {
    out << "current_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity_rps, out);
    out << ", ";
  }

  // member: current_milliamps
  {
    out << "current_milliamps: ";
    rosidl_generator_traits::value_to_yaml(msg.current_milliamps, out);
    out << ", ";
  }

  // member: temperature_celsius
  {
    out << "temperature_celsius: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature_celsius, out);
    out << ", ";
  }

  // member: target_position_rad
  {
    out << "target_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position_rad, out);
    out << ", ";
  }

  // member: target_velocity_rps
  {
    out << "target_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.target_velocity_rps, out);
    out << ", ";
  }

  // member: target_current_ma
  {
    out << "target_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.target_current_ma, out);
    out << ", ";
  }

  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: enabled
  {
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: last_command_time_ms
  {
    out << "last_command_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command_time_ms, out);
    out << ", ";
  }

  // member: saturation_count
  {
    out << "saturation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.saturation_count, out);
    out << ", ";
  }

  // member: timeout_count
  {
    out << "timeout_count: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout_count, out);
    out << ", ";
  }

  // member: error_count
  {
    out << "error_count: ";
    rosidl_generator_traits::value_to_yaml(msg.error_count, out);
    out << ", ";
  }

  // member: overheat_count
  {
    out << "overheat_count: ";
    rosidl_generator_traits::value_to_yaml(msg.overheat_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobomasterMotorState & msg,
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

  // member: current_position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.current_position_rad, out);
    out << "\n";
  }

  // member: current_velocity_rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.current_velocity_rps, out);
    out << "\n";
  }

  // member: current_milliamps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_milliamps: ";
    rosidl_generator_traits::value_to_yaml(msg.current_milliamps, out);
    out << "\n";
  }

  // member: temperature_celsius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature_celsius: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature_celsius, out);
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

  // member: target_velocity_rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.target_velocity_rps, out);
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

  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
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

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: last_command_time_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_command_time_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.last_command_time_ms, out);
    out << "\n";
  }

  // member: saturation_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "saturation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.saturation_count, out);
    out << "\n";
  }

  // member: timeout_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timeout_count: ";
    rosidl_generator_traits::value_to_yaml(msg.timeout_count, out);
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

  // member: overheat_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overheat_count: ";
    rosidl_generator_traits::value_to_yaml(msg.overheat_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobomasterMotorState & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::msg::RobomasterMotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::RobomasterMotorState & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::RobomasterMotorState>()
{
  return "stm32_mavlink_interface::msg::RobomasterMotorState";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::RobomasterMotorState>()
{
  return "stm32_mavlink_interface/msg/RobomasterMotorState";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::RobomasterMotorState>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::RobomasterMotorState>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stm32_mavlink_interface::msg::RobomasterMotorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace stm32_mavlink_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobomasterMotorConfig & msg,
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

  // member: max_velocity_rps
  {
    out << "max_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity_rps, out);
    out << ", ";
  }

  // member: max_acceleration_rps2
  {
    out << "max_acceleration_rps2: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration_rps2, out);
    out << ", ";
  }

  // member: max_current_ma
  {
    out << "max_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.max_current_ma, out);
    out << ", ";
  }

  // member: min_current_ma
  {
    out << "min_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.min_current_ma, out);
    out << ", ";
  }

  // member: min_position_rad
  {
    out << "min_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.min_position_rad, out);
    out << ", ";
  }

  // member: max_position_rad
  {
    out << "max_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position_rad, out);
    out << ", ";
  }

  // member: position_limits_enabled
  {
    out << "position_limits_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.position_limits_enabled, out);
    out << ", ";
  }

  // member: watchdog_timeout_ms
  {
    out << "watchdog_timeout_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_timeout_ms, out);
    out << ", ";
  }

  // member: max_temperature_celsius
  {
    out << "max_temperature_celsius: ";
    rosidl_generator_traits::value_to_yaml(msg.max_temperature_celsius, out);
    out << ", ";
  }

  // member: failsafe_behavior
  {
    out << "failsafe_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe_behavior, out);
    out << ", ";
  }

  // member: position_kp
  {
    out << "position_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kp, out);
    out << ", ";
  }

  // member: position_ki
  {
    out << "position_ki: ";
    rosidl_generator_traits::value_to_yaml(msg.position_ki, out);
    out << ", ";
  }

  // member: position_kd
  {
    out << "position_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kd, out);
    out << ", ";
  }

  // member: velocity_kp
  {
    out << "velocity_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_kp, out);
    out << ", ";
  }

  // member: velocity_ki
  {
    out << "velocity_ki: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_ki, out);
    out << ", ";
  }

  // member: velocity_kd
  {
    out << "velocity_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_kd, out);
    out << ", ";
  }

  // member: startup_position_rad
  {
    out << "startup_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_position_rad, out);
    out << ", ";
  }

  // member: startup_mode
  {
    out << "startup_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_mode, out);
    out << ", ";
  }

  // member: start_disabled
  {
    out << "start_disabled: ";
    rosidl_generator_traits::value_to_yaml(msg.start_disabled, out);
    out << ", ";
  }

  // member: direction_inverted
  {
    out << "direction_inverted: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_inverted, out);
    out << ", ";
  }

  // member: position_offset_rad
  {
    out << "position_offset_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_offset_rad, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RobomasterMotorConfig & msg,
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

  // member: max_velocity_rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_velocity_rps: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity_rps, out);
    out << "\n";
  }

  // member: max_acceleration_rps2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_acceleration_rps2: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration_rps2, out);
    out << "\n";
  }

  // member: max_current_ma
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.max_current_ma, out);
    out << "\n";
  }

  // member: min_current_ma
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_current_ma: ";
    rosidl_generator_traits::value_to_yaml(msg.min_current_ma, out);
    out << "\n";
  }

  // member: min_position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.min_position_rad, out);
    out << "\n";
  }

  // member: max_position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.max_position_rad, out);
    out << "\n";
  }

  // member: position_limits_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_limits_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.position_limits_enabled, out);
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

  // member: max_temperature_celsius
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_temperature_celsius: ";
    rosidl_generator_traits::value_to_yaml(msg.max_temperature_celsius, out);
    out << "\n";
  }

  // member: failsafe_behavior
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "failsafe_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.failsafe_behavior, out);
    out << "\n";
  }

  // member: position_kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kp, out);
    out << "\n";
  }

  // member: position_ki
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_ki: ";
    rosidl_generator_traits::value_to_yaml(msg.position_ki, out);
    out << "\n";
  }

  // member: position_kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.position_kd, out);
    out << "\n";
  }

  // member: velocity_kp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_kp: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_kp, out);
    out << "\n";
  }

  // member: velocity_ki
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_ki: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_ki, out);
    out << "\n";
  }

  // member: velocity_kd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_kd: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_kd, out);
    out << "\n";
  }

  // member: startup_position_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "startup_position_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_position_rad, out);
    out << "\n";
  }

  // member: startup_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "startup_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.startup_mode, out);
    out << "\n";
  }

  // member: start_disabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start_disabled: ";
    rosidl_generator_traits::value_to_yaml(msg.start_disabled, out);
    out << "\n";
  }

  // member: direction_inverted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction_inverted: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_inverted, out);
    out << "\n";
  }

  // member: position_offset_rad
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position_offset_rad: ";
    rosidl_generator_traits::value_to_yaml(msg.position_offset_rad, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RobomasterMotorConfig & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::msg::RobomasterMotorConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::msg::RobomasterMotorConfig & msg)
{
  return stm32_mavlink_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::msg::RobomasterMotorConfig>()
{
  return "stm32_mavlink_interface::msg::RobomasterMotorConfig";
}

template<>
inline const char * name<stm32_mavlink_interface::msg::RobomasterMotorConfig>()
{
  return "stm32_mavlink_interface/msg/RobomasterMotorConfig";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::msg::RobomasterMotorConfig>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::msg::RobomasterMotorConfig>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<stm32_mavlink_interface::msg::RobomasterMotorConfig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__TRAITS_HPP_

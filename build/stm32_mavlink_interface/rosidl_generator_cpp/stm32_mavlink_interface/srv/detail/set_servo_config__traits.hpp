// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stm32_mavlink_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetServoConfig_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: servo_id
  {
    out << "servo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_id, out);
    out << ", ";
  }

  // member: angle_min_deg
  {
    out << "angle_min_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_min_deg, out);
    out << ", ";
  }

  // member: angle_max_deg
  {
    out << "angle_max_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_max_deg, out);
    out << ", ";
  }

  // member: pulse_min_us
  {
    out << "pulse_min_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_min_us, out);
    out << ", ";
  }

  // member: pulse_max_us
  {
    out << "pulse_max_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_max_us, out);
    out << ", ";
  }

  // member: pulse_neutral_us
  {
    out << "pulse_neutral_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_neutral_us, out);
    out << ", ";
  }

  // member: direction_inverted
  {
    out << "direction_inverted: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_inverted, out);
    out << ", ";
  }

  // member: offset_deg
  {
    out << "offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_deg, out);
    out << ", ";
  }

  // member: max_velocity_deg_per_s
  {
    out << "max_velocity_deg_per_s: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity_deg_per_s, out);
    out << ", ";
  }

  // member: max_acceleration_deg_per_s2
  {
    out << "max_acceleration_deg_per_s2: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration_deg_per_s2, out);
    out << ", ";
  }

  // member: watchdog_timeout_ms
  {
    out << "watchdog_timeout_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.watchdog_timeout_ms, out);
    out << ", ";
  }

  // member: fail_safe_behavior
  {
    out << "fail_safe_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.fail_safe_behavior, out);
    out << ", ";
  }

  // member: save_to_flash
  {
    out << "save_to_flash: ";
    rosidl_generator_traits::value_to_yaml(msg.save_to_flash, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetServoConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: servo_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "servo_id: ";
    rosidl_generator_traits::value_to_yaml(msg.servo_id, out);
    out << "\n";
  }

  // member: angle_min_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_min_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_min_deg, out);
    out << "\n";
  }

  // member: angle_max_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle_max_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.angle_max_deg, out);
    out << "\n";
  }

  // member: pulse_min_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_min_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_min_us, out);
    out << "\n";
  }

  // member: pulse_max_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_max_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_max_us, out);
    out << "\n";
  }

  // member: pulse_neutral_us
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_neutral_us: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_neutral_us, out);
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

  // member: offset_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "offset_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.offset_deg, out);
    out << "\n";
  }

  // member: max_velocity_deg_per_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_velocity_deg_per_s: ";
    rosidl_generator_traits::value_to_yaml(msg.max_velocity_deg_per_s, out);
    out << "\n";
  }

  // member: max_acceleration_deg_per_s2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_acceleration_deg_per_s2: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acceleration_deg_per_s2, out);
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

  // member: fail_safe_behavior
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fail_safe_behavior: ";
    rosidl_generator_traits::value_to_yaml(msg.fail_safe_behavior, out);
    out << "\n";
  }

  // member: save_to_flash
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_to_flash: ";
    rosidl_generator_traits::value_to_yaml(msg.save_to_flash, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetServoConfig_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace stm32_mavlink_interface

namespace rosidl_generator_traits
{

[[deprecated("use stm32_mavlink_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stm32_mavlink_interface::srv::SetServoConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::srv::SetServoConfig_Request & msg)
{
  return stm32_mavlink_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetServoConfig_Request>()
{
  return "stm32_mavlink_interface::srv::SetServoConfig_Request";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetServoConfig_Request>()
{
  return "stm32_mavlink_interface/srv/SetServoConfig_Request";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetServoConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetServoConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stm32_mavlink_interface::srv::SetServoConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace stm32_mavlink_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetServoConfig_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetServoConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetServoConfig_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace stm32_mavlink_interface

namespace rosidl_generator_traits
{

[[deprecated("use stm32_mavlink_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const stm32_mavlink_interface::srv::SetServoConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::srv::SetServoConfig_Response & msg)
{
  return stm32_mavlink_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetServoConfig_Response>()
{
  return "stm32_mavlink_interface::srv::SetServoConfig_Response";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetServoConfig_Response>()
{
  return "stm32_mavlink_interface/srv/SetServoConfig_Response";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetServoConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetServoConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stm32_mavlink_interface::srv::SetServoConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetServoConfig>()
{
  return "stm32_mavlink_interface::srv::SetServoConfig";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetServoConfig>()
{
  return "stm32_mavlink_interface/srv/SetServoConfig";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetServoConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<stm32_mavlink_interface::srv::SetServoConfig_Request>::value &&
    has_fixed_size<stm32_mavlink_interface::srv::SetServoConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetServoConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<stm32_mavlink_interface::srv::SetServoConfig_Request>::value &&
    has_bounded_size<stm32_mavlink_interface::srv::SetServoConfig_Response>::value
  >
{
};

template<>
struct is_service<stm32_mavlink_interface::srv::SetServoConfig>
  : std::true_type
{
};

template<>
struct is_service_request<stm32_mavlink_interface::srv::SetServoConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stm32_mavlink_interface::srv::SetServoConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__TRAITS_HPP_

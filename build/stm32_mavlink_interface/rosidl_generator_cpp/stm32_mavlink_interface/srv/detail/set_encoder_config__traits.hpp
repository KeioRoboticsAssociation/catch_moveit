// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__TRAITS_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace stm32_mavlink_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetEncoderConfig_Request & msg,
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
  const SetEncoderConfig_Request & msg,
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

inline std::string to_yaml(const SetEncoderConfig_Request & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::srv::SetEncoderConfig_Request & msg)
{
  return stm32_mavlink_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetEncoderConfig_Request>()
{
  return "stm32_mavlink_interface::srv::SetEncoderConfig_Request";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetEncoderConfig_Request>()
{
  return "stm32_mavlink_interface/srv/SetEncoderConfig_Request";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetEncoderConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetEncoderConfig_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<stm32_mavlink_interface::srv::SetEncoderConfig_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace stm32_mavlink_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetEncoderConfig_Response & msg,
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
  const SetEncoderConfig_Response & msg,
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

inline std::string to_yaml(const SetEncoderConfig_Response & msg, bool use_flow_style = false)
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
  const stm32_mavlink_interface::srv::SetEncoderConfig_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  stm32_mavlink_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use stm32_mavlink_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const stm32_mavlink_interface::srv::SetEncoderConfig_Response & msg)
{
  return stm32_mavlink_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetEncoderConfig_Response>()
{
  return "stm32_mavlink_interface::srv::SetEncoderConfig_Response";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetEncoderConfig_Response>()
{
  return "stm32_mavlink_interface/srv/SetEncoderConfig_Response";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetEncoderConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetEncoderConfig_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<stm32_mavlink_interface::srv::SetEncoderConfig_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<stm32_mavlink_interface::srv::SetEncoderConfig>()
{
  return "stm32_mavlink_interface::srv::SetEncoderConfig";
}

template<>
inline const char * name<stm32_mavlink_interface::srv::SetEncoderConfig>()
{
  return "stm32_mavlink_interface/srv/SetEncoderConfig";
}

template<>
struct has_fixed_size<stm32_mavlink_interface::srv::SetEncoderConfig>
  : std::integral_constant<
    bool,
    has_fixed_size<stm32_mavlink_interface::srv::SetEncoderConfig_Request>::value &&
    has_fixed_size<stm32_mavlink_interface::srv::SetEncoderConfig_Response>::value
  >
{
};

template<>
struct has_bounded_size<stm32_mavlink_interface::srv::SetEncoderConfig>
  : std::integral_constant<
    bool,
    has_bounded_size<stm32_mavlink_interface::srv::SetEncoderConfig_Request>::value &&
    has_bounded_size<stm32_mavlink_interface::srv::SetEncoderConfig_Response>::value
  >
{
};

template<>
struct is_service<stm32_mavlink_interface::srv::SetEncoderConfig>
  : std::true_type
{
};

template<>
struct is_service_request<stm32_mavlink_interface::srv::SetEncoderConfig_Request>
  : std::true_type
{
};

template<>
struct is_service_response<stm32_mavlink_interface::srv::SetEncoderConfig_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__TRAITS_HPP_

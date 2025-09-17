// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rogilink_flex_interfaces:srv/IsConnected.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__TRAITS_HPP_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rogilink_flex_interfaces/srv/detail/is_connected__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rogilink_flex_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsConnected_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsConnected_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsConnected_Request & msg, bool use_flow_style = false)
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

}  // namespace rogilink_flex_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rogilink_flex_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rogilink_flex_interfaces::srv::IsConnected_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  rogilink_flex_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rogilink_flex_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rogilink_flex_interfaces::srv::IsConnected_Request & msg)
{
  return rogilink_flex_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rogilink_flex_interfaces::srv::IsConnected_Request>()
{
  return "rogilink_flex_interfaces::srv::IsConnected_Request";
}

template<>
inline const char * name<rogilink_flex_interfaces::srv::IsConnected_Request>()
{
  return "rogilink_flex_interfaces/srv/IsConnected_Request";
}

template<>
struct has_fixed_size<rogilink_flex_interfaces::srv::IsConnected_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rogilink_flex_interfaces::srv::IsConnected_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rogilink_flex_interfaces::srv::IsConnected_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rogilink_flex_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsConnected_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: connected
  {
    out << "connected: ";
    rosidl_generator_traits::value_to_yaml(msg.connected, out);
    out << ", ";
  }

  // member: device_id
  {
    out << "device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.device_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsConnected_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: connected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "connected: ";
    rosidl_generator_traits::value_to_yaml(msg.connected, out);
    out << "\n";
  }

  // member: device_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.device_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsConnected_Response & msg, bool use_flow_style = false)
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

}  // namespace rogilink_flex_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rogilink_flex_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rogilink_flex_interfaces::srv::IsConnected_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  rogilink_flex_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rogilink_flex_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const rogilink_flex_interfaces::srv::IsConnected_Response & msg)
{
  return rogilink_flex_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<rogilink_flex_interfaces::srv::IsConnected_Response>()
{
  return "rogilink_flex_interfaces::srv::IsConnected_Response";
}

template<>
inline const char * name<rogilink_flex_interfaces::srv::IsConnected_Response>()
{
  return "rogilink_flex_interfaces/srv/IsConnected_Response";
}

template<>
struct has_fixed_size<rogilink_flex_interfaces::srv::IsConnected_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rogilink_flex_interfaces::srv::IsConnected_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rogilink_flex_interfaces::srv::IsConnected_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rogilink_flex_interfaces::srv::IsConnected>()
{
  return "rogilink_flex_interfaces::srv::IsConnected";
}

template<>
inline const char * name<rogilink_flex_interfaces::srv::IsConnected>()
{
  return "rogilink_flex_interfaces/srv/IsConnected";
}

template<>
struct has_fixed_size<rogilink_flex_interfaces::srv::IsConnected>
  : std::integral_constant<
    bool,
    has_fixed_size<rogilink_flex_interfaces::srv::IsConnected_Request>::value &&
    has_fixed_size<rogilink_flex_interfaces::srv::IsConnected_Response>::value
  >
{
};

template<>
struct has_bounded_size<rogilink_flex_interfaces::srv::IsConnected>
  : std::integral_constant<
    bool,
    has_bounded_size<rogilink_flex_interfaces::srv::IsConnected_Request>::value &&
    has_bounded_size<rogilink_flex_interfaces::srv::IsConnected_Response>::value
  >
{
};

template<>
struct is_service<rogilink_flex_interfaces::srv::IsConnected>
  : std::true_type
{
};

template<>
struct is_service_request<rogilink_flex_interfaces::srv::IsConnected_Request>
  : std::true_type
{
};

template<>
struct is_service_response<rogilink_flex_interfaces::srv::IsConnected_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__TRAITS_HPP_

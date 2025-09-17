// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__TRAITS_HPP_
#define ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rogilink_flex_interfaces/msg/detail/frame__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rogilink_flex_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Frame & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame_id
  {
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << ", ";
  }

  // member: device_id
  {
    out << "device_id: ";
    rosidl_generator_traits::value_to_yaml(msg.device_id, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: topic
  {
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Frame & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
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

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Frame & msg, bool use_flow_style = false)
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

}  // namespace rogilink_flex_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use rogilink_flex_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rogilink_flex_interfaces::msg::Frame & msg,
  std::ostream & out, size_t indentation = 0)
{
  rogilink_flex_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rogilink_flex_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const rogilink_flex_interfaces::msg::Frame & msg)
{
  return rogilink_flex_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rogilink_flex_interfaces::msg::Frame>()
{
  return "rogilink_flex_interfaces::msg::Frame";
}

template<>
inline const char * name<rogilink_flex_interfaces::msg::Frame>()
{
  return "rogilink_flex_interfaces/msg/Frame";
}

template<>
struct has_fixed_size<rogilink_flex_interfaces::msg::Frame>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rogilink_flex_interfaces::msg::Frame>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rogilink_flex_interfaces::msg::Frame>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__TRAITS_HPP_

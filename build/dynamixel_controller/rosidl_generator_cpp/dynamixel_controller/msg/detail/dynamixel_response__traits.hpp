// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__TRAITS_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_controller/msg/detail/dynamixel_response__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dynamixel_controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelResponse & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: ids
  {
    if (msg.ids.size() == 0) {
      out << "ids: []";
    } else {
      out << "ids: [";
      size_t pending_items = msg.ids.size();
      for (auto item : msg.ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: error
  {
    if (msg.error.size() == 0) {
      out << "error: []";
    } else {
      out << "error: [";
      size_t pending_items = msg.error.size();
      for (auto item : msg.error) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamixelResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ids.size() == 0) {
      out << "ids: []\n";
    } else {
      out << "ids:\n";
      for (auto item : msg.ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.error.size() == 0) {
      out << "error: []\n";
    } else {
      out << "error:\n";
      for (auto item : msg.error) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamixelResponse & msg, bool use_flow_style = false)
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

}  // namespace dynamixel_controller

namespace rosidl_generator_traits
{

[[deprecated("use dynamixel_controller::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dynamixel_controller::msg::DynamixelResponse & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_controller::msg::DynamixelResponse & msg)
{
  return dynamixel_controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_controller::msg::DynamixelResponse>()
{
  return "dynamixel_controller::msg::DynamixelResponse";
}

template<>
inline const char * name<dynamixel_controller::msg::DynamixelResponse>()
{
  return "dynamixel_controller/msg/DynamixelResponse";
}

template<>
struct has_fixed_size<dynamixel_controller::msg::DynamixelResponse>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dynamixel_controller::msg::DynamixelResponse>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dynamixel_controller::msg::DynamixelResponse>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__TRAITS_HPP_

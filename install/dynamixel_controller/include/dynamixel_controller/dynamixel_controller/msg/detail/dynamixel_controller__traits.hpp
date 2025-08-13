// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__TRAITS_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dynamixel_controller/msg/detail/dynamixel_controller__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dynamixel_controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const DynamixelController & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DynamixelController & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DynamixelController & msg, bool use_flow_style = false)
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
  const dynamixel_controller::msg::DynamixelController & msg,
  std::ostream & out, size_t indentation = 0)
{
  dynamixel_controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dynamixel_controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const dynamixel_controller::msg::DynamixelController & msg)
{
  return dynamixel_controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dynamixel_controller::msg::DynamixelController>()
{
  return "dynamixel_controller::msg::DynamixelController";
}

template<>
inline const char * name<dynamixel_controller::msg::DynamixelController>()
{
  return "dynamixel_controller/msg/DynamixelController";
}

template<>
struct has_fixed_size<dynamixel_controller::msg::DynamixelController>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dynamixel_controller::msg::DynamixelController>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dynamixel_controller::msg::DynamixelController>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__TRAITS_HPP_

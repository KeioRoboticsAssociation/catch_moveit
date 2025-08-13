// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__BUILDER_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_controller/msg/detail/dynamixel_controller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_controller
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_controller::msg::DynamixelController>()
{
  return ::dynamixel_controller::msg::DynamixelController(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__BUILDER_HPP_

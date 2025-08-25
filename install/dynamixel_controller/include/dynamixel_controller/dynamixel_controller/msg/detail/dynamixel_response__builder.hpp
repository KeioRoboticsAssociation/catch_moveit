// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__BUILDER_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_controller/msg/detail/dynamixel_response__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_controller
{

namespace msg
{

namespace builder
{

class Init_DynamixelResponse_data
{
public:
  explicit Init_DynamixelResponse_data(::dynamixel_controller::msg::DynamixelResponse & msg)
  : msg_(msg)
  {}
  ::dynamixel_controller::msg::DynamixelResponse data(::dynamixel_controller::msg::DynamixelResponse::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelResponse msg_;
};

class Init_DynamixelResponse_error
{
public:
  explicit Init_DynamixelResponse_error(::dynamixel_controller::msg::DynamixelResponse & msg)
  : msg_(msg)
  {}
  Init_DynamixelResponse_data error(::dynamixel_controller::msg::DynamixelResponse::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_DynamixelResponse_data(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelResponse msg_;
};

class Init_DynamixelResponse_ids
{
public:
  explicit Init_DynamixelResponse_ids(::dynamixel_controller::msg::DynamixelResponse & msg)
  : msg_(msg)
  {}
  Init_DynamixelResponse_error ids(::dynamixel_controller::msg::DynamixelResponse::_ids_type arg)
  {
    msg_.ids = std::move(arg);
    return Init_DynamixelResponse_error(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelResponse msg_;
};

class Init_DynamixelResponse_command
{
public:
  Init_DynamixelResponse_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamixelResponse_ids command(::dynamixel_controller::msg::DynamixelResponse::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_DynamixelResponse_ids(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelResponse msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_controller::msg::DynamixelResponse>()
{
  return dynamixel_controller::msg::builder::Init_DynamixelResponse_command();
}

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__BUILDER_HPP_

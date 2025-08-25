// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_controller:msg/DynamixelCommand.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__BUILDER_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_controller/msg/detail/dynamixel_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_controller
{

namespace msg
{

namespace builder
{

class Init_DynamixelCommand_data
{
public:
  explicit Init_DynamixelCommand_data(::dynamixel_controller::msg::DynamixelCommand & msg)
  : msg_(msg)
  {}
  ::dynamixel_controller::msg::DynamixelCommand data(::dynamixel_controller::msg::DynamixelCommand::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelCommand msg_;
};

class Init_DynamixelCommand_length
{
public:
  explicit Init_DynamixelCommand_length(::dynamixel_controller::msg::DynamixelCommand & msg)
  : msg_(msg)
  {}
  Init_DynamixelCommand_data length(::dynamixel_controller::msg::DynamixelCommand::_length_type arg)
  {
    msg_.length = std::move(arg);
    return Init_DynamixelCommand_data(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelCommand msg_;
};

class Init_DynamixelCommand_address
{
public:
  explicit Init_DynamixelCommand_address(::dynamixel_controller::msg::DynamixelCommand & msg)
  : msg_(msg)
  {}
  Init_DynamixelCommand_length address(::dynamixel_controller::msg::DynamixelCommand::_address_type arg)
  {
    msg_.address = std::move(arg);
    return Init_DynamixelCommand_length(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelCommand msg_;
};

class Init_DynamixelCommand_ids
{
public:
  explicit Init_DynamixelCommand_ids(::dynamixel_controller::msg::DynamixelCommand & msg)
  : msg_(msg)
  {}
  Init_DynamixelCommand_address ids(::dynamixel_controller::msg::DynamixelCommand::_ids_type arg)
  {
    msg_.ids = std::move(arg);
    return Init_DynamixelCommand_address(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelCommand msg_;
};

class Init_DynamixelCommand_command
{
public:
  Init_DynamixelCommand_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DynamixelCommand_ids command(::dynamixel_controller::msg::DynamixelCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_DynamixelCommand_ids(msg_);
  }

private:
  ::dynamixel_controller::msg::DynamixelCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_controller::msg::DynamixelCommand>()
{
  return dynamixel_controller::msg::builder::Init_DynamixelCommand_command();
}

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__BUILDER_HPP_

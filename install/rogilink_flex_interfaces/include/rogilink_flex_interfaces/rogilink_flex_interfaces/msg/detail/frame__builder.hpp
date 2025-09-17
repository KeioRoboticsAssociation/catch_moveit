// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__BUILDER_HPP_
#define ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rogilink_flex_interfaces/msg/detail/frame__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rogilink_flex_interfaces
{

namespace msg
{

namespace builder
{

class Init_Frame_topic
{
public:
  explicit Init_Frame_topic(::rogilink_flex_interfaces::msg::Frame & msg)
  : msg_(msg)
  {}
  ::rogilink_flex_interfaces::msg::Frame topic(::rogilink_flex_interfaces::msg::Frame::_topic_type arg)
  {
    msg_.topic = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rogilink_flex_interfaces::msg::Frame msg_;
};

class Init_Frame_data
{
public:
  explicit Init_Frame_data(::rogilink_flex_interfaces::msg::Frame & msg)
  : msg_(msg)
  {}
  Init_Frame_topic data(::rogilink_flex_interfaces::msg::Frame::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_Frame_topic(msg_);
  }

private:
  ::rogilink_flex_interfaces::msg::Frame msg_;
};

class Init_Frame_device_id
{
public:
  explicit Init_Frame_device_id(::rogilink_flex_interfaces::msg::Frame & msg)
  : msg_(msg)
  {}
  Init_Frame_data device_id(::rogilink_flex_interfaces::msg::Frame::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_Frame_data(msg_);
  }

private:
  ::rogilink_flex_interfaces::msg::Frame msg_;
};

class Init_Frame_frame_id
{
public:
  Init_Frame_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Frame_device_id frame_id(::rogilink_flex_interfaces::msg::Frame::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_Frame_device_id(msg_);
  }

private:
  ::rogilink_flex_interfaces::msg::Frame msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rogilink_flex_interfaces::msg::Frame>()
{
  return rogilink_flex_interfaces::msg::builder::Init_Frame_frame_id();
}

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__BUILDER_HPP_

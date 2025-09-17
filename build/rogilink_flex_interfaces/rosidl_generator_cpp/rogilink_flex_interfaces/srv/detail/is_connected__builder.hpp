// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rogilink_flex_interfaces:srv/IsConnected.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__BUILDER_HPP_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rogilink_flex_interfaces/srv/detail/is_connected__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rogilink_flex_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rogilink_flex_interfaces::srv::IsConnected_Request>()
{
  return ::rogilink_flex_interfaces::srv::IsConnected_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rogilink_flex_interfaces


namespace rogilink_flex_interfaces
{

namespace srv
{

namespace builder
{

class Init_IsConnected_Response_device_id
{
public:
  explicit Init_IsConnected_Response_device_id(::rogilink_flex_interfaces::srv::IsConnected_Response & msg)
  : msg_(msg)
  {}
  ::rogilink_flex_interfaces::srv::IsConnected_Response device_id(::rogilink_flex_interfaces::srv::IsConnected_Response::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rogilink_flex_interfaces::srv::IsConnected_Response msg_;
};

class Init_IsConnected_Response_connected
{
public:
  Init_IsConnected_Response_connected()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsConnected_Response_device_id connected(::rogilink_flex_interfaces::srv::IsConnected_Response::_connected_type arg)
  {
    msg_.connected = std::move(arg);
    return Init_IsConnected_Response_device_id(msg_);
  }

private:
  ::rogilink_flex_interfaces::srv::IsConnected_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rogilink_flex_interfaces::srv::IsConnected_Response>()
{
  return rogilink_flex_interfaces::srv::builder::Init_IsConnected_Response_connected();
}

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__BUILDER_HPP_

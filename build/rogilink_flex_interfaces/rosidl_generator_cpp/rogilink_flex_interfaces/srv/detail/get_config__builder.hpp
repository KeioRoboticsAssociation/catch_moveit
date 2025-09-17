// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rogilink_flex_interfaces:srv/GetConfig.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rogilink_flex_interfaces/srv/detail/get_config__struct.hpp"
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
auto build<::rogilink_flex_interfaces::srv::GetConfig_Request>()
{
  return ::rogilink_flex_interfaces::srv::GetConfig_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rogilink_flex_interfaces


namespace rogilink_flex_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetConfig_Response_path
{
public:
  Init_GetConfig_Response_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rogilink_flex_interfaces::srv::GetConfig_Response path(::rogilink_flex_interfaces::srv::GetConfig_Response::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rogilink_flex_interfaces::srv::GetConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::rogilink_flex_interfaces::srv::GetConfig_Response>()
{
  return rogilink_flex_interfaces::srv::builder::Init_GetConfig_Response_path();
}

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__BUILDER_HPP_

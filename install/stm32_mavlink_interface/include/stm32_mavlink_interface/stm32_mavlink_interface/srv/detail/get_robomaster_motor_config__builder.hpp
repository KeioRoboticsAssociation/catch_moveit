// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:srv/GetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/srv/detail/get_robomaster_motor_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_GetRobomasterMotorConfig_Request_motor_id
{
public:
  Init_GetRobomasterMotorConfig_Request_motor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request motor_id(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request>()
{
  return stm32_mavlink_interface::srv::builder::Init_GetRobomasterMotorConfig_Request_motor_id();
}

}  // namespace stm32_mavlink_interface


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_GetRobomasterMotorConfig_Response_message
{
public:
  explicit Init_GetRobomasterMotorConfig_Response_message(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response message(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response msg_;
};

class Init_GetRobomasterMotorConfig_Response_config
{
public:
  explicit Init_GetRobomasterMotorConfig_Response_config(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response & msg)
  : msg_(msg)
  {}
  Init_GetRobomasterMotorConfig_Response_message config(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response::_config_type arg)
  {
    msg_.config = std::move(arg);
    return Init_GetRobomasterMotorConfig_Response_message(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response msg_;
};

class Init_GetRobomasterMotorConfig_Response_success
{
public:
  Init_GetRobomasterMotorConfig_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetRobomasterMotorConfig_Response_config success(::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetRobomasterMotorConfig_Response_config(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response>()
{
  return stm32_mavlink_interface::srv::builder::Init_GetRobomasterMotorConfig_Response_success();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

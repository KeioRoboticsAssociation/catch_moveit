// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from stm32_mavlink_interface:srv/SetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "stm32_mavlink_interface/srv/detail/set_robomaster_motor_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetRobomasterMotorConfig_Request_config
{
public:
  explicit Init_SetRobomasterMotorConfig_Request_config(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request config(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request::_config_type arg)
  {
    msg_.config = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request msg_;
};

class Init_SetRobomasterMotorConfig_Request_motor_id
{
public:
  Init_SetRobomasterMotorConfig_Request_motor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetRobomasterMotorConfig_Request_config motor_id(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_SetRobomasterMotorConfig_Request_config(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Request>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetRobomasterMotorConfig_Request_motor_id();
}

}  // namespace stm32_mavlink_interface


namespace stm32_mavlink_interface
{

namespace srv
{

namespace builder
{

class Init_SetRobomasterMotorConfig_Response_message
{
public:
  explicit Init_SetRobomasterMotorConfig_Response_message(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response & msg)
  : msg_(msg)
  {}
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response message(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response msg_;
};

class Init_SetRobomasterMotorConfig_Response_success
{
public:
  Init_SetRobomasterMotorConfig_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetRobomasterMotorConfig_Response_message success(::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetRobomasterMotorConfig_Response_message(msg_);
  }

private:
  ::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::stm32_mavlink_interface::srv::SetRobomasterMotorConfig_Response>()
{
  return stm32_mavlink_interface::srv::builder::Init_SetRobomasterMotorConfig_Response_success();
}

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__BUILDER_HPP_

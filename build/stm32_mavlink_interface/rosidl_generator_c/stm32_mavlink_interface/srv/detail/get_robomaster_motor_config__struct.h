// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:srv/GetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetRobomasterMotorConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request
{
  /// Request
  /// Motor ID (1-8)
  uint8_t motor_id;
} stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request;

// Struct for a sequence of stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request.
typedef struct stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request__Sequence
{
  stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'config'
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetRobomasterMotorConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response
{
  /// Configuration was retrieved successfully
  bool success;
  /// Motor configuration
  stm32_mavlink_interface__msg__RobomasterMotorConfig config;
  /// Status or error message
  rosidl_runtime_c__String message;
} stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response;

// Struct for a sequence of stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response.
typedef struct stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response__Sequence
{
  stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

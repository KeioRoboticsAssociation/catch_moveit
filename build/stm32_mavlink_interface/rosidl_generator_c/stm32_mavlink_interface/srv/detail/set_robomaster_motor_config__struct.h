// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:srv/SetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'config'
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.h"

/// Struct defined in srv/SetRobomasterMotorConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request
{
  /// Request
  /// Motor ID (1-8)
  uint8_t motor_id;
  /// Motor configuration
  stm32_mavlink_interface__msg__RobomasterMotorConfig config;
} stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request;

// Struct for a sequence of stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request.
typedef struct stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence
{
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetRobomasterMotorConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response
{
  /// Configuration was applied successfully
  bool success;
  /// Status or error message
  rosidl_runtime_c__String message;
} stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response;

// Struct for a sequence of stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response.
typedef struct stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence
{
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

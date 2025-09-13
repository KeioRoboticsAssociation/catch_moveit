// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetServoConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetServoConfig_Request
{
  uint8_t servo_id;
  float angle_min_deg;
  float angle_max_deg;
  uint16_t pulse_min_us;
  uint16_t pulse_max_us;
  uint16_t pulse_neutral_us;
  bool direction_inverted;
  float offset_deg;
  float max_velocity_deg_per_s;
  float max_acceleration_deg_per_s2;
  uint32_t watchdog_timeout_ms;
  /// 0=HOLD, 1=NEUTRAL, 2=DISABLE
  uint8_t fail_safe_behavior;
  bool save_to_flash;
} stm32_mavlink_interface__srv__SetServoConfig_Request;

// Struct for a sequence of stm32_mavlink_interface__srv__SetServoConfig_Request.
typedef struct stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence
{
  stm32_mavlink_interface__srv__SetServoConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetServoConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetServoConfig_Response
{
  bool success;
  rosidl_runtime_c__String message;
} stm32_mavlink_interface__srv__SetServoConfig_Response;

// Struct for a sequence of stm32_mavlink_interface__srv__SetServoConfig_Response.
typedef struct stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence
{
  stm32_mavlink_interface__srv__SetServoConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_H_

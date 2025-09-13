// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetEncoderConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetEncoderConfig_Request
{
  uint8_t encoder_id;
  uint16_t cpr;
  bool invert_a;
  bool invert_b;
  bool use_z;
  uint32_t watchdog_timeout_ms;
  int32_t offset_counts;
  bool wrap_around;
  bool save_to_flash;
} stm32_mavlink_interface__srv__SetEncoderConfig_Request;

// Struct for a sequence of stm32_mavlink_interface__srv__SetEncoderConfig_Request.
typedef struct stm32_mavlink_interface__srv__SetEncoderConfig_Request__Sequence
{
  stm32_mavlink_interface__srv__SetEncoderConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetEncoderConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetEncoderConfig in the package stm32_mavlink_interface.
typedef struct stm32_mavlink_interface__srv__SetEncoderConfig_Response
{
  bool success;
  rosidl_runtime_c__String message;
} stm32_mavlink_interface__srv__SetEncoderConfig_Response;

// Struct for a sequence of stm32_mavlink_interface__srv__SetEncoderConfig_Response.
typedef struct stm32_mavlink_interface__srv__SetEncoderConfig_Response__Sequence
{
  stm32_mavlink_interface__srv__SetEncoderConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__srv__SetEncoderConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_ENCODER_CONFIG__STRUCT_H_

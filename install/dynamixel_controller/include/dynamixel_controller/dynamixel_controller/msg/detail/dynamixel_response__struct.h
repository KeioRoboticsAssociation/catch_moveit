// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_H_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ids'
// Member 'error'
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DynamixelResponse in the package dynamixel_controller.
/**
  * DynamixelResponse.msg
  * Response message from Dynamixel operations
 */
typedef struct dynamixel_controller__msg__DynamixelResponse
{
  /// Original command type
  uint8_t command;
  /// Dynamixel IDs that responded
  rosidl_runtime_c__uint8__Sequence ids;
  /// Error status for each ID
  rosidl_runtime_c__uint8__Sequence error;
  /// Response data payload
  rosidl_runtime_c__uint8__Sequence data;
} dynamixel_controller__msg__DynamixelResponse;

// Struct for a sequence of dynamixel_controller__msg__DynamixelResponse.
typedef struct dynamixel_controller__msg__DynamixelResponse__Sequence
{
  dynamixel_controller__msg__DynamixelResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_controller__msg__DynamixelResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_H_
#define ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'topic'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Frame in the package rogilink_flex_interfaces.
typedef struct rogilink_flex_interfaces__msg__Frame
{
  uint8_t frame_id;
  uint8_t device_id;
  rosidl_runtime_c__uint8__Sequence data;
  rosidl_runtime_c__String topic;
} rogilink_flex_interfaces__msg__Frame;

// Struct for a sequence of rogilink_flex_interfaces__msg__Frame.
typedef struct rogilink_flex_interfaces__msg__Frame__Sequence
{
  rogilink_flex_interfaces__msg__Frame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rogilink_flex_interfaces__msg__Frame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_H_

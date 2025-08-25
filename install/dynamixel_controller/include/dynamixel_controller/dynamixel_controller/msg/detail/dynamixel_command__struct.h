// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_controller:msg/DynamixelCommand.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_H_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_H_

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
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/DynamixelCommand in the package dynamixel_controller.
/**
  * DynamixelCommand.msg
  * Command message for Dynamixel operations
 */
typedef struct dynamixel_controller__msg__DynamixelCommand
{
  /// Command type (PING, READ_DATA, WRITE_DATA, SYNC_READ, SYNC_WRITE, etc.)
  uint8_t command;
  /// Dynamixel IDs to target
  rosidl_runtime_c__uint8__Sequence ids;
  /// Register address
  uint16_t address;
  /// Data length
  uint16_t length;
  /// Data payload for write operations
  rosidl_runtime_c__uint8__Sequence data;
} dynamixel_controller__msg__DynamixelCommand;

// Struct for a sequence of dynamixel_controller__msg__DynamixelCommand.
typedef struct dynamixel_controller__msg__DynamixelCommand__Sequence
{
  dynamixel_controller__msg__DynamixelCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_controller__msg__DynamixelCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_H_

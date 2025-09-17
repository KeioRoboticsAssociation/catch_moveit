// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rogilink_flex_interfaces:srv/GetConfig.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_H_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetConfig in the package rogilink_flex_interfaces.
typedef struct rogilink_flex_interfaces__srv__GetConfig_Request
{
  uint8_t structure_needs_at_least_one_member;
} rogilink_flex_interfaces__srv__GetConfig_Request;

// Struct for a sequence of rogilink_flex_interfaces__srv__GetConfig_Request.
typedef struct rogilink_flex_interfaces__srv__GetConfig_Request__Sequence
{
  rogilink_flex_interfaces__srv__GetConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rogilink_flex_interfaces__srv__GetConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetConfig in the package rogilink_flex_interfaces.
typedef struct rogilink_flex_interfaces__srv__GetConfig_Response
{
  rosidl_runtime_c__String path;
} rogilink_flex_interfaces__srv__GetConfig_Response;

// Struct for a sequence of rogilink_flex_interfaces__srv__GetConfig_Response.
typedef struct rogilink_flex_interfaces__srv__GetConfig_Response__Sequence
{
  rogilink_flex_interfaces__srv__GetConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rogilink_flex_interfaces__srv__GetConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_H_

// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rogilink_flex_interfaces/msg/detail/frame__rosidl_typesupport_introspection_c.h"
#include "rogilink_flex_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rogilink_flex_interfaces/msg/detail/frame__functions.h"
#include "rogilink_flex_interfaces/msg/detail/frame__struct.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `topic`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rogilink_flex_interfaces__msg__Frame__init(message_memory);
}

void rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_fini_function(void * message_memory)
{
  rogilink_flex_interfaces__msg__Frame__fini(message_memory);
}

size_t rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__size_function__Frame__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_const_function__Frame__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_function__Frame__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__fetch_function__Frame__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_const_function__Frame__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__assign_function__Frame__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_function__Frame__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__resize_function__Frame__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_member_array[4] = {
  {
    "frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__msg__Frame, frame_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "device_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__msg__Frame, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__msg__Frame, data),  // bytes offset in struct
    NULL,  // default value
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__size_function__Frame__data,  // size() function pointer
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_const_function__Frame__data,  // get_const(index) function pointer
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__get_function__Frame__data,  // get(index) function pointer
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__fetch_function__Frame__data,  // fetch(index, &value) function pointer
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__assign_function__Frame__data,  // assign(index, value) function pointer
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__resize_function__Frame__data  // resize(index) function pointer
  },
  {
    "topic",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__msg__Frame, topic),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_members = {
  "rogilink_flex_interfaces__msg",  // message namespace
  "Frame",  // message name
  4,  // number of fields
  sizeof(rogilink_flex_interfaces__msg__Frame),
  rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_member_array,  // message members
  rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_init_function,  // function to initialize message memory (memory has to be allocated)
  rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_type_support_handle = {
  0,
  &rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rogilink_flex_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, msg, Frame)() {
  if (!rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_type_support_handle.typesupport_identifier) {
    rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rogilink_flex_interfaces__msg__Frame__rosidl_typesupport_introspection_c__Frame_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

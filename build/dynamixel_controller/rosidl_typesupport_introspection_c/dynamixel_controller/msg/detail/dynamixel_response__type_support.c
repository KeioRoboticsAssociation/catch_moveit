// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_controller/msg/detail/dynamixel_response__rosidl_typesupport_introspection_c.h"
#include "dynamixel_controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_controller/msg/detail/dynamixel_response__functions.h"
#include "dynamixel_controller/msg/detail/dynamixel_response__struct.h"


// Include directives for member types
// Member `ids`
// Member `error`
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_controller__msg__DynamixelResponse__init(message_memory);
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_fini_function(void * message_memory)
{
  dynamixel_controller__msg__DynamixelResponse__fini(message_memory);
}

size_t dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__ids(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__ids(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__error(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__error(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__error(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__error(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__error(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__error(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__error(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__error(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__data(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__data(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__data(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__data(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__data(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__data(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_member_array[4] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_controller__msg__DynamixelResponse, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_controller__msg__DynamixelResponse, ids),  // bytes offset in struct
    NULL,  // default value
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__ids,  // size() function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__ids,  // get_const(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__ids,  // get(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__ids,  // fetch(index, &value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__ids,  // assign(index, value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__ids  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_controller__msg__DynamixelResponse, error),  // bytes offset in struct
    NULL,  // default value
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__error,  // size() function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__error,  // get_const(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__error,  // get(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__error,  // fetch(index, &value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__error,  // assign(index, value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__error  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_controller__msg__DynamixelResponse, data),  // bytes offset in struct
    NULL,  // default value
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__size_function__DynamixelResponse__data,  // size() function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_const_function__DynamixelResponse__data,  // get_const(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__get_function__DynamixelResponse__data,  // get(index) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__fetch_function__DynamixelResponse__data,  // fetch(index, &value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__assign_function__DynamixelResponse__data,  // assign(index, value) function pointer
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__resize_function__DynamixelResponse__data  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_members = {
  "dynamixel_controller__msg",  // message namespace
  "DynamixelResponse",  // message name
  4,  // number of fields
  sizeof(dynamixel_controller__msg__DynamixelResponse),
  dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_member_array,  // message members
  dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_init_function,  // function to initialize message memory (memory has to be allocated)
  dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_type_support_handle = {
  0,
  &dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_controller, msg, DynamixelResponse)() {
  if (!dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_type_support_handle.typesupport_identifier) {
    dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dynamixel_controller__msg__DynamixelResponse__rosidl_typesupport_introspection_c__DynamixelResponse_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

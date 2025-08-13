// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamixel_controller/msg/detail/dynamixel_controller__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dynamixel_controller
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DynamixelController_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamixel_controller::msg::DynamixelController(_init);
}

void DynamixelController_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamixel_controller::msg::DynamixelController *>(message_memory);
  typed_message->~DynamixelController();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DynamixelController_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_controller::msg::DynamixelController, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DynamixelController_message_members = {
  "dynamixel_controller::msg",  // message namespace
  "DynamixelController",  // message name
  1,  // number of fields
  sizeof(dynamixel_controller::msg::DynamixelController),
  DynamixelController_message_member_array,  // message members
  DynamixelController_init_function,  // function to initialize message memory (memory has to be allocated)
  DynamixelController_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DynamixelController_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DynamixelController_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dynamixel_controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamixel_controller::msg::DynamixelController>()
{
  return &::dynamixel_controller::msg::rosidl_typesupport_introspection_cpp::DynamixelController_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamixel_controller, msg, DynamixelController)() {
  return &::dynamixel_controller::msg::rosidl_typesupport_introspection_cpp::DynamixelController_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

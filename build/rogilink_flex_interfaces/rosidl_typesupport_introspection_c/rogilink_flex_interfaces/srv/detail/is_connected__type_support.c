// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rogilink_flex_interfaces:srv/IsConnected.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rogilink_flex_interfaces/srv/detail/is_connected__rosidl_typesupport_introspection_c.h"
#include "rogilink_flex_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rogilink_flex_interfaces/srv/detail/is_connected__functions.h"
#include "rogilink_flex_interfaces/srv/detail/is_connected__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rogilink_flex_interfaces__srv__IsConnected_Request__init(message_memory);
}

void rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_fini_function(void * message_memory)
{
  rogilink_flex_interfaces__srv__IsConnected_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__srv__IsConnected_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_members = {
  "rogilink_flex_interfaces__srv",  // message namespace
  "IsConnected_Request",  // message name
  1,  // number of fields
  sizeof(rogilink_flex_interfaces__srv__IsConnected_Request),
  rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_member_array,  // message members
  rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_type_support_handle = {
  0,
  &rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rogilink_flex_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Request)() {
  if (!rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_type_support_handle.typesupport_identifier) {
    rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rogilink_flex_interfaces__srv__IsConnected_Request__rosidl_typesupport_introspection_c__IsConnected_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "rogilink_flex_interfaces/srv/detail/is_connected__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rogilink_flex_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "rogilink_flex_interfaces/srv/detail/is_connected__functions.h"
// already included above
// #include "rogilink_flex_interfaces/srv/detail/is_connected__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rogilink_flex_interfaces__srv__IsConnected_Response__init(message_memory);
}

void rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_fini_function(void * message_memory)
{
  rogilink_flex_interfaces__srv__IsConnected_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_member_array[2] = {
  {
    "connected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rogilink_flex_interfaces__srv__IsConnected_Response, connected),  // bytes offset in struct
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
    offsetof(rogilink_flex_interfaces__srv__IsConnected_Response, device_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_members = {
  "rogilink_flex_interfaces__srv",  // message namespace
  "IsConnected_Response",  // message name
  2,  // number of fields
  sizeof(rogilink_flex_interfaces__srv__IsConnected_Response),
  rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_member_array,  // message members
  rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_type_support_handle = {
  0,
  &rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rogilink_flex_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Response)() {
  if (!rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_type_support_handle.typesupport_identifier) {
    rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rogilink_flex_interfaces__srv__IsConnected_Response__rosidl_typesupport_introspection_c__IsConnected_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rogilink_flex_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rogilink_flex_interfaces/srv/detail/is_connected__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_members = {
  "rogilink_flex_interfaces__srv",  // service namespace
  "IsConnected",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_Request_message_type_support_handle,
  NULL  // response message
  // rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_Response_message_type_support_handle
};

static rosidl_service_type_support_t rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_type_support_handle = {
  0,
  &rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rogilink_flex_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected)() {
  if (!rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_type_support_handle.typesupport_identifier) {
    rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rogilink_flex_interfaces, srv, IsConnected_Response)()->data;
  }

  return &rogilink_flex_interfaces__srv__detail__is_connected__rosidl_typesupport_introspection_c__IsConnected_service_type_support_handle;
}

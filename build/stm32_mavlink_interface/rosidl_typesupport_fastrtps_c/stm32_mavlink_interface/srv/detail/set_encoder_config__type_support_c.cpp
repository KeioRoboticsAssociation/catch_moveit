// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/srv/detail/set_encoder_config__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "stm32_mavlink_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.h"
#include "stm32_mavlink_interface/srv/detail/set_encoder_config__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SetEncoderConfig_Request__ros_msg_type = stm32_mavlink_interface__srv__SetEncoderConfig_Request;

static bool _SetEncoderConfig_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetEncoderConfig_Request__ros_msg_type * ros_message = static_cast<const _SetEncoderConfig_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: encoder_id
  {
    cdr << ros_message->encoder_id;
  }

  // Field name: cpr
  {
    cdr << ros_message->cpr;
  }

  // Field name: invert_a
  {
    cdr << (ros_message->invert_a ? true : false);
  }

  // Field name: invert_b
  {
    cdr << (ros_message->invert_b ? true : false);
  }

  // Field name: use_z
  {
    cdr << (ros_message->use_z ? true : false);
  }

  // Field name: watchdog_timeout_ms
  {
    cdr << ros_message->watchdog_timeout_ms;
  }

  // Field name: offset_counts
  {
    cdr << ros_message->offset_counts;
  }

  // Field name: wrap_around
  {
    cdr << (ros_message->wrap_around ? true : false);
  }

  // Field name: save_to_flash
  {
    cdr << (ros_message->save_to_flash ? true : false);
  }

  return true;
}

static bool _SetEncoderConfig_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetEncoderConfig_Request__ros_msg_type * ros_message = static_cast<_SetEncoderConfig_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: encoder_id
  {
    cdr >> ros_message->encoder_id;
  }

  // Field name: cpr
  {
    cdr >> ros_message->cpr;
  }

  // Field name: invert_a
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invert_a = tmp ? true : false;
  }

  // Field name: invert_b
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->invert_b = tmp ? true : false;
  }

  // Field name: use_z
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->use_z = tmp ? true : false;
  }

  // Field name: watchdog_timeout_ms
  {
    cdr >> ros_message->watchdog_timeout_ms;
  }

  // Field name: offset_counts
  {
    cdr >> ros_message->offset_counts;
  }

  // Field name: wrap_around
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->wrap_around = tmp ? true : false;
  }

  // Field name: save_to_flash
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->save_to_flash = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t get_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetEncoderConfig_Request__ros_msg_type * ros_message = static_cast<const _SetEncoderConfig_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name encoder_id
  {
    size_t item_size = sizeof(ros_message->encoder_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cpr
  {
    size_t item_size = sizeof(ros_message->cpr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invert_a
  {
    size_t item_size = sizeof(ros_message->invert_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name invert_b
  {
    size_t item_size = sizeof(ros_message->invert_b);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name use_z
  {
    size_t item_size = sizeof(ros_message->use_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name watchdog_timeout_ms
  {
    size_t item_size = sizeof(ros_message->watchdog_timeout_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name offset_counts
  {
    size_t item_size = sizeof(ros_message->offset_counts);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name wrap_around
  {
    size_t item_size = sizeof(ros_message->wrap_around);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name save_to_flash
  {
    size_t item_size = sizeof(ros_message->save_to_flash);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SetEncoderConfig_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t max_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: encoder_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cpr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: invert_a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: invert_b
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: use_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: watchdog_timeout_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: offset_counts
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: wrap_around
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: save_to_flash
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = stm32_mavlink_interface__srv__SetEncoderConfig_Request;
    is_plain =
      (
      offsetof(DataType, save_to_flash) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetEncoderConfig_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetEncoderConfig_Request = {
  "stm32_mavlink_interface::srv",
  "SetEncoderConfig_Request",
  _SetEncoderConfig_Request__cdr_serialize,
  _SetEncoderConfig_Request__cdr_deserialize,
  _SetEncoderConfig_Request__get_serialized_size,
  _SetEncoderConfig_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetEncoderConfig_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetEncoderConfig_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetEncoderConfig_Request)() {
  return &_SetEncoderConfig_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "stm32_mavlink_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_encoder_config__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // message
#include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions


using _SetEncoderConfig_Response__ros_msg_type = stm32_mavlink_interface__srv__SetEncoderConfig_Response;

static bool _SetEncoderConfig_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetEncoderConfig_Response__ros_msg_type * ros_message = static_cast<const _SetEncoderConfig_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SetEncoderConfig_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetEncoderConfig_Response__ros_msg_type * ros_message = static_cast<_SetEncoderConfig_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t get_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetEncoderConfig_Response__ros_msg_type * ros_message = static_cast<const _SetEncoderConfig_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SetEncoderConfig_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t max_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = stm32_mavlink_interface__srv__SetEncoderConfig_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetEncoderConfig_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_stm32_mavlink_interface__srv__SetEncoderConfig_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetEncoderConfig_Response = {
  "stm32_mavlink_interface::srv",
  "SetEncoderConfig_Response",
  _SetEncoderConfig_Response__cdr_serialize,
  _SetEncoderConfig_Response__cdr_deserialize,
  _SetEncoderConfig_Response__get_serialized_size,
  _SetEncoderConfig_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetEncoderConfig_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetEncoderConfig_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetEncoderConfig_Response)() {
  return &_SetEncoderConfig_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "stm32_mavlink_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "stm32_mavlink_interface/srv/set_encoder_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetEncoderConfig__callbacks = {
  "stm32_mavlink_interface::srv",
  "SetEncoderConfig",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetEncoderConfig_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetEncoderConfig_Response)(),
};

static rosidl_service_type_support_t SetEncoderConfig__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetEncoderConfig__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetEncoderConfig)() {
  return &SetEncoderConfig__handle;
}

#if defined(__cplusplus)
}
#endif

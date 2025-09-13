// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/srv/detail/set_servo_config__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "stm32_mavlink_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.h"
#include "stm32_mavlink_interface/srv/detail/set_servo_config__functions.h"
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


using _SetServoConfig_Request__ros_msg_type = stm32_mavlink_interface__srv__SetServoConfig_Request;

static bool _SetServoConfig_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetServoConfig_Request__ros_msg_type * ros_message = static_cast<const _SetServoConfig_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: servo_id
  {
    cdr << ros_message->servo_id;
  }

  // Field name: angle_min_deg
  {
    cdr << ros_message->angle_min_deg;
  }

  // Field name: angle_max_deg
  {
    cdr << ros_message->angle_max_deg;
  }

  // Field name: pulse_min_us
  {
    cdr << ros_message->pulse_min_us;
  }

  // Field name: pulse_max_us
  {
    cdr << ros_message->pulse_max_us;
  }

  // Field name: pulse_neutral_us
  {
    cdr << ros_message->pulse_neutral_us;
  }

  // Field name: direction_inverted
  {
    cdr << (ros_message->direction_inverted ? true : false);
  }

  // Field name: offset_deg
  {
    cdr << ros_message->offset_deg;
  }

  // Field name: max_velocity_deg_per_s
  {
    cdr << ros_message->max_velocity_deg_per_s;
  }

  // Field name: max_acceleration_deg_per_s2
  {
    cdr << ros_message->max_acceleration_deg_per_s2;
  }

  // Field name: watchdog_timeout_ms
  {
    cdr << ros_message->watchdog_timeout_ms;
  }

  // Field name: fail_safe_behavior
  {
    cdr << ros_message->fail_safe_behavior;
  }

  // Field name: save_to_flash
  {
    cdr << (ros_message->save_to_flash ? true : false);
  }

  return true;
}

static bool _SetServoConfig_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetServoConfig_Request__ros_msg_type * ros_message = static_cast<_SetServoConfig_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: servo_id
  {
    cdr >> ros_message->servo_id;
  }

  // Field name: angle_min_deg
  {
    cdr >> ros_message->angle_min_deg;
  }

  // Field name: angle_max_deg
  {
    cdr >> ros_message->angle_max_deg;
  }

  // Field name: pulse_min_us
  {
    cdr >> ros_message->pulse_min_us;
  }

  // Field name: pulse_max_us
  {
    cdr >> ros_message->pulse_max_us;
  }

  // Field name: pulse_neutral_us
  {
    cdr >> ros_message->pulse_neutral_us;
  }

  // Field name: direction_inverted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->direction_inverted = tmp ? true : false;
  }

  // Field name: offset_deg
  {
    cdr >> ros_message->offset_deg;
  }

  // Field name: max_velocity_deg_per_s
  {
    cdr >> ros_message->max_velocity_deg_per_s;
  }

  // Field name: max_acceleration_deg_per_s2
  {
    cdr >> ros_message->max_acceleration_deg_per_s2;
  }

  // Field name: watchdog_timeout_ms
  {
    cdr >> ros_message->watchdog_timeout_ms;
  }

  // Field name: fail_safe_behavior
  {
    cdr >> ros_message->fail_safe_behavior;
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
size_t get_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetServoConfig_Request__ros_msg_type * ros_message = static_cast<const _SetServoConfig_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name servo_id
  {
    size_t item_size = sizeof(ros_message->servo_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angle_min_deg
  {
    size_t item_size = sizeof(ros_message->angle_min_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angle_max_deg
  {
    size_t item_size = sizeof(ros_message->angle_max_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pulse_min_us
  {
    size_t item_size = sizeof(ros_message->pulse_min_us);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pulse_max_us
  {
    size_t item_size = sizeof(ros_message->pulse_max_us);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pulse_neutral_us
  {
    size_t item_size = sizeof(ros_message->pulse_neutral_us);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name direction_inverted
  {
    size_t item_size = sizeof(ros_message->direction_inverted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name offset_deg
  {
    size_t item_size = sizeof(ros_message->offset_deg);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_velocity_deg_per_s
  {
    size_t item_size = sizeof(ros_message->max_velocity_deg_per_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_acceleration_deg_per_s2
  {
    size_t item_size = sizeof(ros_message->max_acceleration_deg_per_s2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name watchdog_timeout_ms
  {
    size_t item_size = sizeof(ros_message->watchdog_timeout_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name fail_safe_behavior
  {
    size_t item_size = sizeof(ros_message->fail_safe_behavior);
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

static uint32_t _SetServoConfig_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t max_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Request(
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

  // member: servo_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: angle_min_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: angle_max_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pulse_min_us
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: pulse_max_us
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: pulse_neutral_us
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: direction_inverted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: offset_deg
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: max_velocity_deg_per_s
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: max_acceleration_deg_per_s2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: watchdog_timeout_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: fail_safe_behavior
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
    using DataType = stm32_mavlink_interface__srv__SetServoConfig_Request;
    is_plain =
      (
      offsetof(DataType, save_to_flash) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetServoConfig_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetServoConfig_Request = {
  "stm32_mavlink_interface::srv",
  "SetServoConfig_Request",
  _SetServoConfig_Request__cdr_serialize,
  _SetServoConfig_Request__cdr_deserialize,
  _SetServoConfig_Request__get_serialized_size,
  _SetServoConfig_Request__max_serialized_size
};

static rosidl_message_type_support_t _SetServoConfig_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetServoConfig_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetServoConfig_Request)() {
  return &_SetServoConfig_Request__type_support;
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
// #include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_servo_config__functions.h"
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


using _SetServoConfig_Response__ros_msg_type = stm32_mavlink_interface__srv__SetServoConfig_Response;

static bool _SetServoConfig_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SetServoConfig_Response__ros_msg_type * ros_message = static_cast<const _SetServoConfig_Response__ros_msg_type *>(untyped_ros_message);
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

static bool _SetServoConfig_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SetServoConfig_Response__ros_msg_type * ros_message = static_cast<_SetServoConfig_Response__ros_msg_type *>(untyped_ros_message);
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
size_t get_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SetServoConfig_Response__ros_msg_type * ros_message = static_cast<const _SetServoConfig_Response__ros_msg_type *>(untyped_ros_message);
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

static uint32_t _SetServoConfig_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t max_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Response(
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
    using DataType = stm32_mavlink_interface__srv__SetServoConfig_Response;
    is_plain =
      (
      offsetof(DataType, message) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SetServoConfig_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_stm32_mavlink_interface__srv__SetServoConfig_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SetServoConfig_Response = {
  "stm32_mavlink_interface::srv",
  "SetServoConfig_Response",
  _SetServoConfig_Response__cdr_serialize,
  _SetServoConfig_Response__cdr_deserialize,
  _SetServoConfig_Response__get_serialized_size,
  _SetServoConfig_Response__max_serialized_size
};

static rosidl_message_type_support_t _SetServoConfig_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SetServoConfig_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetServoConfig_Response)() {
  return &_SetServoConfig_Response__type_support;
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
#include "stm32_mavlink_interface/srv/set_servo_config.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SetServoConfig__callbacks = {
  "stm32_mavlink_interface::srv",
  "SetServoConfig",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetServoConfig_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetServoConfig_Response)(),
};

static rosidl_service_type_support_t SetServoConfig__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SetServoConfig__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, srv, SetServoConfig)() {
  return &SetServoConfig__handle;
}

#if defined(__cplusplus)
}
#endif

// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "stm32_mavlink_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.h"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__functions.h"
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

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_stm32_mavlink_interface
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_stm32_mavlink_interface
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_stm32_mavlink_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _RobomasterMotorConfig__ros_msg_type = stm32_mavlink_interface__msg__RobomasterMotorConfig;

static bool _RobomasterMotorConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RobomasterMotorConfig__ros_msg_type * ros_message = static_cast<const _RobomasterMotorConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: motor_id
  {
    cdr << ros_message->motor_id;
  }

  // Field name: max_velocity_rps
  {
    cdr << ros_message->max_velocity_rps;
  }

  // Field name: max_acceleration_rps2
  {
    cdr << ros_message->max_acceleration_rps2;
  }

  // Field name: max_current_ma
  {
    cdr << ros_message->max_current_ma;
  }

  // Field name: min_current_ma
  {
    cdr << ros_message->min_current_ma;
  }

  // Field name: min_position_rad
  {
    cdr << ros_message->min_position_rad;
  }

  // Field name: max_position_rad
  {
    cdr << ros_message->max_position_rad;
  }

  // Field name: position_limits_enabled
  {
    cdr << (ros_message->position_limits_enabled ? true : false);
  }

  // Field name: watchdog_timeout_ms
  {
    cdr << ros_message->watchdog_timeout_ms;
  }

  // Field name: max_temperature_celsius
  {
    cdr << ros_message->max_temperature_celsius;
  }

  // Field name: failsafe_behavior
  {
    cdr << ros_message->failsafe_behavior;
  }

  // Field name: position_kp
  {
    cdr << ros_message->position_kp;
  }

  // Field name: position_ki
  {
    cdr << ros_message->position_ki;
  }

  // Field name: position_kd
  {
    cdr << ros_message->position_kd;
  }

  // Field name: velocity_kp
  {
    cdr << ros_message->velocity_kp;
  }

  // Field name: velocity_ki
  {
    cdr << ros_message->velocity_ki;
  }

  // Field name: velocity_kd
  {
    cdr << ros_message->velocity_kd;
  }

  // Field name: startup_position_rad
  {
    cdr << ros_message->startup_position_rad;
  }

  // Field name: startup_mode
  {
    cdr << ros_message->startup_mode;
  }

  // Field name: start_disabled
  {
    cdr << (ros_message->start_disabled ? true : false);
  }

  // Field name: direction_inverted
  {
    cdr << (ros_message->direction_inverted ? true : false);
  }

  // Field name: position_offset_rad
  {
    cdr << ros_message->position_offset_rad;
  }

  return true;
}

static bool _RobomasterMotorConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RobomasterMotorConfig__ros_msg_type * ros_message = static_cast<_RobomasterMotorConfig__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: motor_id
  {
    cdr >> ros_message->motor_id;
  }

  // Field name: max_velocity_rps
  {
    cdr >> ros_message->max_velocity_rps;
  }

  // Field name: max_acceleration_rps2
  {
    cdr >> ros_message->max_acceleration_rps2;
  }

  // Field name: max_current_ma
  {
    cdr >> ros_message->max_current_ma;
  }

  // Field name: min_current_ma
  {
    cdr >> ros_message->min_current_ma;
  }

  // Field name: min_position_rad
  {
    cdr >> ros_message->min_position_rad;
  }

  // Field name: max_position_rad
  {
    cdr >> ros_message->max_position_rad;
  }

  // Field name: position_limits_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->position_limits_enabled = tmp ? true : false;
  }

  // Field name: watchdog_timeout_ms
  {
    cdr >> ros_message->watchdog_timeout_ms;
  }

  // Field name: max_temperature_celsius
  {
    cdr >> ros_message->max_temperature_celsius;
  }

  // Field name: failsafe_behavior
  {
    cdr >> ros_message->failsafe_behavior;
  }

  // Field name: position_kp
  {
    cdr >> ros_message->position_kp;
  }

  // Field name: position_ki
  {
    cdr >> ros_message->position_ki;
  }

  // Field name: position_kd
  {
    cdr >> ros_message->position_kd;
  }

  // Field name: velocity_kp
  {
    cdr >> ros_message->velocity_kp;
  }

  // Field name: velocity_ki
  {
    cdr >> ros_message->velocity_ki;
  }

  // Field name: velocity_kd
  {
    cdr >> ros_message->velocity_kd;
  }

  // Field name: startup_position_rad
  {
    cdr >> ros_message->startup_position_rad;
  }

  // Field name: startup_mode
  {
    cdr >> ros_message->startup_mode;
  }

  // Field name: start_disabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->start_disabled = tmp ? true : false;
  }

  // Field name: direction_inverted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->direction_inverted = tmp ? true : false;
  }

  // Field name: position_offset_rad
  {
    cdr >> ros_message->position_offset_rad;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t get_serialized_size_stm32_mavlink_interface__msg__RobomasterMotorConfig(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RobomasterMotorConfig__ros_msg_type * ros_message = static_cast<const _RobomasterMotorConfig__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name motor_id
  {
    size_t item_size = sizeof(ros_message->motor_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_velocity_rps
  {
    size_t item_size = sizeof(ros_message->max_velocity_rps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_acceleration_rps2
  {
    size_t item_size = sizeof(ros_message->max_acceleration_rps2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_current_ma
  {
    size_t item_size = sizeof(ros_message->max_current_ma);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min_current_ma
  {
    size_t item_size = sizeof(ros_message->min_current_ma);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min_position_rad
  {
    size_t item_size = sizeof(ros_message->min_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_position_rad
  {
    size_t item_size = sizeof(ros_message->max_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_limits_enabled
  {
    size_t item_size = sizeof(ros_message->position_limits_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name watchdog_timeout_ms
  {
    size_t item_size = sizeof(ros_message->watchdog_timeout_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_temperature_celsius
  {
    size_t item_size = sizeof(ros_message->max_temperature_celsius);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name failsafe_behavior
  {
    size_t item_size = sizeof(ros_message->failsafe_behavior);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_kp
  {
    size_t item_size = sizeof(ros_message->position_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_ki
  {
    size_t item_size = sizeof(ros_message->position_ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_kd
  {
    size_t item_size = sizeof(ros_message->position_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_kp
  {
    size_t item_size = sizeof(ros_message->velocity_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_ki
  {
    size_t item_size = sizeof(ros_message->velocity_ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velocity_kd
  {
    size_t item_size = sizeof(ros_message->velocity_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name startup_position_rad
  {
    size_t item_size = sizeof(ros_message->startup_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name startup_mode
  {
    size_t item_size = sizeof(ros_message->startup_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name start_disabled
  {
    size_t item_size = sizeof(ros_message->start_disabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name direction_inverted
  {
    size_t item_size = sizeof(ros_message->direction_inverted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_offset_rad
  {
    size_t item_size = sizeof(ros_message->position_offset_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RobomasterMotorConfig__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_stm32_mavlink_interface__msg__RobomasterMotorConfig(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_stm32_mavlink_interface
size_t max_serialized_size_stm32_mavlink_interface__msg__RobomasterMotorConfig(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: motor_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: max_velocity_rps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: max_acceleration_rps2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: max_current_ma
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: min_current_ma
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: min_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: max_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position_limits_enabled
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
  // member: max_temperature_celsius
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: failsafe_behavior
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: position_kp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position_ki
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position_kd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity_kp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity_ki
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velocity_kd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: startup_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: startup_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: start_disabled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: direction_inverted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: position_offset_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = stm32_mavlink_interface__msg__RobomasterMotorConfig;
    is_plain =
      (
      offsetof(DataType, position_offset_rad) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RobomasterMotorConfig__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_stm32_mavlink_interface__msg__RobomasterMotorConfig(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RobomasterMotorConfig = {
  "stm32_mavlink_interface::msg",
  "RobomasterMotorConfig",
  _RobomasterMotorConfig__cdr_serialize,
  _RobomasterMotorConfig__cdr_deserialize,
  _RobomasterMotorConfig__get_serialized_size,
  _RobomasterMotorConfig__max_serialized_size
};

static rosidl_message_type_support_t _RobomasterMotorConfig__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RobomasterMotorConfig,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, stm32_mavlink_interface, msg, RobomasterMotorConfig)() {
  return &_RobomasterMotorConfig__type_support;
}

#if defined(__cplusplus)
}
#endif

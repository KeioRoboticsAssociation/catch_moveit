// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__rosidl_typesupport_fastrtps_cpp.hpp"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace stm32_mavlink_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
cdr_serialize(
  const stm32_mavlink_interface::msg::RobomasterMotorConfig & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: motor_id
  cdr << ros_message.motor_id;
  // Member: max_velocity_rps
  cdr << ros_message.max_velocity_rps;
  // Member: max_acceleration_rps2
  cdr << ros_message.max_acceleration_rps2;
  // Member: max_current_ma
  cdr << ros_message.max_current_ma;
  // Member: min_current_ma
  cdr << ros_message.min_current_ma;
  // Member: min_position_rad
  cdr << ros_message.min_position_rad;
  // Member: max_position_rad
  cdr << ros_message.max_position_rad;
  // Member: position_limits_enabled
  cdr << (ros_message.position_limits_enabled ? true : false);
  // Member: watchdog_timeout_ms
  cdr << ros_message.watchdog_timeout_ms;
  // Member: max_temperature_celsius
  cdr << ros_message.max_temperature_celsius;
  // Member: failsafe_behavior
  cdr << ros_message.failsafe_behavior;
  // Member: position_kp
  cdr << ros_message.position_kp;
  // Member: position_ki
  cdr << ros_message.position_ki;
  // Member: position_kd
  cdr << ros_message.position_kd;
  // Member: velocity_kp
  cdr << ros_message.velocity_kp;
  // Member: velocity_ki
  cdr << ros_message.velocity_ki;
  // Member: velocity_kd
  cdr << ros_message.velocity_kd;
  // Member: startup_position_rad
  cdr << ros_message.startup_position_rad;
  // Member: startup_mode
  cdr << ros_message.startup_mode;
  // Member: start_disabled
  cdr << (ros_message.start_disabled ? true : false);
  // Member: direction_inverted
  cdr << (ros_message.direction_inverted ? true : false);
  // Member: position_offset_rad
  cdr << ros_message.position_offset_rad;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  stm32_mavlink_interface::msg::RobomasterMotorConfig & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: motor_id
  cdr >> ros_message.motor_id;

  // Member: max_velocity_rps
  cdr >> ros_message.max_velocity_rps;

  // Member: max_acceleration_rps2
  cdr >> ros_message.max_acceleration_rps2;

  // Member: max_current_ma
  cdr >> ros_message.max_current_ma;

  // Member: min_current_ma
  cdr >> ros_message.min_current_ma;

  // Member: min_position_rad
  cdr >> ros_message.min_position_rad;

  // Member: max_position_rad
  cdr >> ros_message.max_position_rad;

  // Member: position_limits_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.position_limits_enabled = tmp ? true : false;
  }

  // Member: watchdog_timeout_ms
  cdr >> ros_message.watchdog_timeout_ms;

  // Member: max_temperature_celsius
  cdr >> ros_message.max_temperature_celsius;

  // Member: failsafe_behavior
  cdr >> ros_message.failsafe_behavior;

  // Member: position_kp
  cdr >> ros_message.position_kp;

  // Member: position_ki
  cdr >> ros_message.position_ki;

  // Member: position_kd
  cdr >> ros_message.position_kd;

  // Member: velocity_kp
  cdr >> ros_message.velocity_kp;

  // Member: velocity_ki
  cdr >> ros_message.velocity_ki;

  // Member: velocity_kd
  cdr >> ros_message.velocity_kd;

  // Member: startup_position_rad
  cdr >> ros_message.startup_position_rad;

  // Member: startup_mode
  cdr >> ros_message.startup_mode;

  // Member: start_disabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.start_disabled = tmp ? true : false;
  }

  // Member: direction_inverted
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.direction_inverted = tmp ? true : false;
  }

  // Member: position_offset_rad
  cdr >> ros_message.position_offset_rad;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
get_serialized_size(
  const stm32_mavlink_interface::msg::RobomasterMotorConfig & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: motor_id
  {
    size_t item_size = sizeof(ros_message.motor_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_velocity_rps
  {
    size_t item_size = sizeof(ros_message.max_velocity_rps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_acceleration_rps2
  {
    size_t item_size = sizeof(ros_message.max_acceleration_rps2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_current_ma
  {
    size_t item_size = sizeof(ros_message.max_current_ma);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min_current_ma
  {
    size_t item_size = sizeof(ros_message.min_current_ma);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min_position_rad
  {
    size_t item_size = sizeof(ros_message.min_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_position_rad
  {
    size_t item_size = sizeof(ros_message.max_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_limits_enabled
  {
    size_t item_size = sizeof(ros_message.position_limits_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: watchdog_timeout_ms
  {
    size_t item_size = sizeof(ros_message.watchdog_timeout_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_temperature_celsius
  {
    size_t item_size = sizeof(ros_message.max_temperature_celsius);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: failsafe_behavior
  {
    size_t item_size = sizeof(ros_message.failsafe_behavior);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_kp
  {
    size_t item_size = sizeof(ros_message.position_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_ki
  {
    size_t item_size = sizeof(ros_message.position_ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_kd
  {
    size_t item_size = sizeof(ros_message.position_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity_kp
  {
    size_t item_size = sizeof(ros_message.velocity_kp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity_ki
  {
    size_t item_size = sizeof(ros_message.velocity_ki);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: velocity_kd
  {
    size_t item_size = sizeof(ros_message.velocity_kd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: startup_position_rad
  {
    size_t item_size = sizeof(ros_message.startup_position_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: startup_mode
  {
    size_t item_size = sizeof(ros_message.startup_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: start_disabled
  {
    size_t item_size = sizeof(ros_message.start_disabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: direction_inverted
  {
    size_t item_size = sizeof(ros_message.direction_inverted);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_offset_rad
  {
    size_t item_size = sizeof(ros_message.position_offset_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
max_serialized_size_RobomasterMotorConfig(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: motor_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: max_velocity_rps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_acceleration_rps2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_current_ma
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: min_current_ma
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: min_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position_limits_enabled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: watchdog_timeout_ms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_temperature_celsius
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: failsafe_behavior
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: position_kp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position_ki
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position_kd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity_kp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity_ki
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: velocity_kd
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: startup_position_rad
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: startup_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: start_disabled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: direction_inverted
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: position_offset_rad
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
    using DataType = stm32_mavlink_interface::msg::RobomasterMotorConfig;
    is_plain =
      (
      offsetof(DataType, position_offset_rad) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RobomasterMotorConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const stm32_mavlink_interface::msg::RobomasterMotorConfig *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RobomasterMotorConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<stm32_mavlink_interface::msg::RobomasterMotorConfig *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RobomasterMotorConfig__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const stm32_mavlink_interface::msg::RobomasterMotorConfig *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RobomasterMotorConfig__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RobomasterMotorConfig(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RobomasterMotorConfig__callbacks = {
  "stm32_mavlink_interface::msg",
  "RobomasterMotorConfig",
  _RobomasterMotorConfig__cdr_serialize,
  _RobomasterMotorConfig__cdr_deserialize,
  _RobomasterMotorConfig__get_serialized_size,
  _RobomasterMotorConfig__max_serialized_size
};

static rosidl_message_type_support_t _RobomasterMotorConfig__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RobomasterMotorConfig__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace stm32_mavlink_interface

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_stm32_mavlink_interface
const rosidl_message_type_support_t *
get_message_type_support_handle<stm32_mavlink_interface::msg::RobomasterMotorConfig>()
{
  return &stm32_mavlink_interface::msg::typesupport_fastrtps_cpp::_RobomasterMotorConfig__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, stm32_mavlink_interface, msg, RobomasterMotorConfig)() {
  return &stm32_mavlink_interface::msg::typesupport_fastrtps_cpp::_RobomasterMotorConfig__handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/encoder_config__rosidl_typesupport_fastrtps_cpp.hpp"
#include "stm32_mavlink_interface/msg/detail/encoder_config__struct.hpp"

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

namespace stm32_mavlink_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
cdr_serialize(
  const stm32_mavlink_interface::msg::EncoderConfig & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: encoder_id
  cdr << ros_message.encoder_id;
  // Member: cpr
  cdr << ros_message.cpr;
  // Member: invert_a
  cdr << (ros_message.invert_a ? true : false);
  // Member: invert_b
  cdr << (ros_message.invert_b ? true : false);
  // Member: use_z
  cdr << (ros_message.use_z ? true : false);
  // Member: watchdog_timeout_ms
  cdr << ros_message.watchdog_timeout_ms;
  // Member: offset_counts
  cdr << ros_message.offset_counts;
  // Member: wrap_around
  cdr << (ros_message.wrap_around ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  stm32_mavlink_interface::msg::EncoderConfig & ros_message)
{
  // Member: encoder_id
  cdr >> ros_message.encoder_id;

  // Member: cpr
  cdr >> ros_message.cpr;

  // Member: invert_a
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invert_a = tmp ? true : false;
  }

  // Member: invert_b
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.invert_b = tmp ? true : false;
  }

  // Member: use_z
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.use_z = tmp ? true : false;
  }

  // Member: watchdog_timeout_ms
  cdr >> ros_message.watchdog_timeout_ms;

  // Member: offset_counts
  cdr >> ros_message.offset_counts;

  // Member: wrap_around
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.wrap_around = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
get_serialized_size(
  const stm32_mavlink_interface::msg::EncoderConfig & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: encoder_id
  {
    size_t item_size = sizeof(ros_message.encoder_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cpr
  {
    size_t item_size = sizeof(ros_message.cpr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invert_a
  {
    size_t item_size = sizeof(ros_message.invert_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: invert_b
  {
    size_t item_size = sizeof(ros_message.invert_b);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: use_z
  {
    size_t item_size = sizeof(ros_message.use_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: watchdog_timeout_ms
  {
    size_t item_size = sizeof(ros_message.watchdog_timeout_ms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: offset_counts
  {
    size_t item_size = sizeof(ros_message.offset_counts);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wrap_around
  {
    size_t item_size = sizeof(ros_message.wrap_around);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_stm32_mavlink_interface
max_serialized_size_EncoderConfig(
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


  // Member: encoder_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cpr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: invert_a
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: invert_b
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: use_z
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

  // Member: offset_counts
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wrap_around
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
    using DataType = stm32_mavlink_interface::msg::EncoderConfig;
    is_plain =
      (
      offsetof(DataType, wrap_around) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EncoderConfig__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const stm32_mavlink_interface::msg::EncoderConfig *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EncoderConfig__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<stm32_mavlink_interface::msg::EncoderConfig *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EncoderConfig__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const stm32_mavlink_interface::msg::EncoderConfig *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EncoderConfig__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EncoderConfig(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EncoderConfig__callbacks = {
  "stm32_mavlink_interface::msg",
  "EncoderConfig",
  _EncoderConfig__cdr_serialize,
  _EncoderConfig__cdr_deserialize,
  _EncoderConfig__get_serialized_size,
  _EncoderConfig__max_serialized_size
};

static rosidl_message_type_support_t _EncoderConfig__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EncoderConfig__callbacks,
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
get_message_type_support_handle<stm32_mavlink_interface::msg::EncoderConfig>()
{
  return &stm32_mavlink_interface::msg::typesupport_fastrtps_cpp::_EncoderConfig__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, stm32_mavlink_interface, msg, EncoderConfig)() {
  return &stm32_mavlink_interface::msg::typesupport_fastrtps_cpp::_EncoderConfig__handle;
}

#ifdef __cplusplus
}
#endif

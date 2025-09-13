// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/EncoderState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/EncoderState in the package stm32_mavlink_interface.
/**
  * Encoder state message
 */
typedef struct stm32_mavlink_interface__msg__EncoderState
{
  /// Header with timestamp
  std_msgs__msg__Header header;
  /// Encoder ID
  uint8_t encoder_id;
  /// Current encoder position in counts
  int32_t position;
  /// Current angle in radians
  float angle_rad;
  /// Current angle in degrees
  float angle_deg;
  /// Number of complete revolutions
  uint32_t revolutions;
  /// Z-index pulse detected
  bool z_detected;
  /// Encoder status
  uint8_t status;
  /// Error count
  uint32_t error_count;
} stm32_mavlink_interface__msg__EncoderState;

// Struct for a sequence of stm32_mavlink_interface__msg__EncoderState.
typedef struct stm32_mavlink_interface__msg__EncoderState__Sequence
{
  stm32_mavlink_interface__msg__EncoderState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__EncoderState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_H_

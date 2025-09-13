// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EncoderConfig in the package stm32_mavlink_interface.
/**
  * Encoder configuration message
 */
typedef struct stm32_mavlink_interface__msg__EncoderConfig
{
  /// Encoder ID
  uint8_t encoder_id;
  /// Counts per revolution
  uint16_t cpr;
  /// Invert A signal
  bool invert_a;
  /// Invert B signal
  bool invert_b;
  /// Use Z index signal
  bool use_z;
  /// Watchdog timeout in milliseconds
  uint32_t watchdog_timeout_ms;
  /// Position offset
  int32_t offset_counts;
  /// Enable position wrap-around
  bool wrap_around;
} stm32_mavlink_interface__msg__EncoderConfig;

// Struct for a sequence of stm32_mavlink_interface__msg__EncoderConfig.
typedef struct stm32_mavlink_interface__msg__EncoderConfig__Sequence
{
  stm32_mavlink_interface__msg__EncoderConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__EncoderConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_H_

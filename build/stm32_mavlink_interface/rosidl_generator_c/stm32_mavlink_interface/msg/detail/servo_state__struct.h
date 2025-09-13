// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/ServoState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_H_

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

/// Struct defined in msg/ServoState in the package stm32_mavlink_interface.
/**
  * Servo state feedback message
 */
typedef struct stm32_mavlink_interface__msg__ServoState
{
  std_msgs__msg__Header header;
  /// Servo ID
  uint8_t servo_id;
  /// Current angle in degrees
  float current_angle_deg;
  /// Target angle in degrees
  float target_angle_deg;
  /// Current PWM pulse width
  uint16_t pulse_us;
  /// Servo enabled state
  bool enabled;
  /// Status code (0=OK, 1=NOT_INITIALIZED, 2=TIMER_ERROR, 3=OUT_OF_RANGE, 4=TIMEOUT, 5=CONFIG_ERROR)
  uint8_t status;
  /// Total error count
  uint32_t error_count;
} stm32_mavlink_interface__msg__ServoState;

// Struct for a sequence of stm32_mavlink_interface__msg__ServoState.
typedef struct stm32_mavlink_interface__msg__ServoState__Sequence
{
  stm32_mavlink_interface__msg__ServoState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__ServoState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_H_

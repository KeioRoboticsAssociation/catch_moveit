// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_

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

/// Struct defined in msg/ServoCommand in the package stm32_mavlink_interface.
/**
  * Servo command message
 */
typedef struct stm32_mavlink_interface__msg__ServoCommand
{
  std_msgs__msg__Header header;
  /// Servo ID (1-16)
  uint8_t servo_id;
  /// Target angle in degrees
  float angle_deg;
  /// Direct PWM pulse width in microseconds (0 to use angle)
  uint16_t pulse_us;
  /// Enable/disable servo
  bool enable;
} stm32_mavlink_interface__msg__ServoCommand;

// Struct for a sequence of stm32_mavlink_interface__msg__ServoCommand.
typedef struct stm32_mavlink_interface__msg__ServoCommand__Sequence
{
  stm32_mavlink_interface__msg__ServoCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__ServoCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_H_

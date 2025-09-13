// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CONTROL_MODE_CURRENT'.
/**
  * Control mode
 */
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorCommand__CONTROL_MODE_CURRENT = 0
};

/// Constant 'CONTROL_MODE_VELOCITY'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorCommand__CONTROL_MODE_VELOCITY = 1
};

/// Constant 'CONTROL_MODE_POSITION'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorCommand__CONTROL_MODE_POSITION = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/RobomasterMotorCommand in the package stm32_mavlink_interface.
/**
  * RoboMaster Motor Command Message
  * Motor control command for individual RoboMaster motors
 */
typedef struct stm32_mavlink_interface__msg__RobomasterMotorCommand
{
  std_msgs__msg__Header header;
  /// Motor identification
  /// Motor ID (1-8)
  uint8_t motor_id;
  uint8_t control_mode;
  /// Command values (only one should be used based on control_mode)
  /// Target current in milliamps (-16000 to 16000)
  int16_t target_current_ma;
  /// Target velocity in rotations per second
  float target_velocity_rps;
  /// Target position in radians
  float target_position_rad;
  /// Control flags
  /// Enable/disable motor
  bool enabled;
  /// Reset watchdog timer
  bool reset_watchdog;
  /// Emergency stop this motor
  bool emergency_stop;
} stm32_mavlink_interface__msg__RobomasterMotorCommand;

// Struct for a sequence of stm32_mavlink_interface__msg__RobomasterMotorCommand.
typedef struct stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence
{
  stm32_mavlink_interface__msg__RobomasterMotorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_H_

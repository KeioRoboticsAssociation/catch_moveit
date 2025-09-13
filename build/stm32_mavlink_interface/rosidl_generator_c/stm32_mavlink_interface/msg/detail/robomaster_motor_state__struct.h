// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_OK'.
/**
  * Status and diagnostics
 */
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_OK = 0
};

/// Constant 'STATUS_NOT_INITIALIZED'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_NOT_INITIALIZED = 1
};

/// Constant 'STATUS_CAN_ERROR'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_CAN_ERROR = 2
};

/// Constant 'STATUS_OUT_OF_RANGE'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_OUT_OF_RANGE = 3
};

/// Constant 'STATUS_TIMEOUT'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_TIMEOUT = 4
};

/// Constant 'STATUS_CONFIG_ERROR'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_CONFIG_ERROR = 5
};

/// Constant 'STATUS_OVERHEAT'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_OVERHEAT = 6
};

/// Constant 'STATUS_OVERCURRENT'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorState__STATUS_OVERCURRENT = 7
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/RobomasterMotorState in the package stm32_mavlink_interface.
/**
  * RoboMaster Motor State Message
  * Current state and telemetry of a RoboMaster motor
 */
typedef struct stm32_mavlink_interface__msg__RobomasterMotorState
{
  std_msgs__msg__Header header;
  /// Motor identification
  /// Motor ID (1-8)
  uint8_t motor_id;
  /// Current measurements
  /// Current position in radians
  float current_position_rad;
  /// Current velocity in rotations per second
  float current_velocity_rps;
  /// Current draw in milliamps
  int16_t current_milliamps;
  /// Motor temperature in Celsius
  uint8_t temperature_celsius;
  /// Target values
  /// Target position in radians
  float target_position_rad;
  /// Target velocity in rotations per second
  float target_velocity_rps;
  /// Target current in milliamps
  int16_t target_current_ma;
  /// Control state
  /// Current control mode (0=current, 1=velocity, 2=position)
  uint8_t control_mode;
  /// Motor enabled state
  bool enabled;
  uint8_t status;
  /// Statistics
  /// Timestamp of last command
  uint32_t last_command_time_ms;
  /// Number of command saturations
  uint32_t saturation_count;
  /// Number of communication timeouts
  uint32_t timeout_count;
  /// Number of errors
  uint32_t error_count;
  /// Number of overheating events
  uint32_t overheat_count;
} stm32_mavlink_interface__msg__RobomasterMotorState;

// Struct for a sequence of stm32_mavlink_interface__msg__RobomasterMotorState.
typedef struct stm32_mavlink_interface__msg__RobomasterMotorState__Sequence
{
  stm32_mavlink_interface__msg__RobomasterMotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__RobomasterMotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_H_

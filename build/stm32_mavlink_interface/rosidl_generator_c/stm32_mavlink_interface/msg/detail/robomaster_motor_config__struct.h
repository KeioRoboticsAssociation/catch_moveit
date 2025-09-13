// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_H_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FAILSAFE_HOLD_POSITION'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorConfig__FAILSAFE_HOLD_POSITION = 0
};

/// Constant 'FAILSAFE_BRAKE'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorConfig__FAILSAFE_BRAKE = 1
};

/// Constant 'FAILSAFE_DISABLE_OUTPUT'.
enum
{
  stm32_mavlink_interface__msg__RobomasterMotorConfig__FAILSAFE_DISABLE_OUTPUT = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/RobomasterMotorConfig in the package stm32_mavlink_interface.
/**
  * RoboMaster Motor Configuration Message
  * Configuration parameters for RoboMaster motors
 */
typedef struct stm32_mavlink_interface__msg__RobomasterMotorConfig
{
  std_msgs__msg__Header header;
  /// Motor identification
  /// Motor ID (1-8)
  uint8_t motor_id;
  /// Physical limits
  /// Maximum velocity in RPS
  float max_velocity_rps;
  /// Maximum acceleration in RPS^2
  float max_acceleration_rps2;
  /// Maximum current in milliamps
  int16_t max_current_ma;
  /// Minimum current in milliamps
  int16_t min_current_ma;
  /// Position limits
  /// Minimum position in radians
  float min_position_rad;
  /// Maximum position in radians
  float max_position_rad;
  /// Enable position limits
  bool position_limits_enabled;
  /// Safety settings
  /// Watchdog timeout in milliseconds
  uint32_t watchdog_timeout_ms;
  /// Maximum temperature in Celsius
  uint8_t max_temperature_celsius;
  uint8_t failsafe_behavior;
  /// PID control parameters
  /// Position P gain
  float position_kp;
  /// Position I gain
  float position_ki;
  /// Position D gain
  float position_kd;
  /// Velocity P gain
  float velocity_kp;
  /// Velocity I gain
  float velocity_ki;
  /// Velocity D gain
  float velocity_kd;
  /// Startup settings
  /// Initial position on startup
  float startup_position_rad;
  /// Startup control mode (0=current, 1=velocity, 2=position)
  uint8_t startup_mode;
  /// Start in disabled state
  bool start_disabled;
  /// Direction and offset
  /// Invert rotation direction
  bool direction_inverted;
  /// Position offset in radians
  float position_offset_rad;
} stm32_mavlink_interface__msg__RobomasterMotorConfig;

// Struct for a sequence of stm32_mavlink_interface__msg__RobomasterMotorConfig.
typedef struct stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence
{
  stm32_mavlink_interface__msg__RobomasterMotorConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_H_

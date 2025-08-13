// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_H_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PING'.
enum
{
  dynamixel_controller__msg__DynamixelController__PING = 1
};

/// Constant 'READ_DATA'.
enum
{
  dynamixel_controller__msg__DynamixelController__READ_DATA = 2
};

/// Constant 'WRITE_DATA'.
enum
{
  dynamixel_controller__msg__DynamixelController__WRITE_DATA = 3
};

/// Constant 'REG_WRITE'.
enum
{
  dynamixel_controller__msg__DynamixelController__REG_WRITE = 4
};

/// Constant 'ACTION'.
enum
{
  dynamixel_controller__msg__DynamixelController__ACTION = 5
};

/// Constant 'FACTORY_RESET'.
enum
{
  dynamixel_controller__msg__DynamixelController__FACTORY_RESET = 6
};

/// Constant 'REBOOT'.
enum
{
  dynamixel_controller__msg__DynamixelController__REBOOT = 8
};

/// Constant 'SYNC_READ'.
/**
  * 0x82 in hex
 */
enum
{
  dynamixel_controller__msg__DynamixelController__SYNC_READ = 130
};

/// Constant 'SYNC_WRITE'.
/**
  * 0x83 in hex
 */
enum
{
  dynamixel_controller__msg__DynamixelController__SYNC_WRITE = 131
};

/// Constant 'BULK_READ'.
/**
  * 0x92 in hex
 */
enum
{
  dynamixel_controller__msg__DynamixelController__BULK_READ = 146
};

/// Constant 'BULK_WRITE'.
/**
  * 0x93 in hex
 */
enum
{
  dynamixel_controller__msg__DynamixelController__BULK_WRITE = 147
};

/// Constant 'MODEL_NUMBER'.
/**
  * EEPROM領域のアドレス（読み出し専用または読み書き）
  * -----------------------------
  * 0x00
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MODEL_NUMBER = 0
};

/// Constant 'MODEL_INFORMATION'.
/**
  * 0x02
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MODEL_INFORMATION = 2
};

/// Constant 'FIRMWARE_VERSION'.
/**
  * 0x06
 */
enum
{
  dynamixel_controller__msg__DynamixelController__FIRMWARE_VERSION = 6
};

/// Constant 'ID'.
/**
  * 0x07
 */
enum
{
  dynamixel_controller__msg__DynamixelController__ID = 7
};

/// Constant 'BAUD_RATE'.
/**
  * 0x08
 */
enum
{
  dynamixel_controller__msg__DynamixelController__BAUD_RATE = 8
};

/// Constant 'RETURN_DELAY_TIME'.
/**
  * 0x09
 */
enum
{
  dynamixel_controller__msg__DynamixelController__RETURN_DELAY_TIME = 9
};

/// Constant 'DRIVE_MODE'.
/**
  * 0x0A
 */
enum
{
  dynamixel_controller__msg__DynamixelController__DRIVE_MODE = 10
};

/// Constant 'OPERATING_MODE'.
/**
  * 0x0B
 */
enum
{
  dynamixel_controller__msg__DynamixelController__OPERATING_MODE = 11
};

/// Constant 'SECONDARY_ID'.
/**
  * 0x0C
 */
enum
{
  dynamixel_controller__msg__DynamixelController__SECONDARY_ID = 12
};

/// Constant 'PROTOCOL_TYPE'.
/**
  * 0x0D
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PROTOCOL_TYPE = 13
};

/// Constant 'HOMING_OFFSET'.
/**
  * 0x14
 */
enum
{
  dynamixel_controller__msg__DynamixelController__HOMING_OFFSET = 20
};

/// Constant 'MOVING_THRESHOLD'.
/**
  * 0x18
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MOVING_THRESHOLD = 24
};

/// Constant 'TEMPERATURE_LIMIT'.
/**
  * 0x1F
 */
enum
{
  dynamixel_controller__msg__DynamixelController__TEMPERATURE_LIMIT = 31
};

/// Constant 'MAX_VOLTAGE_LIMIT'.
/**
  * 0x20
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MAX_VOLTAGE_LIMIT = 32
};

/// Constant 'MIN_VOLTAGE_LIMIT'.
/**
  * 0x22
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MIN_VOLTAGE_LIMIT = 34
};

/// Constant 'PWM_LIMIT'.
/**
  * 0x24
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PWM_LIMIT = 36
};

/// Constant 'CURRENT_LIMIT'.
/**
  * 0x26
 */
enum
{
  dynamixel_controller__msg__DynamixelController__CURRENT_LIMIT = 38
};

/// Constant 'VELOCITY_LIMIT'.
/**
  * 0x28
 */
enum
{
  dynamixel_controller__msg__DynamixelController__VELOCITY_LIMIT = 40
};

/// Constant 'MAX_POSITION_LIMIT'.
/**
  * 0x2C
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MAX_POSITION_LIMIT = 44
};

/// Constant 'MIN_POSITION_LIMIT'.
/**
  * 0x30
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MIN_POSITION_LIMIT = 48
};

/// Constant 'TORQUE_ENABLE'.
/**
  * RAM領域のアドレス（読み出し専用または読み書き）
  * -----------------------------
  * 0x40
 */
enum
{
  dynamixel_controller__msg__DynamixelController__TORQUE_ENABLE = 64
};

/// Constant 'LED'.
/**
  * 0x41
 */
enum
{
  dynamixel_controller__msg__DynamixelController__LED = 65
};

/// Constant 'STATUS_RETURN_LEVEL'.
/**
  * 0x44
 */
enum
{
  dynamixel_controller__msg__DynamixelController__STATUS_RETURN_LEVEL = 68
};

/// Constant 'REGISTERED_INSTRUCTION'.
/**
  * 0x45
 */
enum
{
  dynamixel_controller__msg__DynamixelController__REGISTERED_INSTRUCTION = 69
};

/// Constant 'HARDWARE_ERROR_STATUS'.
/**
  * 0x46
 */
enum
{
  dynamixel_controller__msg__DynamixelController__HARDWARE_ERROR_STATUS = 70
};

/// Constant 'VELOCITY_I_GAIN'.
/**
  * 0x4C
 */
enum
{
  dynamixel_controller__msg__DynamixelController__VELOCITY_I_GAIN = 76
};

/// Constant 'VELOCITY_P_GAIN'.
/**
  * 0x4E
 */
enum
{
  dynamixel_controller__msg__DynamixelController__VELOCITY_P_GAIN = 78
};

/// Constant 'POSITION_D_GAIN'.
/**
  * 0x50
 */
enum
{
  dynamixel_controller__msg__DynamixelController__POSITION_D_GAIN = 80
};

/// Constant 'POSITION_I_GAIN'.
/**
  * 0x52
 */
enum
{
  dynamixel_controller__msg__DynamixelController__POSITION_I_GAIN = 82
};

/// Constant 'POSITION_P_GAIN'.
/**
  * 0x54
 */
enum
{
  dynamixel_controller__msg__DynamixelController__POSITION_P_GAIN = 84
};

/// Constant 'FEEDFORWARD_2_GAIN'.
/**
  * 0x58
 */
enum
{
  dynamixel_controller__msg__DynamixelController__FEEDFORWARD_2_GAIN = 88
};

/// Constant 'FEEDFORWARD_1_GAIN'.
/**
  * 0x5A
 */
enum
{
  dynamixel_controller__msg__DynamixelController__FEEDFORWARD_1_GAIN = 90
};

/// Constant 'BUS_WATCHDOG'.
/**
  * 0x62
 */
enum
{
  dynamixel_controller__msg__DynamixelController__BUS_WATCHDOG = 98
};

/// Constant 'GOAL_PWM'.
/**
  * 0x64
 */
enum
{
  dynamixel_controller__msg__DynamixelController__GOAL_PWM = 100
};

/// Constant 'GOAL_CURRENT'.
/**
  * 0x66
 */
enum
{
  dynamixel_controller__msg__DynamixelController__GOAL_CURRENT = 102
};

/// Constant 'GOAL_VELOCITY'.
/**
  * 0x68
 */
enum
{
  dynamixel_controller__msg__DynamixelController__GOAL_VELOCITY = 104
};

/// Constant 'PROFILE_ACCELERATION'.
/**
  * 0x6C
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PROFILE_ACCELERATION = 108
};

/// Constant 'PROFILE_VELOCITY'.
/**
  * 0x70
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PROFILE_VELOCITY = 112
};

/// Constant 'GOAL_POSITION'.
/**
  * 0x74
 */
enum
{
  dynamixel_controller__msg__DynamixelController__GOAL_POSITION = 116
};

/// Constant 'REALTIME_TICK'.
/**
  * 0x78
 */
enum
{
  dynamixel_controller__msg__DynamixelController__REALTIME_TICK = 120
};

/// Constant 'MOVING'.
/**
  * 0x7A
 */
enum
{
  dynamixel_controller__msg__DynamixelController__MOVING = 122
};

/// Constant 'PRESENT_PWM'.
/**
  * 0x7C
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_PWM = 124
};

/// Constant 'PRESENT_CURRENT'.
/**
  * 0x7E
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_CURRENT = 126
};

/// Constant 'PRESENT_VELOCITY'.
/**
  * 0x80
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_VELOCITY = 128
};

/// Constant 'PRESENT_POSITION'.
/**
  * 0x84
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_POSITION = 132
};

/// Constant 'PRESENT_INPUT_VOLTAGE'.
/**
  * 0x90
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_INPUT_VOLTAGE = 144
};

/// Constant 'PRESENT_TEMPERATURE'.
/**
  * 0x92
 */
enum
{
  dynamixel_controller__msg__DynamixelController__PRESENT_TEMPERATURE = 146
};

/// Struct defined in msg/DynamixelController in the package dynamixel_controller.
/**
  * DynamixelControllerConstants.msg
  * =============================
  * Instruction codes (命令コード)
  * -----------------------------
 */
typedef struct dynamixel_controller__msg__DynamixelController
{
  uint8_t structure_needs_at_least_one_member;
} dynamixel_controller__msg__DynamixelController;

// Struct for a sequence of dynamixel_controller__msg__DynamixelController.
typedef struct dynamixel_controller__msg__DynamixelController__Sequence
{
  dynamixel_controller__msg__DynamixelController * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_controller__msg__DynamixelController__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_H_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorState __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorState __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobomasterMotorState_
{
  using Type = RobomasterMotorState_<ContainerAllocator>;

  explicit RobomasterMotorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->current_position_rad = 0.0f;
      this->current_velocity_rps = 0.0f;
      this->current_milliamps = 0;
      this->temperature_celsius = 0;
      this->target_position_rad = 0.0f;
      this->target_velocity_rps = 0.0f;
      this->target_current_ma = 0;
      this->control_mode = 0;
      this->enabled = false;
      this->status = 0;
      this->last_command_time_ms = 0ul;
      this->saturation_count = 0ul;
      this->timeout_count = 0ul;
      this->error_count = 0ul;
      this->overheat_count = 0ul;
    }
  }

  explicit RobomasterMotorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->current_position_rad = 0.0f;
      this->current_velocity_rps = 0.0f;
      this->current_milliamps = 0;
      this->temperature_celsius = 0;
      this->target_position_rad = 0.0f;
      this->target_velocity_rps = 0.0f;
      this->target_current_ma = 0;
      this->control_mode = 0;
      this->enabled = false;
      this->status = 0;
      this->last_command_time_ms = 0ul;
      this->saturation_count = 0ul;
      this->timeout_count = 0ul;
      this->error_count = 0ul;
      this->overheat_count = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _current_position_rad_type =
    float;
  _current_position_rad_type current_position_rad;
  using _current_velocity_rps_type =
    float;
  _current_velocity_rps_type current_velocity_rps;
  using _current_milliamps_type =
    int16_t;
  _current_milliamps_type current_milliamps;
  using _temperature_celsius_type =
    uint8_t;
  _temperature_celsius_type temperature_celsius;
  using _target_position_rad_type =
    float;
  _target_position_rad_type target_position_rad;
  using _target_velocity_rps_type =
    float;
  _target_velocity_rps_type target_velocity_rps;
  using _target_current_ma_type =
    int16_t;
  _target_current_ma_type target_current_ma;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _enabled_type =
    bool;
  _enabled_type enabled;
  using _status_type =
    uint8_t;
  _status_type status;
  using _last_command_time_ms_type =
    uint32_t;
  _last_command_time_ms_type last_command_time_ms;
  using _saturation_count_type =
    uint32_t;
  _saturation_count_type saturation_count;
  using _timeout_count_type =
    uint32_t;
  _timeout_count_type timeout_count;
  using _error_count_type =
    uint32_t;
  _error_count_type error_count;
  using _overheat_count_type =
    uint32_t;
  _overheat_count_type overheat_count;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__motor_id(
    const uint8_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }
  Type & set__current_position_rad(
    const float & _arg)
  {
    this->current_position_rad = _arg;
    return *this;
  }
  Type & set__current_velocity_rps(
    const float & _arg)
  {
    this->current_velocity_rps = _arg;
    return *this;
  }
  Type & set__current_milliamps(
    const int16_t & _arg)
  {
    this->current_milliamps = _arg;
    return *this;
  }
  Type & set__temperature_celsius(
    const uint8_t & _arg)
  {
    this->temperature_celsius = _arg;
    return *this;
  }
  Type & set__target_position_rad(
    const float & _arg)
  {
    this->target_position_rad = _arg;
    return *this;
  }
  Type & set__target_velocity_rps(
    const float & _arg)
  {
    this->target_velocity_rps = _arg;
    return *this;
  }
  Type & set__target_current_ma(
    const int16_t & _arg)
  {
    this->target_current_ma = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__enabled(
    const bool & _arg)
  {
    this->enabled = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__last_command_time_ms(
    const uint32_t & _arg)
  {
    this->last_command_time_ms = _arg;
    return *this;
  }
  Type & set__saturation_count(
    const uint32_t & _arg)
  {
    this->saturation_count = _arg;
    return *this;
  }
  Type & set__timeout_count(
    const uint32_t & _arg)
  {
    this->timeout_count = _arg;
    return *this;
  }
  Type & set__error_count(
    const uint32_t & _arg)
  {
    this->error_count = _arg;
    return *this;
  }
  Type & set__overheat_count(
    const uint32_t & _arg)
  {
    this->overheat_count = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STATUS_OK =
    0u;
  static constexpr uint8_t STATUS_NOT_INITIALIZED =
    1u;
  static constexpr uint8_t STATUS_CAN_ERROR =
    2u;
  static constexpr uint8_t STATUS_OUT_OF_RANGE =
    3u;
  static constexpr uint8_t STATUS_TIMEOUT =
    4u;
  static constexpr uint8_t STATUS_CONFIG_ERROR =
    5u;
  static constexpr uint8_t STATUS_OVERHEAT =
    6u;
  static constexpr uint8_t STATUS_OVERCURRENT =
    7u;

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorState
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorState
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobomasterMotorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->current_position_rad != other.current_position_rad) {
      return false;
    }
    if (this->current_velocity_rps != other.current_velocity_rps) {
      return false;
    }
    if (this->current_milliamps != other.current_milliamps) {
      return false;
    }
    if (this->temperature_celsius != other.temperature_celsius) {
      return false;
    }
    if (this->target_position_rad != other.target_position_rad) {
      return false;
    }
    if (this->target_velocity_rps != other.target_velocity_rps) {
      return false;
    }
    if (this->target_current_ma != other.target_current_ma) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->enabled != other.enabled) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->last_command_time_ms != other.last_command_time_ms) {
      return false;
    }
    if (this->saturation_count != other.saturation_count) {
      return false;
    }
    if (this->timeout_count != other.timeout_count) {
      return false;
    }
    if (this->error_count != other.error_count) {
      return false;
    }
    if (this->overheat_count != other.overheat_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobomasterMotorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobomasterMotorState_

// alias to use template instance with default allocator
using RobomasterMotorState =
  stm32_mavlink_interface::msg::RobomasterMotorState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_NOT_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_CAN_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_OUT_OF_RANGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_TIMEOUT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_CONFIG_ERROR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_OVERHEAT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorState_<ContainerAllocator>::STATUS_OVERCURRENT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_STATE__STRUCT_HPP_

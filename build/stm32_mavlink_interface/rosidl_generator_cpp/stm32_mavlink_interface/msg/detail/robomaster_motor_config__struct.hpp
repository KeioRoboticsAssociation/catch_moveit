// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_

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
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorConfig __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorConfig __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobomasterMotorConfig_
{
  using Type = RobomasterMotorConfig_<ContainerAllocator>;

  explicit RobomasterMotorConfig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->max_velocity_rps = 0.0f;
      this->max_acceleration_rps2 = 0.0f;
      this->max_current_ma = 0;
      this->min_current_ma = 0;
      this->min_position_rad = 0.0f;
      this->max_position_rad = 0.0f;
      this->position_limits_enabled = false;
      this->watchdog_timeout_ms = 0ul;
      this->max_temperature_celsius = 0;
      this->failsafe_behavior = 0;
      this->position_kp = 0.0f;
      this->position_ki = 0.0f;
      this->position_kd = 0.0f;
      this->velocity_kp = 0.0f;
      this->velocity_ki = 0.0f;
      this->velocity_kd = 0.0f;
      this->startup_position_rad = 0.0f;
      this->startup_mode = 0;
      this->start_disabled = false;
      this->direction_inverted = false;
      this->position_offset_rad = 0.0f;
    }
  }

  explicit RobomasterMotorConfig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->max_velocity_rps = 0.0f;
      this->max_acceleration_rps2 = 0.0f;
      this->max_current_ma = 0;
      this->min_current_ma = 0;
      this->min_position_rad = 0.0f;
      this->max_position_rad = 0.0f;
      this->position_limits_enabled = false;
      this->watchdog_timeout_ms = 0ul;
      this->max_temperature_celsius = 0;
      this->failsafe_behavior = 0;
      this->position_kp = 0.0f;
      this->position_ki = 0.0f;
      this->position_kd = 0.0f;
      this->velocity_kp = 0.0f;
      this->velocity_ki = 0.0f;
      this->velocity_kd = 0.0f;
      this->startup_position_rad = 0.0f;
      this->startup_mode = 0;
      this->start_disabled = false;
      this->direction_inverted = false;
      this->position_offset_rad = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _max_velocity_rps_type =
    float;
  _max_velocity_rps_type max_velocity_rps;
  using _max_acceleration_rps2_type =
    float;
  _max_acceleration_rps2_type max_acceleration_rps2;
  using _max_current_ma_type =
    int16_t;
  _max_current_ma_type max_current_ma;
  using _min_current_ma_type =
    int16_t;
  _min_current_ma_type min_current_ma;
  using _min_position_rad_type =
    float;
  _min_position_rad_type min_position_rad;
  using _max_position_rad_type =
    float;
  _max_position_rad_type max_position_rad;
  using _position_limits_enabled_type =
    bool;
  _position_limits_enabled_type position_limits_enabled;
  using _watchdog_timeout_ms_type =
    uint32_t;
  _watchdog_timeout_ms_type watchdog_timeout_ms;
  using _max_temperature_celsius_type =
    uint8_t;
  _max_temperature_celsius_type max_temperature_celsius;
  using _failsafe_behavior_type =
    uint8_t;
  _failsafe_behavior_type failsafe_behavior;
  using _position_kp_type =
    float;
  _position_kp_type position_kp;
  using _position_ki_type =
    float;
  _position_ki_type position_ki;
  using _position_kd_type =
    float;
  _position_kd_type position_kd;
  using _velocity_kp_type =
    float;
  _velocity_kp_type velocity_kp;
  using _velocity_ki_type =
    float;
  _velocity_ki_type velocity_ki;
  using _velocity_kd_type =
    float;
  _velocity_kd_type velocity_kd;
  using _startup_position_rad_type =
    float;
  _startup_position_rad_type startup_position_rad;
  using _startup_mode_type =
    uint8_t;
  _startup_mode_type startup_mode;
  using _start_disabled_type =
    bool;
  _start_disabled_type start_disabled;
  using _direction_inverted_type =
    bool;
  _direction_inverted_type direction_inverted;
  using _position_offset_rad_type =
    float;
  _position_offset_rad_type position_offset_rad;

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
  Type & set__max_velocity_rps(
    const float & _arg)
  {
    this->max_velocity_rps = _arg;
    return *this;
  }
  Type & set__max_acceleration_rps2(
    const float & _arg)
  {
    this->max_acceleration_rps2 = _arg;
    return *this;
  }
  Type & set__max_current_ma(
    const int16_t & _arg)
  {
    this->max_current_ma = _arg;
    return *this;
  }
  Type & set__min_current_ma(
    const int16_t & _arg)
  {
    this->min_current_ma = _arg;
    return *this;
  }
  Type & set__min_position_rad(
    const float & _arg)
  {
    this->min_position_rad = _arg;
    return *this;
  }
  Type & set__max_position_rad(
    const float & _arg)
  {
    this->max_position_rad = _arg;
    return *this;
  }
  Type & set__position_limits_enabled(
    const bool & _arg)
  {
    this->position_limits_enabled = _arg;
    return *this;
  }
  Type & set__watchdog_timeout_ms(
    const uint32_t & _arg)
  {
    this->watchdog_timeout_ms = _arg;
    return *this;
  }
  Type & set__max_temperature_celsius(
    const uint8_t & _arg)
  {
    this->max_temperature_celsius = _arg;
    return *this;
  }
  Type & set__failsafe_behavior(
    const uint8_t & _arg)
  {
    this->failsafe_behavior = _arg;
    return *this;
  }
  Type & set__position_kp(
    const float & _arg)
  {
    this->position_kp = _arg;
    return *this;
  }
  Type & set__position_ki(
    const float & _arg)
  {
    this->position_ki = _arg;
    return *this;
  }
  Type & set__position_kd(
    const float & _arg)
  {
    this->position_kd = _arg;
    return *this;
  }
  Type & set__velocity_kp(
    const float & _arg)
  {
    this->velocity_kp = _arg;
    return *this;
  }
  Type & set__velocity_ki(
    const float & _arg)
  {
    this->velocity_ki = _arg;
    return *this;
  }
  Type & set__velocity_kd(
    const float & _arg)
  {
    this->velocity_kd = _arg;
    return *this;
  }
  Type & set__startup_position_rad(
    const float & _arg)
  {
    this->startup_position_rad = _arg;
    return *this;
  }
  Type & set__startup_mode(
    const uint8_t & _arg)
  {
    this->startup_mode = _arg;
    return *this;
  }
  Type & set__start_disabled(
    const bool & _arg)
  {
    this->start_disabled = _arg;
    return *this;
  }
  Type & set__direction_inverted(
    const bool & _arg)
  {
    this->direction_inverted = _arg;
    return *this;
  }
  Type & set__position_offset_rad(
    const float & _arg)
  {
    this->position_offset_rad = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FAILSAFE_HOLD_POSITION =
    0u;
  static constexpr uint8_t FAILSAFE_BRAKE =
    1u;
  static constexpr uint8_t FAILSAFE_DISABLE_OUTPUT =
    2u;

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorConfig
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorConfig
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobomasterMotorConfig_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->max_velocity_rps != other.max_velocity_rps) {
      return false;
    }
    if (this->max_acceleration_rps2 != other.max_acceleration_rps2) {
      return false;
    }
    if (this->max_current_ma != other.max_current_ma) {
      return false;
    }
    if (this->min_current_ma != other.min_current_ma) {
      return false;
    }
    if (this->min_position_rad != other.min_position_rad) {
      return false;
    }
    if (this->max_position_rad != other.max_position_rad) {
      return false;
    }
    if (this->position_limits_enabled != other.position_limits_enabled) {
      return false;
    }
    if (this->watchdog_timeout_ms != other.watchdog_timeout_ms) {
      return false;
    }
    if (this->max_temperature_celsius != other.max_temperature_celsius) {
      return false;
    }
    if (this->failsafe_behavior != other.failsafe_behavior) {
      return false;
    }
    if (this->position_kp != other.position_kp) {
      return false;
    }
    if (this->position_ki != other.position_ki) {
      return false;
    }
    if (this->position_kd != other.position_kd) {
      return false;
    }
    if (this->velocity_kp != other.velocity_kp) {
      return false;
    }
    if (this->velocity_ki != other.velocity_ki) {
      return false;
    }
    if (this->velocity_kd != other.velocity_kd) {
      return false;
    }
    if (this->startup_position_rad != other.startup_position_rad) {
      return false;
    }
    if (this->startup_mode != other.startup_mode) {
      return false;
    }
    if (this->start_disabled != other.start_disabled) {
      return false;
    }
    if (this->direction_inverted != other.direction_inverted) {
      return false;
    }
    if (this->position_offset_rad != other.position_offset_rad) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobomasterMotorConfig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobomasterMotorConfig_

// alias to use template instance with default allocator
using RobomasterMotorConfig =
  stm32_mavlink_interface::msg::RobomasterMotorConfig_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorConfig_<ContainerAllocator>::FAILSAFE_HOLD_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorConfig_<ContainerAllocator>::FAILSAFE_BRAKE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorConfig_<ContainerAllocator>::FAILSAFE_DISABLE_OUTPUT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_

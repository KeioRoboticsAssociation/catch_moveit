// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorCommand __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorCommand __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobomasterMotorCommand_
{
  using Type = RobomasterMotorCommand_<ContainerAllocator>;

  explicit RobomasterMotorCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->control_mode = 0;
      this->target_current_ma = 0;
      this->target_velocity_rps = 0.0f;
      this->target_position_rad = 0.0f;
      this->enabled = false;
      this->reset_watchdog = false;
      this->emergency_stop = false;
    }
  }

  explicit RobomasterMotorCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
      this->control_mode = 0;
      this->target_current_ma = 0;
      this->target_velocity_rps = 0.0f;
      this->target_position_rad = 0.0f;
      this->enabled = false;
      this->reset_watchdog = false;
      this->emergency_stop = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _target_current_ma_type =
    int16_t;
  _target_current_ma_type target_current_ma;
  using _target_velocity_rps_type =
    float;
  _target_velocity_rps_type target_velocity_rps;
  using _target_position_rad_type =
    float;
  _target_position_rad_type target_position_rad;
  using _enabled_type =
    bool;
  _enabled_type enabled;
  using _reset_watchdog_type =
    bool;
  _reset_watchdog_type reset_watchdog;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;

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
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__target_current_ma(
    const int16_t & _arg)
  {
    this->target_current_ma = _arg;
    return *this;
  }
  Type & set__target_velocity_rps(
    const float & _arg)
  {
    this->target_velocity_rps = _arg;
    return *this;
  }
  Type & set__target_position_rad(
    const float & _arg)
  {
    this->target_position_rad = _arg;
    return *this;
  }
  Type & set__enabled(
    const bool & _arg)
  {
    this->enabled = _arg;
    return *this;
  }
  Type & set__reset_watchdog(
    const bool & _arg)
  {
    this->reset_watchdog = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t CONTROL_MODE_CURRENT =
    0u;
  static constexpr uint8_t CONTROL_MODE_VELOCITY =
    1u;
  static constexpr uint8_t CONTROL_MODE_POSITION =
    2u;

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorCommand
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__RobomasterMotorCommand
    std::shared_ptr<stm32_mavlink_interface::msg::RobomasterMotorCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobomasterMotorCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->motor_id != other.motor_id) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->target_current_ma != other.target_current_ma) {
      return false;
    }
    if (this->target_velocity_rps != other.target_velocity_rps) {
      return false;
    }
    if (this->target_position_rad != other.target_position_rad) {
      return false;
    }
    if (this->enabled != other.enabled) {
      return false;
    }
    if (this->reset_watchdog != other.reset_watchdog) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobomasterMotorCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobomasterMotorCommand_

// alias to use template instance with default allocator
using RobomasterMotorCommand =
  stm32_mavlink_interface::msg::RobomasterMotorCommand_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorCommand_<ContainerAllocator>::CONTROL_MODE_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorCommand_<ContainerAllocator>::CONTROL_MODE_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t RobomasterMotorCommand_<ContainerAllocator>::CONTROL_MODE_POSITION;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ROBOMASTER_MOTOR_COMMAND__STRUCT_HPP_

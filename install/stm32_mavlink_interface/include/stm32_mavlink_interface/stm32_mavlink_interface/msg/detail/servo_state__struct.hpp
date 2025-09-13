// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/ServoState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_HPP_

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
# define DEPRECATED__stm32_mavlink_interface__msg__ServoState __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__ServoState __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ServoState_
{
  using Type = ServoState_<ContainerAllocator>;

  explicit ServoState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->current_angle_deg = 0.0f;
      this->target_angle_deg = 0.0f;
      this->pulse_us = 0;
      this->enabled = false;
      this->status = 0;
      this->error_count = 0ul;
    }
  }

  explicit ServoState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->current_angle_deg = 0.0f;
      this->target_angle_deg = 0.0f;
      this->pulse_us = 0;
      this->enabled = false;
      this->status = 0;
      this->error_count = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _servo_id_type =
    uint8_t;
  _servo_id_type servo_id;
  using _current_angle_deg_type =
    float;
  _current_angle_deg_type current_angle_deg;
  using _target_angle_deg_type =
    float;
  _target_angle_deg_type target_angle_deg;
  using _pulse_us_type =
    uint16_t;
  _pulse_us_type pulse_us;
  using _enabled_type =
    bool;
  _enabled_type enabled;
  using _status_type =
    uint8_t;
  _status_type status;
  using _error_count_type =
    uint32_t;
  _error_count_type error_count;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__servo_id(
    const uint8_t & _arg)
  {
    this->servo_id = _arg;
    return *this;
  }
  Type & set__current_angle_deg(
    const float & _arg)
  {
    this->current_angle_deg = _arg;
    return *this;
  }
  Type & set__target_angle_deg(
    const float & _arg)
  {
    this->target_angle_deg = _arg;
    return *this;
  }
  Type & set__pulse_us(
    const uint16_t & _arg)
  {
    this->pulse_us = _arg;
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
  Type & set__error_count(
    const uint32_t & _arg)
  {
    this->error_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__ServoState
    std::shared_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__ServoState
    std::shared_ptr<stm32_mavlink_interface::msg::ServoState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->servo_id != other.servo_id) {
      return false;
    }
    if (this->current_angle_deg != other.current_angle_deg) {
      return false;
    }
    if (this->target_angle_deg != other.target_angle_deg) {
      return false;
    }
    if (this->pulse_us != other.pulse_us) {
      return false;
    }
    if (this->enabled != other.enabled) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->error_count != other.error_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoState_

// alias to use template instance with default allocator
using ServoState =
  stm32_mavlink_interface::msg::ServoState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_STATE__STRUCT_HPP_

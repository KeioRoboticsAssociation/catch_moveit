// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/ServoCommand.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__stm32_mavlink_interface__msg__ServoCommand __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__ServoCommand __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ServoCommand_
{
  using Type = ServoCommand_<ContainerAllocator>;

  explicit ServoCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->angle_deg = 0.0f;
      this->pulse_us = 0;
      this->enable = false;
    }
  }

  explicit ServoCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->angle_deg = 0.0f;
      this->pulse_us = 0;
      this->enable = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _servo_id_type =
    uint8_t;
  _servo_id_type servo_id;
  using _angle_deg_type =
    float;
  _angle_deg_type angle_deg;
  using _pulse_us_type =
    uint16_t;
  _pulse_us_type pulse_us;
  using _enable_type =
    bool;
  _enable_type enable;

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
  Type & set__angle_deg(
    const float & _arg)
  {
    this->angle_deg = _arg;
    return *this;
  }
  Type & set__pulse_us(
    const uint16_t & _arg)
  {
    this->pulse_us = _arg;
    return *this;
  }
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__ServoCommand
    std::shared_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__ServoCommand
    std::shared_ptr<stm32_mavlink_interface::msg::ServoCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->servo_id != other.servo_id) {
      return false;
    }
    if (this->angle_deg != other.angle_deg) {
      return false;
    }
    if (this->pulse_us != other.pulse_us) {
      return false;
    }
    if (this->enable != other.enable) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoCommand_

// alias to use template instance with default allocator
using ServoCommand =
  stm32_mavlink_interface::msg::ServoCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__SERVO_COMMAND__STRUCT_HPP_

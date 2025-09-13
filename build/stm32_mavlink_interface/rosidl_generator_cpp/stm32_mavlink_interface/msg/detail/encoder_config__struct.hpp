// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__msg__EncoderConfig __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__EncoderConfig __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderConfig_
{
  using Type = EncoderConfig_<ContainerAllocator>;

  explicit EncoderConfig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_id = 0;
      this->cpr = 0;
      this->invert_a = false;
      this->invert_b = false;
      this->use_z = false;
      this->watchdog_timeout_ms = 0ul;
      this->offset_counts = 0l;
      this->wrap_around = false;
    }
  }

  explicit EncoderConfig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_id = 0;
      this->cpr = 0;
      this->invert_a = false;
      this->invert_b = false;
      this->use_z = false;
      this->watchdog_timeout_ms = 0ul;
      this->offset_counts = 0l;
      this->wrap_around = false;
    }
  }

  // field types and members
  using _encoder_id_type =
    uint8_t;
  _encoder_id_type encoder_id;
  using _cpr_type =
    uint16_t;
  _cpr_type cpr;
  using _invert_a_type =
    bool;
  _invert_a_type invert_a;
  using _invert_b_type =
    bool;
  _invert_b_type invert_b;
  using _use_z_type =
    bool;
  _use_z_type use_z;
  using _watchdog_timeout_ms_type =
    uint32_t;
  _watchdog_timeout_ms_type watchdog_timeout_ms;
  using _offset_counts_type =
    int32_t;
  _offset_counts_type offset_counts;
  using _wrap_around_type =
    bool;
  _wrap_around_type wrap_around;

  // setters for named parameter idiom
  Type & set__encoder_id(
    const uint8_t & _arg)
  {
    this->encoder_id = _arg;
    return *this;
  }
  Type & set__cpr(
    const uint16_t & _arg)
  {
    this->cpr = _arg;
    return *this;
  }
  Type & set__invert_a(
    const bool & _arg)
  {
    this->invert_a = _arg;
    return *this;
  }
  Type & set__invert_b(
    const bool & _arg)
  {
    this->invert_b = _arg;
    return *this;
  }
  Type & set__use_z(
    const bool & _arg)
  {
    this->use_z = _arg;
    return *this;
  }
  Type & set__watchdog_timeout_ms(
    const uint32_t & _arg)
  {
    this->watchdog_timeout_ms = _arg;
    return *this;
  }
  Type & set__offset_counts(
    const int32_t & _arg)
  {
    this->offset_counts = _arg;
    return *this;
  }
  Type & set__wrap_around(
    const bool & _arg)
  {
    this->wrap_around = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__EncoderConfig
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__EncoderConfig
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderConfig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderConfig_ & other) const
  {
    if (this->encoder_id != other.encoder_id) {
      return false;
    }
    if (this->cpr != other.cpr) {
      return false;
    }
    if (this->invert_a != other.invert_a) {
      return false;
    }
    if (this->invert_b != other.invert_b) {
      return false;
    }
    if (this->use_z != other.use_z) {
      return false;
    }
    if (this->watchdog_timeout_ms != other.watchdog_timeout_ms) {
      return false;
    }
    if (this->offset_counts != other.offset_counts) {
      return false;
    }
    if (this->wrap_around != other.wrap_around) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderConfig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderConfig_

// alias to use template instance with default allocator
using EncoderConfig =
  stm32_mavlink_interface::msg::EncoderConfig_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_CONFIG__STRUCT_HPP_

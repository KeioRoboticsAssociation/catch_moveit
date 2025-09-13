// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:msg/EncoderState.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_HPP_

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
# define DEPRECATED__stm32_mavlink_interface__msg__EncoderState __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__msg__EncoderState __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderState_
{
  using Type = EncoderState_<ContainerAllocator>;

  explicit EncoderState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_id = 0;
      this->position = 0l;
      this->angle_rad = 0.0f;
      this->angle_deg = 0.0f;
      this->revolutions = 0ul;
      this->z_detected = false;
      this->status = 0;
      this->error_count = 0ul;
    }
  }

  explicit EncoderState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->encoder_id = 0;
      this->position = 0l;
      this->angle_rad = 0.0f;
      this->angle_deg = 0.0f;
      this->revolutions = 0ul;
      this->z_detected = false;
      this->status = 0;
      this->error_count = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _encoder_id_type =
    uint8_t;
  _encoder_id_type encoder_id;
  using _position_type =
    int32_t;
  _position_type position;
  using _angle_rad_type =
    float;
  _angle_rad_type angle_rad;
  using _angle_deg_type =
    float;
  _angle_deg_type angle_deg;
  using _revolutions_type =
    uint32_t;
  _revolutions_type revolutions;
  using _z_detected_type =
    bool;
  _z_detected_type z_detected;
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
  Type & set__encoder_id(
    const uint8_t & _arg)
  {
    this->encoder_id = _arg;
    return *this;
  }
  Type & set__position(
    const int32_t & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__angle_rad(
    const float & _arg)
  {
    this->angle_rad = _arg;
    return *this;
  }
  Type & set__angle_deg(
    const float & _arg)
  {
    this->angle_deg = _arg;
    return *this;
  }
  Type & set__revolutions(
    const uint32_t & _arg)
  {
    this->revolutions = _arg;
    return *this;
  }
  Type & set__z_detected(
    const bool & _arg)
  {
    this->z_detected = _arg;
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
    stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__msg__EncoderState
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__msg__EncoderState
    std::shared_ptr<stm32_mavlink_interface::msg::EncoderState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->encoder_id != other.encoder_id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->angle_rad != other.angle_rad) {
      return false;
    }
    if (this->angle_deg != other.angle_deg) {
      return false;
    }
    if (this->revolutions != other.revolutions) {
      return false;
    }
    if (this->z_detected != other.z_detected) {
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
  bool operator!=(const EncoderState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderState_

// alias to use template instance with default allocator
using EncoderState =
  stm32_mavlink_interface::msg::EncoderState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__MSG__DETAIL__ENCODER_STATE__STRUCT_HPP_

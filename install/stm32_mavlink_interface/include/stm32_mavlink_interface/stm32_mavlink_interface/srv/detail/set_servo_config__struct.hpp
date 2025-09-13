// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Request __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetServoConfig_Request_
{
  using Type = SetServoConfig_Request_<ContainerAllocator>;

  explicit SetServoConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->angle_min_deg = 0.0f;
      this->angle_max_deg = 0.0f;
      this->pulse_min_us = 0;
      this->pulse_max_us = 0;
      this->pulse_neutral_us = 0;
      this->direction_inverted = false;
      this->offset_deg = 0.0f;
      this->max_velocity_deg_per_s = 0.0f;
      this->max_acceleration_deg_per_s2 = 0.0f;
      this->watchdog_timeout_ms = 0ul;
      this->fail_safe_behavior = 0;
      this->save_to_flash = false;
    }
  }

  explicit SetServoConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->servo_id = 0;
      this->angle_min_deg = 0.0f;
      this->angle_max_deg = 0.0f;
      this->pulse_min_us = 0;
      this->pulse_max_us = 0;
      this->pulse_neutral_us = 0;
      this->direction_inverted = false;
      this->offset_deg = 0.0f;
      this->max_velocity_deg_per_s = 0.0f;
      this->max_acceleration_deg_per_s2 = 0.0f;
      this->watchdog_timeout_ms = 0ul;
      this->fail_safe_behavior = 0;
      this->save_to_flash = false;
    }
  }

  // field types and members
  using _servo_id_type =
    uint8_t;
  _servo_id_type servo_id;
  using _angle_min_deg_type =
    float;
  _angle_min_deg_type angle_min_deg;
  using _angle_max_deg_type =
    float;
  _angle_max_deg_type angle_max_deg;
  using _pulse_min_us_type =
    uint16_t;
  _pulse_min_us_type pulse_min_us;
  using _pulse_max_us_type =
    uint16_t;
  _pulse_max_us_type pulse_max_us;
  using _pulse_neutral_us_type =
    uint16_t;
  _pulse_neutral_us_type pulse_neutral_us;
  using _direction_inverted_type =
    bool;
  _direction_inverted_type direction_inverted;
  using _offset_deg_type =
    float;
  _offset_deg_type offset_deg;
  using _max_velocity_deg_per_s_type =
    float;
  _max_velocity_deg_per_s_type max_velocity_deg_per_s;
  using _max_acceleration_deg_per_s2_type =
    float;
  _max_acceleration_deg_per_s2_type max_acceleration_deg_per_s2;
  using _watchdog_timeout_ms_type =
    uint32_t;
  _watchdog_timeout_ms_type watchdog_timeout_ms;
  using _fail_safe_behavior_type =
    uint8_t;
  _fail_safe_behavior_type fail_safe_behavior;
  using _save_to_flash_type =
    bool;
  _save_to_flash_type save_to_flash;

  // setters for named parameter idiom
  Type & set__servo_id(
    const uint8_t & _arg)
  {
    this->servo_id = _arg;
    return *this;
  }
  Type & set__angle_min_deg(
    const float & _arg)
  {
    this->angle_min_deg = _arg;
    return *this;
  }
  Type & set__angle_max_deg(
    const float & _arg)
  {
    this->angle_max_deg = _arg;
    return *this;
  }
  Type & set__pulse_min_us(
    const uint16_t & _arg)
  {
    this->pulse_min_us = _arg;
    return *this;
  }
  Type & set__pulse_max_us(
    const uint16_t & _arg)
  {
    this->pulse_max_us = _arg;
    return *this;
  }
  Type & set__pulse_neutral_us(
    const uint16_t & _arg)
  {
    this->pulse_neutral_us = _arg;
    return *this;
  }
  Type & set__direction_inverted(
    const bool & _arg)
  {
    this->direction_inverted = _arg;
    return *this;
  }
  Type & set__offset_deg(
    const float & _arg)
  {
    this->offset_deg = _arg;
    return *this;
  }
  Type & set__max_velocity_deg_per_s(
    const float & _arg)
  {
    this->max_velocity_deg_per_s = _arg;
    return *this;
  }
  Type & set__max_acceleration_deg_per_s2(
    const float & _arg)
  {
    this->max_acceleration_deg_per_s2 = _arg;
    return *this;
  }
  Type & set__watchdog_timeout_ms(
    const uint32_t & _arg)
  {
    this->watchdog_timeout_ms = _arg;
    return *this;
  }
  Type & set__fail_safe_behavior(
    const uint8_t & _arg)
  {
    this->fail_safe_behavior = _arg;
    return *this;
  }
  Type & set__save_to_flash(
    const bool & _arg)
  {
    this->save_to_flash = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Request
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Request
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetServoConfig_Request_ & other) const
  {
    if (this->servo_id != other.servo_id) {
      return false;
    }
    if (this->angle_min_deg != other.angle_min_deg) {
      return false;
    }
    if (this->angle_max_deg != other.angle_max_deg) {
      return false;
    }
    if (this->pulse_min_us != other.pulse_min_us) {
      return false;
    }
    if (this->pulse_max_us != other.pulse_max_us) {
      return false;
    }
    if (this->pulse_neutral_us != other.pulse_neutral_us) {
      return false;
    }
    if (this->direction_inverted != other.direction_inverted) {
      return false;
    }
    if (this->offset_deg != other.offset_deg) {
      return false;
    }
    if (this->max_velocity_deg_per_s != other.max_velocity_deg_per_s) {
      return false;
    }
    if (this->max_acceleration_deg_per_s2 != other.max_acceleration_deg_per_s2) {
      return false;
    }
    if (this->watchdog_timeout_ms != other.watchdog_timeout_ms) {
      return false;
    }
    if (this->fail_safe_behavior != other.fail_safe_behavior) {
      return false;
    }
    if (this->save_to_flash != other.save_to_flash) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetServoConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetServoConfig_Request_

// alias to use template instance with default allocator
using SetServoConfig_Request =
  stm32_mavlink_interface::srv::SetServoConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stm32_mavlink_interface


#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Response __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetServoConfig_Response_
{
  using Type = SetServoConfig_Response_<ContainerAllocator>;

  explicit SetServoConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetServoConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Response
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__srv__SetServoConfig_Response
    std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetServoConfig_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetServoConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetServoConfig_Response_

// alias to use template instance with default allocator
using SetServoConfig_Response =
  stm32_mavlink_interface::srv::SetServoConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stm32_mavlink_interface

namespace stm32_mavlink_interface
{

namespace srv
{

struct SetServoConfig
{
  using Request = stm32_mavlink_interface::srv::SetServoConfig_Request;
  using Response = stm32_mavlink_interface::srv::SetServoConfig_Response;
};

}  // namespace srv

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__SET_SERVO_CONFIG__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from stm32_mavlink_interface:srv/GetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice

#ifndef STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_
#define STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobomasterMotorConfig_Request_
{
  using Type = GetRobomasterMotorConfig_Request_<ContainerAllocator>;

  explicit GetRobomasterMotorConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
    }
  }

  explicit GetRobomasterMotorConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->motor_id = 0;
    }
  }

  // field types and members
  using _motor_id_type =
    uint8_t;
  _motor_id_type motor_id;

  // setters for named parameter idiom
  Type & set__motor_id(
    const uint8_t & _arg)
  {
    this->motor_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Request
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobomasterMotorConfig_Request_ & other) const
  {
    if (this->motor_id != other.motor_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobomasterMotorConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobomasterMotorConfig_Request_

// alias to use template instance with default allocator
using GetRobomasterMotorConfig_Request =
  stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stm32_mavlink_interface


// Include directives for member types
// Member 'config'
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response __declspec(deprecated)
#endif

namespace stm32_mavlink_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetRobomasterMotorConfig_Response_
{
  using Type = GetRobomasterMotorConfig_Response_<ContainerAllocator>;

  explicit GetRobomasterMotorConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : config(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit GetRobomasterMotorConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : config(_alloc, _init),
    message(_alloc)
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
  using _config_type =
    stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator>;
  _config_type config;
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
  Type & set__config(
    const stm32_mavlink_interface::msg::RobomasterMotorConfig_<ContainerAllocator> & _arg)
  {
    this->config = _arg;
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
    stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__stm32_mavlink_interface__srv__GetRobomasterMotorConfig_Response
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetRobomasterMotorConfig_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->config != other.config) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetRobomasterMotorConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetRobomasterMotorConfig_Response_

// alias to use template instance with default allocator
using GetRobomasterMotorConfig_Response =
  stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace stm32_mavlink_interface

namespace stm32_mavlink_interface
{

namespace srv
{

struct GetRobomasterMotorConfig
{
  using Request = stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Request;
  using Response = stm32_mavlink_interface::srv::GetRobomasterMotorConfig_Response;
};

}  // namespace srv

}  // namespace stm32_mavlink_interface

#endif  // STM32_MAVLINK_INTERFACE__SRV__DETAIL__GET_ROBOMASTER_MOTOR_CONFIG__STRUCT_HPP_

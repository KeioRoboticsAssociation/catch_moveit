// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dynamixel_controller__msg__DynamixelResponse __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_controller__msg__DynamixelResponse __declspec(deprecated)
#endif

namespace dynamixel_controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelResponse_
{
  using Type = DynamixelResponse_<ContainerAllocator>;

  explicit DynamixelResponse_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  explicit DynamixelResponse_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
    }
  }

  // field types and members
  using _command_type =
    uint8_t;
  _command_type command;
  using _ids_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _ids_type ids;
  using _error_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _error_type error;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__command(
    const uint8_t & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__ids(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->ids = _arg;
    return *this;
  }
  Type & set__error(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelResponse
    std::shared_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelResponse
    std::shared_ptr<dynamixel_controller::msg::DynamixelResponse_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelResponse_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->ids != other.ids) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelResponse_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelResponse_

// alias to use template instance with default allocator
using DynamixelResponse =
  dynamixel_controller::msg::DynamixelResponse_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_RESPONSE__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_controller:msg/DynamixelCommand.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dynamixel_controller__msg__DynamixelCommand __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_controller__msg__DynamixelCommand __declspec(deprecated)
#endif

namespace dynamixel_controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelCommand_
{
  using Type = DynamixelCommand_<ContainerAllocator>;

  explicit DynamixelCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
      this->address = 0;
      this->length = 0;
    }
  }

  explicit DynamixelCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->command = 0;
      this->address = 0;
      this->length = 0;
    }
  }

  // field types and members
  using _command_type =
    uint8_t;
  _command_type command;
  using _ids_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _ids_type ids;
  using _address_type =
    uint16_t;
  _address_type address;
  using _length_type =
    uint16_t;
  _length_type length;
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
  Type & set__address(
    const uint16_t & _arg)
  {
    this->address = _arg;
    return *this;
  }
  Type & set__length(
    const uint16_t & _arg)
  {
    this->length = _arg;
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
    dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelCommand
    std::shared_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelCommand
    std::shared_ptr<dynamixel_controller::msg::DynamixelCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelCommand_ & other) const
  {
    if (this->command != other.command) {
      return false;
    }
    if (this->ids != other.ids) {
      return false;
    }
    if (this->address != other.address) {
      return false;
    }
    if (this->length != other.length) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelCommand_

// alias to use template instance with default allocator
using DynamixelCommand =
  dynamixel_controller::msg::DynamixelCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_COMMAND__STRUCT_HPP_

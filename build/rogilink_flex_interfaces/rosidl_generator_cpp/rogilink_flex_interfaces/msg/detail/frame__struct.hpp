// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_HPP_
#define ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rogilink_flex_interfaces__msg__Frame __attribute__((deprecated))
#else
# define DEPRECATED__rogilink_flex_interfaces__msg__Frame __declspec(deprecated)
#endif

namespace rogilink_flex_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Frame_
{
  using Type = Frame_<ContainerAllocator>;

  explicit Frame_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->frame_id = 0;
      this->device_id = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->frame_id = 0;
      this->device_id = 0;
      this->topic = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic = "";
    }
  }

  explicit Frame_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->frame_id = 0;
      this->device_id = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->frame_id = 0;
      this->device_id = 0;
      this->topic = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic = "";
    }
  }

  // field types and members
  using _frame_id_type =
    uint8_t;
  _frame_id_type frame_id;
  using _device_id_type =
    uint8_t;
  _device_id_type device_id;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _topic_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_type topic;

  // setters for named parameter idiom
  Type & set__frame_id(
    const uint8_t & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__device_id(
    const uint8_t & _arg)
  {
    this->device_id = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__topic(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> *;
  using ConstRawPtr =
    const rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rogilink_flex_interfaces__msg__Frame
    std::shared_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rogilink_flex_interfaces__msg__Frame
    std::shared_ptr<rogilink_flex_interfaces::msg::Frame_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Frame_ & other) const
  {
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->device_id != other.device_id) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->topic != other.topic) {
      return false;
    }
    return true;
  }
  bool operator!=(const Frame_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Frame_

// alias to use template instance with default allocator
using Frame =
  rogilink_flex_interfaces::msg::Frame_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__MSG__DETAIL__FRAME__STRUCT_HPP_

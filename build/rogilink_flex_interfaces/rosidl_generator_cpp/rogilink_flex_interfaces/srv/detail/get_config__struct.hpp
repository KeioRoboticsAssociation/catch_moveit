// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rogilink_flex_interfaces:srv/GetConfig.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_HPP_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Request __attribute__((deprecated))
#else
# define DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Request __declspec(deprecated)
#endif

namespace rogilink_flex_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetConfig_Request_
{
  using Type = GetConfig_Request_<ContainerAllocator>;

  explicit GetConfig_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetConfig_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Request
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Request
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetConfig_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetConfig_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetConfig_Request_

// alias to use template instance with default allocator
using GetConfig_Request =
  rogilink_flex_interfaces::srv::GetConfig_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rogilink_flex_interfaces


#ifndef _WIN32
# define DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Response __attribute__((deprecated))
#else
# define DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Response __declspec(deprecated)
#endif

namespace rogilink_flex_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetConfig_Response_
{
  using Type = GetConfig_Response_<ContainerAllocator>;

  explicit GetConfig_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
    }
  }

  explicit GetConfig_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->path = "";
    }
  }

  // field types and members
  using _path_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _path_type path;

  // setters for named parameter idiom
  Type & set__path(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Response
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rogilink_flex_interfaces__srv__GetConfig_Response
    std::shared_ptr<rogilink_flex_interfaces::srv::GetConfig_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetConfig_Response_ & other) const
  {
    if (this->path != other.path) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetConfig_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetConfig_Response_

// alias to use template instance with default allocator
using GetConfig_Response =
  rogilink_flex_interfaces::srv::GetConfig_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rogilink_flex_interfaces

namespace rogilink_flex_interfaces
{

namespace srv
{

struct GetConfig
{
  using Request = rogilink_flex_interfaces::srv::GetConfig_Request;
  using Response = rogilink_flex_interfaces::srv::GetConfig_Response;
};

}  // namespace srv

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__GET_CONFIG__STRUCT_HPP_

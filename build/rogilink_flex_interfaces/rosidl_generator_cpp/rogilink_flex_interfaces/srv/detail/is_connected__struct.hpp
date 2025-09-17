// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rogilink_flex_interfaces:srv/IsConnected.idl
// generated code does not contain a copyright notice

#ifndef ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__STRUCT_HPP_
#define ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Request __attribute__((deprecated))
#else
# define DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Request __declspec(deprecated)
#endif

namespace rogilink_flex_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsConnected_Request_
{
  using Type = IsConnected_Request_<ContainerAllocator>;

  explicit IsConnected_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit IsConnected_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Request
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Request
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsConnected_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsConnected_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsConnected_Request_

// alias to use template instance with default allocator
using IsConnected_Request =
  rogilink_flex_interfaces::srv::IsConnected_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rogilink_flex_interfaces


#ifndef _WIN32
# define DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Response __attribute__((deprecated))
#else
# define DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Response __declspec(deprecated)
#endif

namespace rogilink_flex_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct IsConnected_Response_
{
  using Type = IsConnected_Response_<ContainerAllocator>;

  explicit IsConnected_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->connected = false;
      this->device_id = 0;
    }
  }

  explicit IsConnected_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->connected = false;
      this->device_id = 0;
    }
  }

  // field types and members
  using _connected_type =
    bool;
  _connected_type connected;
  using _device_id_type =
    uint8_t;
  _device_id_type device_id;

  // setters for named parameter idiom
  Type & set__connected(
    const bool & _arg)
  {
    this->connected = _arg;
    return *this;
  }
  Type & set__device_id(
    const uint8_t & _arg)
  {
    this->device_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Response
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rogilink_flex_interfaces__srv__IsConnected_Response
    std::shared_ptr<rogilink_flex_interfaces::srv::IsConnected_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IsConnected_Response_ & other) const
  {
    if (this->connected != other.connected) {
      return false;
    }
    if (this->device_id != other.device_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const IsConnected_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IsConnected_Response_

// alias to use template instance with default allocator
using IsConnected_Response =
  rogilink_flex_interfaces::srv::IsConnected_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rogilink_flex_interfaces

namespace rogilink_flex_interfaces
{

namespace srv
{

struct IsConnected
{
  using Request = rogilink_flex_interfaces::srv::IsConnected_Request;
  using Response = rogilink_flex_interfaces::srv::IsConnected_Response;
};

}  // namespace srv

}  // namespace rogilink_flex_interfaces

#endif  // ROGILINK_FLEX_INTERFACES__SRV__DETAIL__IS_CONNECTED__STRUCT_HPP_

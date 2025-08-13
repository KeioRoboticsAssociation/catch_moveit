// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_HPP_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dynamixel_controller__msg__DynamixelController __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_controller__msg__DynamixelController __declspec(deprecated)
#endif

namespace dynamixel_controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DynamixelController_
{
  using Type = DynamixelController_<ContainerAllocator>;

  explicit DynamixelController_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit DynamixelController_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  static constexpr uint8_t PING =
    1u;
  static constexpr uint8_t READ_DATA =
    2u;
  static constexpr uint8_t WRITE_DATA =
    3u;
  static constexpr uint8_t REG_WRITE =
    4u;
  static constexpr uint8_t ACTION =
    5u;
  static constexpr uint8_t FACTORY_RESET =
    6u;
  static constexpr uint8_t REBOOT =
    8u;
  static constexpr uint8_t SYNC_READ =
    130u;
  static constexpr uint8_t SYNC_WRITE =
    131u;
  static constexpr uint8_t BULK_READ =
    146u;
  static constexpr uint8_t BULK_WRITE =
    147u;
  static constexpr uint8_t MODEL_NUMBER =
    0u;
  static constexpr uint8_t MODEL_INFORMATION =
    2u;
  static constexpr uint8_t FIRMWARE_VERSION =
    6u;
  static constexpr uint8_t ID =
    7u;
  static constexpr uint8_t BAUD_RATE =
    8u;
  static constexpr uint8_t RETURN_DELAY_TIME =
    9u;
  static constexpr uint8_t DRIVE_MODE =
    10u;
  static constexpr uint8_t OPERATING_MODE =
    11u;
  static constexpr uint8_t SECONDARY_ID =
    12u;
  static constexpr uint8_t PROTOCOL_TYPE =
    13u;
  static constexpr uint8_t HOMING_OFFSET =
    20u;
  static constexpr uint8_t MOVING_THRESHOLD =
    24u;
  static constexpr uint8_t TEMPERATURE_LIMIT =
    31u;
  static constexpr uint8_t MAX_VOLTAGE_LIMIT =
    32u;
  static constexpr uint8_t MIN_VOLTAGE_LIMIT =
    34u;
  static constexpr uint8_t PWM_LIMIT =
    36u;
  static constexpr uint8_t CURRENT_LIMIT =
    38u;
  static constexpr uint8_t VELOCITY_LIMIT =
    40u;
  static constexpr uint8_t MAX_POSITION_LIMIT =
    44u;
  static constexpr uint8_t MIN_POSITION_LIMIT =
    48u;
  static constexpr uint8_t TORQUE_ENABLE =
    64u;
  static constexpr uint8_t LED =
    65u;
  static constexpr uint8_t STATUS_RETURN_LEVEL =
    68u;
  static constexpr uint8_t REGISTERED_INSTRUCTION =
    69u;
  static constexpr uint8_t HARDWARE_ERROR_STATUS =
    70u;
  static constexpr uint8_t VELOCITY_I_GAIN =
    76u;
  static constexpr uint8_t VELOCITY_P_GAIN =
    78u;
  static constexpr uint8_t POSITION_D_GAIN =
    80u;
  static constexpr uint8_t POSITION_I_GAIN =
    82u;
  static constexpr uint8_t POSITION_P_GAIN =
    84u;
  static constexpr uint8_t FEEDFORWARD_2_GAIN =
    88u;
  static constexpr uint8_t FEEDFORWARD_1_GAIN =
    90u;
  static constexpr uint8_t BUS_WATCHDOG =
    98u;
  static constexpr uint8_t GOAL_PWM =
    100u;
  static constexpr uint8_t GOAL_CURRENT =
    102u;
  static constexpr uint8_t GOAL_VELOCITY =
    104u;
  static constexpr uint8_t PROFILE_ACCELERATION =
    108u;
  static constexpr uint8_t PROFILE_VELOCITY =
    112u;
  static constexpr uint8_t GOAL_POSITION =
    116u;
  static constexpr uint8_t REALTIME_TICK =
    120u;
  static constexpr uint8_t MOVING =
    122u;
  static constexpr uint8_t PRESENT_PWM =
    124u;
  static constexpr uint8_t PRESENT_CURRENT =
    126u;
  static constexpr uint8_t PRESENT_VELOCITY =
    128u;
  static constexpr uint8_t PRESENT_POSITION =
    132u;
  static constexpr uint8_t PRESENT_INPUT_VOLTAGE =
    144u;
  static constexpr uint8_t PRESENT_TEMPERATURE =
    146u;

  // pointer types
  using RawPtr =
    dynamixel_controller::msg::DynamixelController_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_controller::msg::DynamixelController_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelController_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_controller::msg::DynamixelController_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelController
    std::shared_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_controller__msg__DynamixelController
    std::shared_ptr<dynamixel_controller::msg::DynamixelController_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DynamixelController_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const DynamixelController_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DynamixelController_

// alias to use template instance with default allocator
using DynamixelController =
  dynamixel_controller::msg::DynamixelController_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::READ_DATA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::WRITE_DATA;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::REG_WRITE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::ACTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::FACTORY_RESET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::REBOOT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::SYNC_READ;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::SYNC_WRITE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::BULK_READ;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::BULK_WRITE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MODEL_NUMBER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MODEL_INFORMATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::FIRMWARE_VERSION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::ID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::BAUD_RATE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::RETURN_DELAY_TIME;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::DRIVE_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::OPERATING_MODE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::SECONDARY_ID;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PROTOCOL_TYPE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::HOMING_OFFSET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MOVING_THRESHOLD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::TEMPERATURE_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MAX_VOLTAGE_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MIN_VOLTAGE_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PWM_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::CURRENT_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::VELOCITY_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MAX_POSITION_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MIN_POSITION_LIMIT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::TORQUE_ENABLE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::LED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::STATUS_RETURN_LEVEL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::REGISTERED_INSTRUCTION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::HARDWARE_ERROR_STATUS;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::VELOCITY_I_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::VELOCITY_P_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::POSITION_D_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::POSITION_I_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::POSITION_P_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::FEEDFORWARD_2_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::FEEDFORWARD_1_GAIN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::BUS_WATCHDOG;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::GOAL_PWM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::GOAL_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::GOAL_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PROFILE_ACCELERATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PROFILE_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::GOAL_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::REALTIME_TICK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::MOVING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_PWM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_CURRENT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_VELOCITY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_POSITION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_INPUT_VOLTAGE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t DynamixelController_<ContainerAllocator>::PRESENT_TEMPERATURE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace dynamixel_controller

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__STRUCT_HPP_

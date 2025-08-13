// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__FUNCTIONS_H_
#define DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dynamixel_controller/msg/rosidl_generator_c__visibility_control.h"

#include "dynamixel_controller/msg/detail/dynamixel_controller__struct.h"

/// Initialize msg/DynamixelController message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dynamixel_controller__msg__DynamixelController
 * )) before or use
 * dynamixel_controller__msg__DynamixelController__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__init(dynamixel_controller__msg__DynamixelController * msg);

/// Finalize msg/DynamixelController message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
void
dynamixel_controller__msg__DynamixelController__fini(dynamixel_controller__msg__DynamixelController * msg);

/// Create msg/DynamixelController message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dynamixel_controller__msg__DynamixelController__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
dynamixel_controller__msg__DynamixelController *
dynamixel_controller__msg__DynamixelController__create();

/// Destroy msg/DynamixelController message.
/**
 * It calls
 * dynamixel_controller__msg__DynamixelController__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
void
dynamixel_controller__msg__DynamixelController__destroy(dynamixel_controller__msg__DynamixelController * msg);

/// Check for msg/DynamixelController message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__are_equal(const dynamixel_controller__msg__DynamixelController * lhs, const dynamixel_controller__msg__DynamixelController * rhs);

/// Copy a msg/DynamixelController message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__copy(
  const dynamixel_controller__msg__DynamixelController * input,
  dynamixel_controller__msg__DynamixelController * output);

/// Initialize array of msg/DynamixelController messages.
/**
 * It allocates the memory for the number of elements and calls
 * dynamixel_controller__msg__DynamixelController__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__Sequence__init(dynamixel_controller__msg__DynamixelController__Sequence * array, size_t size);

/// Finalize array of msg/DynamixelController messages.
/**
 * It calls
 * dynamixel_controller__msg__DynamixelController__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
void
dynamixel_controller__msg__DynamixelController__Sequence__fini(dynamixel_controller__msg__DynamixelController__Sequence * array);

/// Create array of msg/DynamixelController messages.
/**
 * It allocates the memory for the array and calls
 * dynamixel_controller__msg__DynamixelController__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
dynamixel_controller__msg__DynamixelController__Sequence *
dynamixel_controller__msg__DynamixelController__Sequence__create(size_t size);

/// Destroy array of msg/DynamixelController messages.
/**
 * It calls
 * dynamixel_controller__msg__DynamixelController__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
void
dynamixel_controller__msg__DynamixelController__Sequence__destroy(dynamixel_controller__msg__DynamixelController__Sequence * array);

/// Check for msg/DynamixelController message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__Sequence__are_equal(const dynamixel_controller__msg__DynamixelController__Sequence * lhs, const dynamixel_controller__msg__DynamixelController__Sequence * rhs);

/// Copy an array of msg/DynamixelController messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dynamixel_controller
bool
dynamixel_controller__msg__DynamixelController__Sequence__copy(
  const dynamixel_controller__msg__DynamixelController__Sequence * input,
  dynamixel_controller__msg__DynamixelController__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_CONTROLLER__MSG__DETAIL__DYNAMIXEL_CONTROLLER__FUNCTIONS_H_

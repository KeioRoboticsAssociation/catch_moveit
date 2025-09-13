// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorCommand.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__init(stm32_mavlink_interface__msg__RobomasterMotorCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(msg);
    return false;
  }
  // motor_id
  // control_mode
  // target_current_ma
  // target_velocity_rps
  // target_position_rad
  // enabled
  // reset_watchdog
  // emergency_stop
  return true;
}

void
stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(stm32_mavlink_interface__msg__RobomasterMotorCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_id
  // control_mode
  // target_current_ma
  // target_velocity_rps
  // target_position_rad
  // enabled
  // reset_watchdog
  // emergency_stop
}

bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__are_equal(const stm32_mavlink_interface__msg__RobomasterMotorCommand * lhs, const stm32_mavlink_interface__msg__RobomasterMotorCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // target_current_ma
  if (lhs->target_current_ma != rhs->target_current_ma) {
    return false;
  }
  // target_velocity_rps
  if (lhs->target_velocity_rps != rhs->target_velocity_rps) {
    return false;
  }
  // target_position_rad
  if (lhs->target_position_rad != rhs->target_position_rad) {
    return false;
  }
  // enabled
  if (lhs->enabled != rhs->enabled) {
    return false;
  }
  // reset_watchdog
  if (lhs->reset_watchdog != rhs->reset_watchdog) {
    return false;
  }
  // emergency_stop
  if (lhs->emergency_stop != rhs->emergency_stop) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__copy(
  const stm32_mavlink_interface__msg__RobomasterMotorCommand * input,
  stm32_mavlink_interface__msg__RobomasterMotorCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // control_mode
  output->control_mode = input->control_mode;
  // target_current_ma
  output->target_current_ma = input->target_current_ma;
  // target_velocity_rps
  output->target_velocity_rps = input->target_velocity_rps;
  // target_position_rad
  output->target_position_rad = input->target_position_rad;
  // enabled
  output->enabled = input->enabled;
  // reset_watchdog
  output->reset_watchdog = input->reset_watchdog;
  // emergency_stop
  output->emergency_stop = input->emergency_stop;
  return true;
}

stm32_mavlink_interface__msg__RobomasterMotorCommand *
stm32_mavlink_interface__msg__RobomasterMotorCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorCommand * msg = (stm32_mavlink_interface__msg__RobomasterMotorCommand *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__RobomasterMotorCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__msg__RobomasterMotorCommand));
  bool success = stm32_mavlink_interface__msg__RobomasterMotorCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__msg__RobomasterMotorCommand__destroy(stm32_mavlink_interface__msg__RobomasterMotorCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__init(stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorCommand * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__msg__RobomasterMotorCommand *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__msg__RobomasterMotorCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__msg__RobomasterMotorCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__fini(stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence *
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * array = (stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__destroy(stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__are_equal(const stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * lhs, const stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__msg__RobomasterMotorCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence__copy(
  const stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * input,
  stm32_mavlink_interface__msg__RobomasterMotorCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__msg__RobomasterMotorCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__msg__RobomasterMotorCommand * data =
      (stm32_mavlink_interface__msg__RobomasterMotorCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__msg__RobomasterMotorCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__msg__RobomasterMotorCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__msg__RobomasterMotorCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

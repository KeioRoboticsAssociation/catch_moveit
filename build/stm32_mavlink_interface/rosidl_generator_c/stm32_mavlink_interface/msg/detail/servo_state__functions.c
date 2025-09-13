// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:msg/ServoState.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/servo_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
stm32_mavlink_interface__msg__ServoState__init(stm32_mavlink_interface__msg__ServoState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stm32_mavlink_interface__msg__ServoState__fini(msg);
    return false;
  }
  // servo_id
  // current_angle_deg
  // target_angle_deg
  // pulse_us
  // enabled
  // status
  // error_count
  return true;
}

void
stm32_mavlink_interface__msg__ServoState__fini(stm32_mavlink_interface__msg__ServoState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // servo_id
  // current_angle_deg
  // target_angle_deg
  // pulse_us
  // enabled
  // status
  // error_count
}

bool
stm32_mavlink_interface__msg__ServoState__are_equal(const stm32_mavlink_interface__msg__ServoState * lhs, const stm32_mavlink_interface__msg__ServoState * rhs)
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
  // servo_id
  if (lhs->servo_id != rhs->servo_id) {
    return false;
  }
  // current_angle_deg
  if (lhs->current_angle_deg != rhs->current_angle_deg) {
    return false;
  }
  // target_angle_deg
  if (lhs->target_angle_deg != rhs->target_angle_deg) {
    return false;
  }
  // pulse_us
  if (lhs->pulse_us != rhs->pulse_us) {
    return false;
  }
  // enabled
  if (lhs->enabled != rhs->enabled) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // error_count
  if (lhs->error_count != rhs->error_count) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__msg__ServoState__copy(
  const stm32_mavlink_interface__msg__ServoState * input,
  stm32_mavlink_interface__msg__ServoState * output)
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
  // servo_id
  output->servo_id = input->servo_id;
  // current_angle_deg
  output->current_angle_deg = input->current_angle_deg;
  // target_angle_deg
  output->target_angle_deg = input->target_angle_deg;
  // pulse_us
  output->pulse_us = input->pulse_us;
  // enabled
  output->enabled = input->enabled;
  // status
  output->status = input->status;
  // error_count
  output->error_count = input->error_count;
  return true;
}

stm32_mavlink_interface__msg__ServoState *
stm32_mavlink_interface__msg__ServoState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoState * msg = (stm32_mavlink_interface__msg__ServoState *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__ServoState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__msg__ServoState));
  bool success = stm32_mavlink_interface__msg__ServoState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__msg__ServoState__destroy(stm32_mavlink_interface__msg__ServoState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__msg__ServoState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__msg__ServoState__Sequence__init(stm32_mavlink_interface__msg__ServoState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoState * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__msg__ServoState *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__msg__ServoState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__msg__ServoState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__msg__ServoState__fini(&data[i - 1]);
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
stm32_mavlink_interface__msg__ServoState__Sequence__fini(stm32_mavlink_interface__msg__ServoState__Sequence * array)
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
      stm32_mavlink_interface__msg__ServoState__fini(&array->data[i]);
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

stm32_mavlink_interface__msg__ServoState__Sequence *
stm32_mavlink_interface__msg__ServoState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoState__Sequence * array = (stm32_mavlink_interface__msg__ServoState__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__ServoState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__msg__ServoState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__msg__ServoState__Sequence__destroy(stm32_mavlink_interface__msg__ServoState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__msg__ServoState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__msg__ServoState__Sequence__are_equal(const stm32_mavlink_interface__msg__ServoState__Sequence * lhs, const stm32_mavlink_interface__msg__ServoState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__msg__ServoState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__msg__ServoState__Sequence__copy(
  const stm32_mavlink_interface__msg__ServoState__Sequence * input,
  stm32_mavlink_interface__msg__ServoState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__msg__ServoState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__msg__ServoState * data =
      (stm32_mavlink_interface__msg__ServoState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__msg__ServoState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__msg__ServoState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__msg__ServoState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

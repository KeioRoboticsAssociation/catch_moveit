// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:msg/ServoCommand.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/servo_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
stm32_mavlink_interface__msg__ServoCommand__init(stm32_mavlink_interface__msg__ServoCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stm32_mavlink_interface__msg__ServoCommand__fini(msg);
    return false;
  }
  // servo_id
  // angle_deg
  // pulse_us
  // enable
  return true;
}

void
stm32_mavlink_interface__msg__ServoCommand__fini(stm32_mavlink_interface__msg__ServoCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // servo_id
  // angle_deg
  // pulse_us
  // enable
}

bool
stm32_mavlink_interface__msg__ServoCommand__are_equal(const stm32_mavlink_interface__msg__ServoCommand * lhs, const stm32_mavlink_interface__msg__ServoCommand * rhs)
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
  // angle_deg
  if (lhs->angle_deg != rhs->angle_deg) {
    return false;
  }
  // pulse_us
  if (lhs->pulse_us != rhs->pulse_us) {
    return false;
  }
  // enable
  if (lhs->enable != rhs->enable) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__msg__ServoCommand__copy(
  const stm32_mavlink_interface__msg__ServoCommand * input,
  stm32_mavlink_interface__msg__ServoCommand * output)
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
  // angle_deg
  output->angle_deg = input->angle_deg;
  // pulse_us
  output->pulse_us = input->pulse_us;
  // enable
  output->enable = input->enable;
  return true;
}

stm32_mavlink_interface__msg__ServoCommand *
stm32_mavlink_interface__msg__ServoCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoCommand * msg = (stm32_mavlink_interface__msg__ServoCommand *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__ServoCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__msg__ServoCommand));
  bool success = stm32_mavlink_interface__msg__ServoCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__msg__ServoCommand__destroy(stm32_mavlink_interface__msg__ServoCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__msg__ServoCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__msg__ServoCommand__Sequence__init(stm32_mavlink_interface__msg__ServoCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoCommand * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__msg__ServoCommand *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__msg__ServoCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__msg__ServoCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__msg__ServoCommand__fini(&data[i - 1]);
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
stm32_mavlink_interface__msg__ServoCommand__Sequence__fini(stm32_mavlink_interface__msg__ServoCommand__Sequence * array)
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
      stm32_mavlink_interface__msg__ServoCommand__fini(&array->data[i]);
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

stm32_mavlink_interface__msg__ServoCommand__Sequence *
stm32_mavlink_interface__msg__ServoCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__ServoCommand__Sequence * array = (stm32_mavlink_interface__msg__ServoCommand__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__ServoCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__msg__ServoCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__msg__ServoCommand__Sequence__destroy(stm32_mavlink_interface__msg__ServoCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__msg__ServoCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__msg__ServoCommand__Sequence__are_equal(const stm32_mavlink_interface__msg__ServoCommand__Sequence * lhs, const stm32_mavlink_interface__msg__ServoCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__msg__ServoCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__msg__ServoCommand__Sequence__copy(
  const stm32_mavlink_interface__msg__ServoCommand__Sequence * input,
  stm32_mavlink_interface__msg__ServoCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__msg__ServoCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__msg__ServoCommand * data =
      (stm32_mavlink_interface__msg__ServoCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__msg__ServoCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__msg__ServoCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__msg__ServoCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

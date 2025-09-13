// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:msg/EncoderConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/encoder_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
stm32_mavlink_interface__msg__EncoderConfig__init(stm32_mavlink_interface__msg__EncoderConfig * msg)
{
  if (!msg) {
    return false;
  }
  // encoder_id
  // cpr
  // invert_a
  // invert_b
  // use_z
  // watchdog_timeout_ms
  // offset_counts
  // wrap_around
  return true;
}

void
stm32_mavlink_interface__msg__EncoderConfig__fini(stm32_mavlink_interface__msg__EncoderConfig * msg)
{
  if (!msg) {
    return;
  }
  // encoder_id
  // cpr
  // invert_a
  // invert_b
  // use_z
  // watchdog_timeout_ms
  // offset_counts
  // wrap_around
}

bool
stm32_mavlink_interface__msg__EncoderConfig__are_equal(const stm32_mavlink_interface__msg__EncoderConfig * lhs, const stm32_mavlink_interface__msg__EncoderConfig * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // encoder_id
  if (lhs->encoder_id != rhs->encoder_id) {
    return false;
  }
  // cpr
  if (lhs->cpr != rhs->cpr) {
    return false;
  }
  // invert_a
  if (lhs->invert_a != rhs->invert_a) {
    return false;
  }
  // invert_b
  if (lhs->invert_b != rhs->invert_b) {
    return false;
  }
  // use_z
  if (lhs->use_z != rhs->use_z) {
    return false;
  }
  // watchdog_timeout_ms
  if (lhs->watchdog_timeout_ms != rhs->watchdog_timeout_ms) {
    return false;
  }
  // offset_counts
  if (lhs->offset_counts != rhs->offset_counts) {
    return false;
  }
  // wrap_around
  if (lhs->wrap_around != rhs->wrap_around) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__msg__EncoderConfig__copy(
  const stm32_mavlink_interface__msg__EncoderConfig * input,
  stm32_mavlink_interface__msg__EncoderConfig * output)
{
  if (!input || !output) {
    return false;
  }
  // encoder_id
  output->encoder_id = input->encoder_id;
  // cpr
  output->cpr = input->cpr;
  // invert_a
  output->invert_a = input->invert_a;
  // invert_b
  output->invert_b = input->invert_b;
  // use_z
  output->use_z = input->use_z;
  // watchdog_timeout_ms
  output->watchdog_timeout_ms = input->watchdog_timeout_ms;
  // offset_counts
  output->offset_counts = input->offset_counts;
  // wrap_around
  output->wrap_around = input->wrap_around;
  return true;
}

stm32_mavlink_interface__msg__EncoderConfig *
stm32_mavlink_interface__msg__EncoderConfig__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__EncoderConfig * msg = (stm32_mavlink_interface__msg__EncoderConfig *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__EncoderConfig), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__msg__EncoderConfig));
  bool success = stm32_mavlink_interface__msg__EncoderConfig__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__msg__EncoderConfig__destroy(stm32_mavlink_interface__msg__EncoderConfig * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__msg__EncoderConfig__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__msg__EncoderConfig__Sequence__init(stm32_mavlink_interface__msg__EncoderConfig__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__EncoderConfig * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__msg__EncoderConfig *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__msg__EncoderConfig), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__msg__EncoderConfig__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__msg__EncoderConfig__fini(&data[i - 1]);
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
stm32_mavlink_interface__msg__EncoderConfig__Sequence__fini(stm32_mavlink_interface__msg__EncoderConfig__Sequence * array)
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
      stm32_mavlink_interface__msg__EncoderConfig__fini(&array->data[i]);
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

stm32_mavlink_interface__msg__EncoderConfig__Sequence *
stm32_mavlink_interface__msg__EncoderConfig__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__EncoderConfig__Sequence * array = (stm32_mavlink_interface__msg__EncoderConfig__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__EncoderConfig__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__msg__EncoderConfig__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__msg__EncoderConfig__Sequence__destroy(stm32_mavlink_interface__msg__EncoderConfig__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__msg__EncoderConfig__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__msg__EncoderConfig__Sequence__are_equal(const stm32_mavlink_interface__msg__EncoderConfig__Sequence * lhs, const stm32_mavlink_interface__msg__EncoderConfig__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__msg__EncoderConfig__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__msg__EncoderConfig__Sequence__copy(
  const stm32_mavlink_interface__msg__EncoderConfig__Sequence * input,
  stm32_mavlink_interface__msg__EncoderConfig__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__msg__EncoderConfig);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__msg__EncoderConfig * data =
      (stm32_mavlink_interface__msg__EncoderConfig *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__msg__EncoderConfig__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__msg__EncoderConfig__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__msg__EncoderConfig__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

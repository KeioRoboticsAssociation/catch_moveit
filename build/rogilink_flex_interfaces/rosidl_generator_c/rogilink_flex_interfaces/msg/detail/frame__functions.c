// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rogilink_flex_interfaces:msg/Frame.idl
// generated code does not contain a copyright notice
#include "rogilink_flex_interfaces/msg/detail/frame__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `topic`
#include "rosidl_runtime_c/string_functions.h"

bool
rogilink_flex_interfaces__msg__Frame__init(rogilink_flex_interfaces__msg__Frame * msg)
{
  if (!msg) {
    return false;
  }
  // frame_id
  msg->frame_id = 0;
  // device_id
  msg->device_id = 0;
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    rogilink_flex_interfaces__msg__Frame__fini(msg);
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__init(&msg->topic)) {
    rogilink_flex_interfaces__msg__Frame__fini(msg);
    return false;
  }
  return true;
}

void
rogilink_flex_interfaces__msg__Frame__fini(rogilink_flex_interfaces__msg__Frame * msg)
{
  if (!msg) {
    return;
  }
  // frame_id
  // device_id
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
  // topic
  rosidl_runtime_c__String__fini(&msg->topic);
}

bool
rogilink_flex_interfaces__msg__Frame__are_equal(const rogilink_flex_interfaces__msg__Frame * lhs, const rogilink_flex_interfaces__msg__Frame * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame_id
  if (lhs->frame_id != rhs->frame_id) {
    return false;
  }
  // device_id
  if (lhs->device_id != rhs->device_id) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic), &(rhs->topic)))
  {
    return false;
  }
  return true;
}

bool
rogilink_flex_interfaces__msg__Frame__copy(
  const rogilink_flex_interfaces__msg__Frame * input,
  rogilink_flex_interfaces__msg__Frame * output)
{
  if (!input || !output) {
    return false;
  }
  // frame_id
  output->frame_id = input->frame_id;
  // device_id
  output->device_id = input->device_id;
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__copy(
      &(input->topic), &(output->topic)))
  {
    return false;
  }
  return true;
}

rogilink_flex_interfaces__msg__Frame *
rogilink_flex_interfaces__msg__Frame__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__msg__Frame * msg = (rogilink_flex_interfaces__msg__Frame *)allocator.allocate(sizeof(rogilink_flex_interfaces__msg__Frame), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rogilink_flex_interfaces__msg__Frame));
  bool success = rogilink_flex_interfaces__msg__Frame__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rogilink_flex_interfaces__msg__Frame__destroy(rogilink_flex_interfaces__msg__Frame * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rogilink_flex_interfaces__msg__Frame__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rogilink_flex_interfaces__msg__Frame__Sequence__init(rogilink_flex_interfaces__msg__Frame__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__msg__Frame * data = NULL;

  if (size) {
    data = (rogilink_flex_interfaces__msg__Frame *)allocator.zero_allocate(size, sizeof(rogilink_flex_interfaces__msg__Frame), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rogilink_flex_interfaces__msg__Frame__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rogilink_flex_interfaces__msg__Frame__fini(&data[i - 1]);
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
rogilink_flex_interfaces__msg__Frame__Sequence__fini(rogilink_flex_interfaces__msg__Frame__Sequence * array)
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
      rogilink_flex_interfaces__msg__Frame__fini(&array->data[i]);
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

rogilink_flex_interfaces__msg__Frame__Sequence *
rogilink_flex_interfaces__msg__Frame__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__msg__Frame__Sequence * array = (rogilink_flex_interfaces__msg__Frame__Sequence *)allocator.allocate(sizeof(rogilink_flex_interfaces__msg__Frame__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rogilink_flex_interfaces__msg__Frame__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rogilink_flex_interfaces__msg__Frame__Sequence__destroy(rogilink_flex_interfaces__msg__Frame__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rogilink_flex_interfaces__msg__Frame__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rogilink_flex_interfaces__msg__Frame__Sequence__are_equal(const rogilink_flex_interfaces__msg__Frame__Sequence * lhs, const rogilink_flex_interfaces__msg__Frame__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rogilink_flex_interfaces__msg__Frame__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rogilink_flex_interfaces__msg__Frame__Sequence__copy(
  const rogilink_flex_interfaces__msg__Frame__Sequence * input,
  rogilink_flex_interfaces__msg__Frame__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rogilink_flex_interfaces__msg__Frame);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rogilink_flex_interfaces__msg__Frame * data =
      (rogilink_flex_interfaces__msg__Frame *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rogilink_flex_interfaces__msg__Frame__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rogilink_flex_interfaces__msg__Frame__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rogilink_flex_interfaces__msg__Frame__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

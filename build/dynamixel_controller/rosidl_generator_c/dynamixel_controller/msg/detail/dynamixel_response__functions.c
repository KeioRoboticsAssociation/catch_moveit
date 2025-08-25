// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_controller:msg/DynamixelResponse.idl
// generated code does not contain a copyright notice
#include "dynamixel_controller/msg/detail/dynamixel_response__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ids`
// Member `error`
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dynamixel_controller__msg__DynamixelResponse__init(dynamixel_controller__msg__DynamixelResponse * msg)
{
  if (!msg) {
    return false;
  }
  // command
  // ids
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->ids, 0)) {
    dynamixel_controller__msg__DynamixelResponse__fini(msg);
    return false;
  }
  // error
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->error, 0)) {
    dynamixel_controller__msg__DynamixelResponse__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    dynamixel_controller__msg__DynamixelResponse__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_controller__msg__DynamixelResponse__fini(dynamixel_controller__msg__DynamixelResponse * msg)
{
  if (!msg) {
    return;
  }
  // command
  // ids
  rosidl_runtime_c__uint8__Sequence__fini(&msg->ids);
  // error
  rosidl_runtime_c__uint8__Sequence__fini(&msg->error);
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
dynamixel_controller__msg__DynamixelResponse__are_equal(const dynamixel_controller__msg__DynamixelResponse * lhs, const dynamixel_controller__msg__DynamixelResponse * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (lhs->command != rhs->command) {
    return false;
  }
  // ids
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->ids), &(rhs->ids)))
  {
    return false;
  }
  // error
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->error), &(rhs->error)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_controller__msg__DynamixelResponse__copy(
  const dynamixel_controller__msg__DynamixelResponse * input,
  dynamixel_controller__msg__DynamixelResponse * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  output->command = input->command;
  // ids
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->ids), &(output->ids)))
  {
    return false;
  }
  // error
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->error), &(output->error)))
  {
    return false;
  }
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

dynamixel_controller__msg__DynamixelResponse *
dynamixel_controller__msg__DynamixelResponse__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelResponse * msg = (dynamixel_controller__msg__DynamixelResponse *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelResponse), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_controller__msg__DynamixelResponse));
  bool success = dynamixel_controller__msg__DynamixelResponse__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_controller__msg__DynamixelResponse__destroy(dynamixel_controller__msg__DynamixelResponse * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_controller__msg__DynamixelResponse__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_controller__msg__DynamixelResponse__Sequence__init(dynamixel_controller__msg__DynamixelResponse__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelResponse * data = NULL;

  if (size) {
    data = (dynamixel_controller__msg__DynamixelResponse *)allocator.zero_allocate(size, sizeof(dynamixel_controller__msg__DynamixelResponse), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_controller__msg__DynamixelResponse__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_controller__msg__DynamixelResponse__fini(&data[i - 1]);
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
dynamixel_controller__msg__DynamixelResponse__Sequence__fini(dynamixel_controller__msg__DynamixelResponse__Sequence * array)
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
      dynamixel_controller__msg__DynamixelResponse__fini(&array->data[i]);
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

dynamixel_controller__msg__DynamixelResponse__Sequence *
dynamixel_controller__msg__DynamixelResponse__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelResponse__Sequence * array = (dynamixel_controller__msg__DynamixelResponse__Sequence *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelResponse__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_controller__msg__DynamixelResponse__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_controller__msg__DynamixelResponse__Sequence__destroy(dynamixel_controller__msg__DynamixelResponse__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_controller__msg__DynamixelResponse__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_controller__msg__DynamixelResponse__Sequence__are_equal(const dynamixel_controller__msg__DynamixelResponse__Sequence * lhs, const dynamixel_controller__msg__DynamixelResponse__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelResponse__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_controller__msg__DynamixelResponse__Sequence__copy(
  const dynamixel_controller__msg__DynamixelResponse__Sequence * input,
  dynamixel_controller__msg__DynamixelResponse__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_controller__msg__DynamixelResponse);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_controller__msg__DynamixelResponse * data =
      (dynamixel_controller__msg__DynamixelResponse *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_controller__msg__DynamixelResponse__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_controller__msg__DynamixelResponse__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelResponse__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

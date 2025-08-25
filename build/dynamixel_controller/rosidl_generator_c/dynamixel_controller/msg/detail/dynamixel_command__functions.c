// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_controller:msg/DynamixelCommand.idl
// generated code does not contain a copyright notice
#include "dynamixel_controller/msg/detail/dynamixel_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ids`
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dynamixel_controller__msg__DynamixelCommand__init(dynamixel_controller__msg__DynamixelCommand * msg)
{
  if (!msg) {
    return false;
  }
  // command
  // ids
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->ids, 0)) {
    dynamixel_controller__msg__DynamixelCommand__fini(msg);
    return false;
  }
  // address
  // length
  // data
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->data, 0)) {
    dynamixel_controller__msg__DynamixelCommand__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_controller__msg__DynamixelCommand__fini(dynamixel_controller__msg__DynamixelCommand * msg)
{
  if (!msg) {
    return;
  }
  // command
  // ids
  rosidl_runtime_c__uint8__Sequence__fini(&msg->ids);
  // address
  // length
  // data
  rosidl_runtime_c__uint8__Sequence__fini(&msg->data);
}

bool
dynamixel_controller__msg__DynamixelCommand__are_equal(const dynamixel_controller__msg__DynamixelCommand * lhs, const dynamixel_controller__msg__DynamixelCommand * rhs)
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
  // address
  if (lhs->address != rhs->address) {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
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
dynamixel_controller__msg__DynamixelCommand__copy(
  const dynamixel_controller__msg__DynamixelCommand * input,
  dynamixel_controller__msg__DynamixelCommand * output)
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
  // address
  output->address = input->address;
  // length
  output->length = input->length;
  // data
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

dynamixel_controller__msg__DynamixelCommand *
dynamixel_controller__msg__DynamixelCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelCommand * msg = (dynamixel_controller__msg__DynamixelCommand *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_controller__msg__DynamixelCommand));
  bool success = dynamixel_controller__msg__DynamixelCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_controller__msg__DynamixelCommand__destroy(dynamixel_controller__msg__DynamixelCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_controller__msg__DynamixelCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_controller__msg__DynamixelCommand__Sequence__init(dynamixel_controller__msg__DynamixelCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelCommand * data = NULL;

  if (size) {
    data = (dynamixel_controller__msg__DynamixelCommand *)allocator.zero_allocate(size, sizeof(dynamixel_controller__msg__DynamixelCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_controller__msg__DynamixelCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_controller__msg__DynamixelCommand__fini(&data[i - 1]);
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
dynamixel_controller__msg__DynamixelCommand__Sequence__fini(dynamixel_controller__msg__DynamixelCommand__Sequence * array)
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
      dynamixel_controller__msg__DynamixelCommand__fini(&array->data[i]);
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

dynamixel_controller__msg__DynamixelCommand__Sequence *
dynamixel_controller__msg__DynamixelCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelCommand__Sequence * array = (dynamixel_controller__msg__DynamixelCommand__Sequence *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_controller__msg__DynamixelCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_controller__msg__DynamixelCommand__Sequence__destroy(dynamixel_controller__msg__DynamixelCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_controller__msg__DynamixelCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_controller__msg__DynamixelCommand__Sequence__are_equal(const dynamixel_controller__msg__DynamixelCommand__Sequence * lhs, const dynamixel_controller__msg__DynamixelCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_controller__msg__DynamixelCommand__Sequence__copy(
  const dynamixel_controller__msg__DynamixelCommand__Sequence * input,
  dynamixel_controller__msg__DynamixelCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_controller__msg__DynamixelCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_controller__msg__DynamixelCommand * data =
      (dynamixel_controller__msg__DynamixelCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_controller__msg__DynamixelCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_controller__msg__DynamixelCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

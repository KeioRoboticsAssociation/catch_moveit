// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_controller:msg/DynamixelController.idl
// generated code does not contain a copyright notice
#include "dynamixel_controller/msg/detail/dynamixel_controller__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dynamixel_controller__msg__DynamixelController__init(dynamixel_controller__msg__DynamixelController * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
dynamixel_controller__msg__DynamixelController__fini(dynamixel_controller__msg__DynamixelController * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
dynamixel_controller__msg__DynamixelController__are_equal(const dynamixel_controller__msg__DynamixelController * lhs, const dynamixel_controller__msg__DynamixelController * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
dynamixel_controller__msg__DynamixelController__copy(
  const dynamixel_controller__msg__DynamixelController * input,
  dynamixel_controller__msg__DynamixelController * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

dynamixel_controller__msg__DynamixelController *
dynamixel_controller__msg__DynamixelController__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelController * msg = (dynamixel_controller__msg__DynamixelController *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelController), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_controller__msg__DynamixelController));
  bool success = dynamixel_controller__msg__DynamixelController__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_controller__msg__DynamixelController__destroy(dynamixel_controller__msg__DynamixelController * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_controller__msg__DynamixelController__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_controller__msg__DynamixelController__Sequence__init(dynamixel_controller__msg__DynamixelController__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelController * data = NULL;

  if (size) {
    data = (dynamixel_controller__msg__DynamixelController *)allocator.zero_allocate(size, sizeof(dynamixel_controller__msg__DynamixelController), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_controller__msg__DynamixelController__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_controller__msg__DynamixelController__fini(&data[i - 1]);
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
dynamixel_controller__msg__DynamixelController__Sequence__fini(dynamixel_controller__msg__DynamixelController__Sequence * array)
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
      dynamixel_controller__msg__DynamixelController__fini(&array->data[i]);
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

dynamixel_controller__msg__DynamixelController__Sequence *
dynamixel_controller__msg__DynamixelController__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_controller__msg__DynamixelController__Sequence * array = (dynamixel_controller__msg__DynamixelController__Sequence *)allocator.allocate(sizeof(dynamixel_controller__msg__DynamixelController__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_controller__msg__DynamixelController__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_controller__msg__DynamixelController__Sequence__destroy(dynamixel_controller__msg__DynamixelController__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_controller__msg__DynamixelController__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_controller__msg__DynamixelController__Sequence__are_equal(const dynamixel_controller__msg__DynamixelController__Sequence * lhs, const dynamixel_controller__msg__DynamixelController__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelController__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_controller__msg__DynamixelController__Sequence__copy(
  const dynamixel_controller__msg__DynamixelController__Sequence * input,
  dynamixel_controller__msg__DynamixelController__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_controller__msg__DynamixelController);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_controller__msg__DynamixelController * data =
      (dynamixel_controller__msg__DynamixelController *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_controller__msg__DynamixelController__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_controller__msg__DynamixelController__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_controller__msg__DynamixelController__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

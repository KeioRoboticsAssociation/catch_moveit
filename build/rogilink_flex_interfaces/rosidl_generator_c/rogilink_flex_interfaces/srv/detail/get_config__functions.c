// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rogilink_flex_interfaces:srv/GetConfig.idl
// generated code does not contain a copyright notice
#include "rogilink_flex_interfaces/srv/detail/get_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
rogilink_flex_interfaces__srv__GetConfig_Request__init(rogilink_flex_interfaces__srv__GetConfig_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
rogilink_flex_interfaces__srv__GetConfig_Request__fini(rogilink_flex_interfaces__srv__GetConfig_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
rogilink_flex_interfaces__srv__GetConfig_Request__are_equal(const rogilink_flex_interfaces__srv__GetConfig_Request * lhs, const rogilink_flex_interfaces__srv__GetConfig_Request * rhs)
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
rogilink_flex_interfaces__srv__GetConfig_Request__copy(
  const rogilink_flex_interfaces__srv__GetConfig_Request * input,
  rogilink_flex_interfaces__srv__GetConfig_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

rogilink_flex_interfaces__srv__GetConfig_Request *
rogilink_flex_interfaces__srv__GetConfig_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Request * msg = (rogilink_flex_interfaces__srv__GetConfig_Request *)allocator.allocate(sizeof(rogilink_flex_interfaces__srv__GetConfig_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rogilink_flex_interfaces__srv__GetConfig_Request));
  bool success = rogilink_flex_interfaces__srv__GetConfig_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rogilink_flex_interfaces__srv__GetConfig_Request__destroy(rogilink_flex_interfaces__srv__GetConfig_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rogilink_flex_interfaces__srv__GetConfig_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__init(rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Request * data = NULL;

  if (size) {
    data = (rogilink_flex_interfaces__srv__GetConfig_Request *)allocator.zero_allocate(size, sizeof(rogilink_flex_interfaces__srv__GetConfig_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rogilink_flex_interfaces__srv__GetConfig_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rogilink_flex_interfaces__srv__GetConfig_Request__fini(&data[i - 1]);
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
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__fini(rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * array)
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
      rogilink_flex_interfaces__srv__GetConfig_Request__fini(&array->data[i]);
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

rogilink_flex_interfaces__srv__GetConfig_Request__Sequence *
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * array = (rogilink_flex_interfaces__srv__GetConfig_Request__Sequence *)allocator.allocate(sizeof(rogilink_flex_interfaces__srv__GetConfig_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__destroy(rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__are_equal(const rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * lhs, const rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rogilink_flex_interfaces__srv__GetConfig_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rogilink_flex_interfaces__srv__GetConfig_Request__Sequence__copy(
  const rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * input,
  rogilink_flex_interfaces__srv__GetConfig_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rogilink_flex_interfaces__srv__GetConfig_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rogilink_flex_interfaces__srv__GetConfig_Request * data =
      (rogilink_flex_interfaces__srv__GetConfig_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rogilink_flex_interfaces__srv__GetConfig_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rogilink_flex_interfaces__srv__GetConfig_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rogilink_flex_interfaces__srv__GetConfig_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `path`
#include "rosidl_runtime_c/string_functions.h"

bool
rogilink_flex_interfaces__srv__GetConfig_Response__init(rogilink_flex_interfaces__srv__GetConfig_Response * msg)
{
  if (!msg) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__init(&msg->path)) {
    rogilink_flex_interfaces__srv__GetConfig_Response__fini(msg);
    return false;
  }
  return true;
}

void
rogilink_flex_interfaces__srv__GetConfig_Response__fini(rogilink_flex_interfaces__srv__GetConfig_Response * msg)
{
  if (!msg) {
    return;
  }
  // path
  rosidl_runtime_c__String__fini(&msg->path);
}

bool
rogilink_flex_interfaces__srv__GetConfig_Response__are_equal(const rogilink_flex_interfaces__srv__GetConfig_Response * lhs, const rogilink_flex_interfaces__srv__GetConfig_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->path), &(rhs->path)))
  {
    return false;
  }
  return true;
}

bool
rogilink_flex_interfaces__srv__GetConfig_Response__copy(
  const rogilink_flex_interfaces__srv__GetConfig_Response * input,
  rogilink_flex_interfaces__srv__GetConfig_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // path
  if (!rosidl_runtime_c__String__copy(
      &(input->path), &(output->path)))
  {
    return false;
  }
  return true;
}

rogilink_flex_interfaces__srv__GetConfig_Response *
rogilink_flex_interfaces__srv__GetConfig_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Response * msg = (rogilink_flex_interfaces__srv__GetConfig_Response *)allocator.allocate(sizeof(rogilink_flex_interfaces__srv__GetConfig_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rogilink_flex_interfaces__srv__GetConfig_Response));
  bool success = rogilink_flex_interfaces__srv__GetConfig_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rogilink_flex_interfaces__srv__GetConfig_Response__destroy(rogilink_flex_interfaces__srv__GetConfig_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rogilink_flex_interfaces__srv__GetConfig_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__init(rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Response * data = NULL;

  if (size) {
    data = (rogilink_flex_interfaces__srv__GetConfig_Response *)allocator.zero_allocate(size, sizeof(rogilink_flex_interfaces__srv__GetConfig_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rogilink_flex_interfaces__srv__GetConfig_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rogilink_flex_interfaces__srv__GetConfig_Response__fini(&data[i - 1]);
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
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__fini(rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * array)
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
      rogilink_flex_interfaces__srv__GetConfig_Response__fini(&array->data[i]);
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

rogilink_flex_interfaces__srv__GetConfig_Response__Sequence *
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * array = (rogilink_flex_interfaces__srv__GetConfig_Response__Sequence *)allocator.allocate(sizeof(rogilink_flex_interfaces__srv__GetConfig_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__destroy(rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__are_equal(const rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * lhs, const rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rogilink_flex_interfaces__srv__GetConfig_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rogilink_flex_interfaces__srv__GetConfig_Response__Sequence__copy(
  const rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * input,
  rogilink_flex_interfaces__srv__GetConfig_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rogilink_flex_interfaces__srv__GetConfig_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rogilink_flex_interfaces__srv__GetConfig_Response * data =
      (rogilink_flex_interfaces__srv__GetConfig_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rogilink_flex_interfaces__srv__GetConfig_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rogilink_flex_interfaces__srv__GetConfig_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rogilink_flex_interfaces__srv__GetConfig_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

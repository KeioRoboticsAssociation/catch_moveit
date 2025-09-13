// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:srv/SetRobomasterMotorConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/srv/detail/set_robomaster_motor_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `config`
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__functions.h"

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__init(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * msg)
{
  if (!msg) {
    return false;
  }
  // motor_id
  // config
  if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__init(&msg->config)) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(msg);
    return false;
  }
  return true;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * msg)
{
  if (!msg) {
    return;
  }
  // motor_id
  // config
  stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(&msg->config);
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__are_equal(const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * lhs, const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // config
  if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__are_equal(
      &(lhs->config), &(rhs->config)))
  {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__copy(
  const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * input,
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // config
  if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__copy(
      &(input->config), &(output->config)))
  {
    return false;
  }
  return true;
}

stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request *
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * msg = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request));
  bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__destroy(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__init(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(&data[i - 1]);
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
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__fini(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * array)
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
      stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(&array->data[i]);
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

stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence *
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * array = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__destroy(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__are_equal(const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * lhs, const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence__copy(
  const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * input,
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request * data =
      (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__init(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(msg);
    return false;
  }
  return true;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__are_equal(const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * lhs, const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__copy(
  const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * input,
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response *
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * msg = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response));
  bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__destroy(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__init(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(&data[i - 1]);
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
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__fini(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * array)
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
      stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(&array->data[i]);
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

stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence *
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * array = (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__destroy(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__are_equal(const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * lhs, const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence__copy(
  const stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * input,
  stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response * data =
      (stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetRobomasterMotorConfig_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

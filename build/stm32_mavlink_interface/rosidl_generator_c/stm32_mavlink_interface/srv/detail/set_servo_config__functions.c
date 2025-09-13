// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/srv/detail/set_servo_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
stm32_mavlink_interface__srv__SetServoConfig_Request__init(stm32_mavlink_interface__srv__SetServoConfig_Request * msg)
{
  if (!msg) {
    return false;
  }
  // servo_id
  // angle_min_deg
  // angle_max_deg
  // pulse_min_us
  // pulse_max_us
  // pulse_neutral_us
  // direction_inverted
  // offset_deg
  // max_velocity_deg_per_s
  // max_acceleration_deg_per_s2
  // watchdog_timeout_ms
  // fail_safe_behavior
  // save_to_flash
  return true;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Request__fini(stm32_mavlink_interface__srv__SetServoConfig_Request * msg)
{
  if (!msg) {
    return;
  }
  // servo_id
  // angle_min_deg
  // angle_max_deg
  // pulse_min_us
  // pulse_max_us
  // pulse_neutral_us
  // direction_inverted
  // offset_deg
  // max_velocity_deg_per_s
  // max_acceleration_deg_per_s2
  // watchdog_timeout_ms
  // fail_safe_behavior
  // save_to_flash
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Request__are_equal(const stm32_mavlink_interface__srv__SetServoConfig_Request * lhs, const stm32_mavlink_interface__srv__SetServoConfig_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // servo_id
  if (lhs->servo_id != rhs->servo_id) {
    return false;
  }
  // angle_min_deg
  if (lhs->angle_min_deg != rhs->angle_min_deg) {
    return false;
  }
  // angle_max_deg
  if (lhs->angle_max_deg != rhs->angle_max_deg) {
    return false;
  }
  // pulse_min_us
  if (lhs->pulse_min_us != rhs->pulse_min_us) {
    return false;
  }
  // pulse_max_us
  if (lhs->pulse_max_us != rhs->pulse_max_us) {
    return false;
  }
  // pulse_neutral_us
  if (lhs->pulse_neutral_us != rhs->pulse_neutral_us) {
    return false;
  }
  // direction_inverted
  if (lhs->direction_inverted != rhs->direction_inverted) {
    return false;
  }
  // offset_deg
  if (lhs->offset_deg != rhs->offset_deg) {
    return false;
  }
  // max_velocity_deg_per_s
  if (lhs->max_velocity_deg_per_s != rhs->max_velocity_deg_per_s) {
    return false;
  }
  // max_acceleration_deg_per_s2
  if (lhs->max_acceleration_deg_per_s2 != rhs->max_acceleration_deg_per_s2) {
    return false;
  }
  // watchdog_timeout_ms
  if (lhs->watchdog_timeout_ms != rhs->watchdog_timeout_ms) {
    return false;
  }
  // fail_safe_behavior
  if (lhs->fail_safe_behavior != rhs->fail_safe_behavior) {
    return false;
  }
  // save_to_flash
  if (lhs->save_to_flash != rhs->save_to_flash) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Request__copy(
  const stm32_mavlink_interface__srv__SetServoConfig_Request * input,
  stm32_mavlink_interface__srv__SetServoConfig_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // servo_id
  output->servo_id = input->servo_id;
  // angle_min_deg
  output->angle_min_deg = input->angle_min_deg;
  // angle_max_deg
  output->angle_max_deg = input->angle_max_deg;
  // pulse_min_us
  output->pulse_min_us = input->pulse_min_us;
  // pulse_max_us
  output->pulse_max_us = input->pulse_max_us;
  // pulse_neutral_us
  output->pulse_neutral_us = input->pulse_neutral_us;
  // direction_inverted
  output->direction_inverted = input->direction_inverted;
  // offset_deg
  output->offset_deg = input->offset_deg;
  // max_velocity_deg_per_s
  output->max_velocity_deg_per_s = input->max_velocity_deg_per_s;
  // max_acceleration_deg_per_s2
  output->max_acceleration_deg_per_s2 = input->max_acceleration_deg_per_s2;
  // watchdog_timeout_ms
  output->watchdog_timeout_ms = input->watchdog_timeout_ms;
  // fail_safe_behavior
  output->fail_safe_behavior = input->fail_safe_behavior;
  // save_to_flash
  output->save_to_flash = input->save_to_flash;
  return true;
}

stm32_mavlink_interface__srv__SetServoConfig_Request *
stm32_mavlink_interface__srv__SetServoConfig_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Request * msg = (stm32_mavlink_interface__srv__SetServoConfig_Request *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetServoConfig_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__srv__SetServoConfig_Request));
  bool success = stm32_mavlink_interface__srv__SetServoConfig_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Request__destroy(stm32_mavlink_interface__srv__SetServoConfig_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__srv__SetServoConfig_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__init(stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Request * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__srv__SetServoConfig_Request *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__srv__SetServoConfig_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__srv__SetServoConfig_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__srv__SetServoConfig_Request__fini(&data[i - 1]);
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
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__fini(stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * array)
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
      stm32_mavlink_interface__srv__SetServoConfig_Request__fini(&array->data[i]);
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

stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence *
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * array = (stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__destroy(stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__are_equal(const stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * lhs, const stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetServoConfig_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence__copy(
  const stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * input,
  stm32_mavlink_interface__srv__SetServoConfig_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__srv__SetServoConfig_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__srv__SetServoConfig_Request * data =
      (stm32_mavlink_interface__srv__SetServoConfig_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__srv__SetServoConfig_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__srv__SetServoConfig_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetServoConfig_Request__copy(
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
stm32_mavlink_interface__srv__SetServoConfig_Response__init(stm32_mavlink_interface__srv__SetServoConfig_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    stm32_mavlink_interface__srv__SetServoConfig_Response__fini(msg);
    return false;
  }
  return true;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Response__fini(stm32_mavlink_interface__srv__SetServoConfig_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Response__are_equal(const stm32_mavlink_interface__srv__SetServoConfig_Response * lhs, const stm32_mavlink_interface__srv__SetServoConfig_Response * rhs)
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
stm32_mavlink_interface__srv__SetServoConfig_Response__copy(
  const stm32_mavlink_interface__srv__SetServoConfig_Response * input,
  stm32_mavlink_interface__srv__SetServoConfig_Response * output)
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

stm32_mavlink_interface__srv__SetServoConfig_Response *
stm32_mavlink_interface__srv__SetServoConfig_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Response * msg = (stm32_mavlink_interface__srv__SetServoConfig_Response *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetServoConfig_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__srv__SetServoConfig_Response));
  bool success = stm32_mavlink_interface__srv__SetServoConfig_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Response__destroy(stm32_mavlink_interface__srv__SetServoConfig_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__srv__SetServoConfig_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__init(stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Response * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__srv__SetServoConfig_Response *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__srv__SetServoConfig_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__srv__SetServoConfig_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__srv__SetServoConfig_Response__fini(&data[i - 1]);
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
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__fini(stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * array)
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
      stm32_mavlink_interface__srv__SetServoConfig_Response__fini(&array->data[i]);
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

stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence *
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * array = (stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__destroy(stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__are_equal(const stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * lhs, const stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetServoConfig_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence__copy(
  const stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * input,
  stm32_mavlink_interface__srv__SetServoConfig_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__srv__SetServoConfig_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__srv__SetServoConfig_Response * data =
      (stm32_mavlink_interface__srv__SetServoConfig_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__srv__SetServoConfig_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__srv__SetServoConfig_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__srv__SetServoConfig_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

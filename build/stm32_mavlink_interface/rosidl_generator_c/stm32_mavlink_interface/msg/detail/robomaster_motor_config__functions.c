// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
// generated code does not contain a copyright notice
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__init(stm32_mavlink_interface__msg__RobomasterMotorConfig * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(msg);
    return false;
  }
  // motor_id
  // max_velocity_rps
  // max_acceleration_rps2
  // max_current_ma
  // min_current_ma
  // min_position_rad
  // max_position_rad
  // position_limits_enabled
  // watchdog_timeout_ms
  // max_temperature_celsius
  // failsafe_behavior
  // position_kp
  // position_ki
  // position_kd
  // velocity_kp
  // velocity_ki
  // velocity_kd
  // startup_position_rad
  // startup_mode
  // start_disabled
  // direction_inverted
  // position_offset_rad
  return true;
}

void
stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(stm32_mavlink_interface__msg__RobomasterMotorConfig * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // motor_id
  // max_velocity_rps
  // max_acceleration_rps2
  // max_current_ma
  // min_current_ma
  // min_position_rad
  // max_position_rad
  // position_limits_enabled
  // watchdog_timeout_ms
  // max_temperature_celsius
  // failsafe_behavior
  // position_kp
  // position_ki
  // position_kd
  // velocity_kp
  // velocity_ki
  // velocity_kd
  // startup_position_rad
  // startup_mode
  // start_disabled
  // direction_inverted
  // position_offset_rad
}

bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__are_equal(const stm32_mavlink_interface__msg__RobomasterMotorConfig * lhs, const stm32_mavlink_interface__msg__RobomasterMotorConfig * rhs)
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
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // max_velocity_rps
  if (lhs->max_velocity_rps != rhs->max_velocity_rps) {
    return false;
  }
  // max_acceleration_rps2
  if (lhs->max_acceleration_rps2 != rhs->max_acceleration_rps2) {
    return false;
  }
  // max_current_ma
  if (lhs->max_current_ma != rhs->max_current_ma) {
    return false;
  }
  // min_current_ma
  if (lhs->min_current_ma != rhs->min_current_ma) {
    return false;
  }
  // min_position_rad
  if (lhs->min_position_rad != rhs->min_position_rad) {
    return false;
  }
  // max_position_rad
  if (lhs->max_position_rad != rhs->max_position_rad) {
    return false;
  }
  // position_limits_enabled
  if (lhs->position_limits_enabled != rhs->position_limits_enabled) {
    return false;
  }
  // watchdog_timeout_ms
  if (lhs->watchdog_timeout_ms != rhs->watchdog_timeout_ms) {
    return false;
  }
  // max_temperature_celsius
  if (lhs->max_temperature_celsius != rhs->max_temperature_celsius) {
    return false;
  }
  // failsafe_behavior
  if (lhs->failsafe_behavior != rhs->failsafe_behavior) {
    return false;
  }
  // position_kp
  if (lhs->position_kp != rhs->position_kp) {
    return false;
  }
  // position_ki
  if (lhs->position_ki != rhs->position_ki) {
    return false;
  }
  // position_kd
  if (lhs->position_kd != rhs->position_kd) {
    return false;
  }
  // velocity_kp
  if (lhs->velocity_kp != rhs->velocity_kp) {
    return false;
  }
  // velocity_ki
  if (lhs->velocity_ki != rhs->velocity_ki) {
    return false;
  }
  // velocity_kd
  if (lhs->velocity_kd != rhs->velocity_kd) {
    return false;
  }
  // startup_position_rad
  if (lhs->startup_position_rad != rhs->startup_position_rad) {
    return false;
  }
  // startup_mode
  if (lhs->startup_mode != rhs->startup_mode) {
    return false;
  }
  // start_disabled
  if (lhs->start_disabled != rhs->start_disabled) {
    return false;
  }
  // direction_inverted
  if (lhs->direction_inverted != rhs->direction_inverted) {
    return false;
  }
  // position_offset_rad
  if (lhs->position_offset_rad != rhs->position_offset_rad) {
    return false;
  }
  return true;
}

bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__copy(
  const stm32_mavlink_interface__msg__RobomasterMotorConfig * input,
  stm32_mavlink_interface__msg__RobomasterMotorConfig * output)
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
  // motor_id
  output->motor_id = input->motor_id;
  // max_velocity_rps
  output->max_velocity_rps = input->max_velocity_rps;
  // max_acceleration_rps2
  output->max_acceleration_rps2 = input->max_acceleration_rps2;
  // max_current_ma
  output->max_current_ma = input->max_current_ma;
  // min_current_ma
  output->min_current_ma = input->min_current_ma;
  // min_position_rad
  output->min_position_rad = input->min_position_rad;
  // max_position_rad
  output->max_position_rad = input->max_position_rad;
  // position_limits_enabled
  output->position_limits_enabled = input->position_limits_enabled;
  // watchdog_timeout_ms
  output->watchdog_timeout_ms = input->watchdog_timeout_ms;
  // max_temperature_celsius
  output->max_temperature_celsius = input->max_temperature_celsius;
  // failsafe_behavior
  output->failsafe_behavior = input->failsafe_behavior;
  // position_kp
  output->position_kp = input->position_kp;
  // position_ki
  output->position_ki = input->position_ki;
  // position_kd
  output->position_kd = input->position_kd;
  // velocity_kp
  output->velocity_kp = input->velocity_kp;
  // velocity_ki
  output->velocity_ki = input->velocity_ki;
  // velocity_kd
  output->velocity_kd = input->velocity_kd;
  // startup_position_rad
  output->startup_position_rad = input->startup_position_rad;
  // startup_mode
  output->startup_mode = input->startup_mode;
  // start_disabled
  output->start_disabled = input->start_disabled;
  // direction_inverted
  output->direction_inverted = input->direction_inverted;
  // position_offset_rad
  output->position_offset_rad = input->position_offset_rad;
  return true;
}

stm32_mavlink_interface__msg__RobomasterMotorConfig *
stm32_mavlink_interface__msg__RobomasterMotorConfig__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorConfig * msg = (stm32_mavlink_interface__msg__RobomasterMotorConfig *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__RobomasterMotorConfig), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(stm32_mavlink_interface__msg__RobomasterMotorConfig));
  bool success = stm32_mavlink_interface__msg__RobomasterMotorConfig__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
stm32_mavlink_interface__msg__RobomasterMotorConfig__destroy(stm32_mavlink_interface__msg__RobomasterMotorConfig * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__init(stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorConfig * data = NULL;

  if (size) {
    data = (stm32_mavlink_interface__msg__RobomasterMotorConfig *)allocator.zero_allocate(size, sizeof(stm32_mavlink_interface__msg__RobomasterMotorConfig), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = stm32_mavlink_interface__msg__RobomasterMotorConfig__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(&data[i - 1]);
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
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__fini(stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * array)
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
      stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(&array->data[i]);
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

stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence *
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * array = (stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence *)allocator.allocate(sizeof(stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__destroy(stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__are_equal(const stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * lhs, const stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence__copy(
  const stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * input,
  stm32_mavlink_interface__msg__RobomasterMotorConfig__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(stm32_mavlink_interface__msg__RobomasterMotorConfig);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    stm32_mavlink_interface__msg__RobomasterMotorConfig * data =
      (stm32_mavlink_interface__msg__RobomasterMotorConfig *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          stm32_mavlink_interface__msg__RobomasterMotorConfig__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!stm32_mavlink_interface__msg__RobomasterMotorConfig__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

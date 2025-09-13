// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_state__struct.h"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_state__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__msg__robomaster_motor_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[73];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("stm32_mavlink_interface.msg._robomaster_motor_state.RobomasterMotorState", full_classname_dest, 72) == 0);
  }
  stm32_mavlink_interface__msg__RobomasterMotorState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // motor_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->motor_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // current_position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_velocity_rps
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_velocity_rps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_velocity_rps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // current_milliamps
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_milliamps");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->current_milliamps = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // temperature_celsius
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature_celsius");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->temperature_celsius = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // target_position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_velocity_rps
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_velocity_rps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_velocity_rps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_current_ma
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_current_ma");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->target_current_ma = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // control_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "control_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->control_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // status
    PyObject * field = PyObject_GetAttrString(_pymsg, "status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // last_command_time_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "last_command_time_ms");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->last_command_time_ms = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // saturation_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "saturation_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->saturation_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timeout_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "timeout_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timeout_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // overheat_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "overheat_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->overheat_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stm32_mavlink_interface__msg__robomaster_motor_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobomasterMotorState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.msg._robomaster_motor_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobomasterMotorState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__msg__RobomasterMotorState * ros_message = (stm32_mavlink_interface__msg__RobomasterMotorState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->motor_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_velocity_rps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_velocity_rps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_velocity_rps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // current_milliamps
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->current_milliamps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_milliamps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature_celsius
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->temperature_celsius);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature_celsius", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_velocity_rps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_velocity_rps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_velocity_rps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_current_ma
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->target_current_ma);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_current_ma", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // control_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->control_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "control_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // last_command_time_ms
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->last_command_time_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "last_command_time_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // saturation_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->saturation_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "saturation_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timeout_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timeout_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timeout_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // overheat_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->overheat_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "overheat_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stm32_mavlink_interface:msg/RobomasterMotorConfig.idl
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
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__struct.h"
#include "stm32_mavlink_interface/msg/detail/robomaster_motor_config__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__msg__robomaster_motor_config__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
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
    assert(strncmp("stm32_mavlink_interface.msg._robomaster_motor_config.RobomasterMotorConfig", full_classname_dest, 74) == 0);
  }
  stm32_mavlink_interface__msg__RobomasterMotorConfig * ros_message = _ros_message;
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
  {  // max_velocity_rps
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_velocity_rps");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_velocity_rps = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_acceleration_rps2
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_acceleration_rps2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_acceleration_rps2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_current_ma
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_current_ma");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->max_current_ma = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // min_current_ma
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_current_ma");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->min_current_ma = (int16_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // min_position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_limits_enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_limits_enabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->position_limits_enabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // watchdog_timeout_ms
    PyObject * field = PyObject_GetAttrString(_pymsg, "watchdog_timeout_ms");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->watchdog_timeout_ms = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // max_temperature_celsius
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_temperature_celsius");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->max_temperature_celsius = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // failsafe_behavior
    PyObject * field = PyObject_GetAttrString(_pymsg, "failsafe_behavior");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->failsafe_behavior = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // position_kp
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_kp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_kp = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_ki
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_ki");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_ki = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // position_kd
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_kd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_kd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_kp
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_kp");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_kp = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_ki
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_ki");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_ki = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // velocity_kd
    PyObject * field = PyObject_GetAttrString(_pymsg, "velocity_kd");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->velocity_kd = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // startup_position_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "startup_position_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->startup_position_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // startup_mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "startup_mode");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->startup_mode = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // start_disabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_disabled");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->start_disabled = (Py_True == field);
    Py_DECREF(field);
  }
  {  // direction_inverted
    PyObject * field = PyObject_GetAttrString(_pymsg, "direction_inverted");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->direction_inverted = (Py_True == field);
    Py_DECREF(field);
  }
  {  // position_offset_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "position_offset_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->position_offset_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stm32_mavlink_interface__msg__robomaster_motor_config__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of RobomasterMotorConfig */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.msg._robomaster_motor_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "RobomasterMotorConfig");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__msg__RobomasterMotorConfig * ros_message = (stm32_mavlink_interface__msg__RobomasterMotorConfig *)raw_ros_message;
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
  {  // max_velocity_rps
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_velocity_rps);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_velocity_rps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_acceleration_rps2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_acceleration_rps2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_acceleration_rps2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_current_ma
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->max_current_ma);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_current_ma", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_current_ma
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->min_current_ma);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_current_ma", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_limits_enabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->position_limits_enabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_limits_enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // watchdog_timeout_ms
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->watchdog_timeout_ms);
    {
      int rc = PyObject_SetAttrString(_pymessage, "watchdog_timeout_ms", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_temperature_celsius
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->max_temperature_celsius);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_temperature_celsius", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // failsafe_behavior
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->failsafe_behavior);
    {
      int rc = PyObject_SetAttrString(_pymessage, "failsafe_behavior", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_kp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_kp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_kp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_ki
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_ki);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_ki", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_kd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_kd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_kd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_kp
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_kp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_kp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_ki
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_ki);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_ki", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // velocity_kd
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->velocity_kd);
    {
      int rc = PyObject_SetAttrString(_pymessage, "velocity_kd", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // startup_position_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->startup_position_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "startup_position_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // startup_mode
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->startup_mode);
    {
      int rc = PyObject_SetAttrString(_pymessage, "startup_mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_disabled
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->start_disabled ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_disabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // direction_inverted
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->direction_inverted ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "direction_inverted", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position_offset_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->position_offset_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position_offset_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

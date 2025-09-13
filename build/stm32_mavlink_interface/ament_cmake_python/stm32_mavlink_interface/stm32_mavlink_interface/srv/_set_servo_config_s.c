// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stm32_mavlink_interface:srv/SetServoConfig.idl
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
#include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.h"
#include "stm32_mavlink_interface/srv/detail/set_servo_config__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__srv__set_servo_config__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[69];
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
    assert(strncmp("stm32_mavlink_interface.srv._set_servo_config.SetServoConfig_Request", full_classname_dest, 68) == 0);
  }
  stm32_mavlink_interface__srv__SetServoConfig_Request * ros_message = _ros_message;
  {  // servo_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "servo_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->servo_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // angle_min_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_min_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_min_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_max_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_max_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_max_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pulse_min_us
    PyObject * field = PyObject_GetAttrString(_pymsg, "pulse_min_us");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pulse_min_us = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pulse_max_us
    PyObject * field = PyObject_GetAttrString(_pymsg, "pulse_max_us");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pulse_max_us = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pulse_neutral_us
    PyObject * field = PyObject_GetAttrString(_pymsg, "pulse_neutral_us");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pulse_neutral_us = (uint16_t)PyLong_AsUnsignedLong(field);
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
  {  // offset_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "offset_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->offset_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_velocity_deg_per_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_velocity_deg_per_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_velocity_deg_per_s = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_acceleration_deg_per_s2
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_acceleration_deg_per_s2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_acceleration_deg_per_s2 = (float)PyFloat_AS_DOUBLE(field);
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
  {  // fail_safe_behavior
    PyObject * field = PyObject_GetAttrString(_pymsg, "fail_safe_behavior");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fail_safe_behavior = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // save_to_flash
    PyObject * field = PyObject_GetAttrString(_pymsg, "save_to_flash");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->save_to_flash = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stm32_mavlink_interface__srv__set_servo_config__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetServoConfig_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.srv._set_servo_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetServoConfig_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__srv__SetServoConfig_Request * ros_message = (stm32_mavlink_interface__srv__SetServoConfig_Request *)raw_ros_message;
  {  // servo_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->servo_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "servo_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_min_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_min_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_min_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_max_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_max_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_max_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pulse_min_us
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pulse_min_us);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pulse_min_us", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pulse_max_us
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pulse_max_us);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pulse_max_us", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pulse_neutral_us
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pulse_neutral_us);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pulse_neutral_us", field);
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
  {  // offset_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->offset_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offset_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_velocity_deg_per_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_velocity_deg_per_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_velocity_deg_per_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_acceleration_deg_per_s2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_acceleration_deg_per_s2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_acceleration_deg_per_s2", field);
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
  {  // fail_safe_behavior
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fail_safe_behavior);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fail_safe_behavior", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // save_to_flash
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->save_to_flash ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "save_to_flash", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_servo_config__struct.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_servo_config__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__srv__set_servo_config__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[70];
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
    assert(strncmp("stm32_mavlink_interface.srv._set_servo_config.SetServoConfig_Response", full_classname_dest, 69) == 0);
  }
  stm32_mavlink_interface__srv__SetServoConfig_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stm32_mavlink_interface__srv__set_servo_config__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetServoConfig_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.srv._set_servo_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetServoConfig_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__srv__SetServoConfig_Response * ros_message = (stm32_mavlink_interface__srv__SetServoConfig_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

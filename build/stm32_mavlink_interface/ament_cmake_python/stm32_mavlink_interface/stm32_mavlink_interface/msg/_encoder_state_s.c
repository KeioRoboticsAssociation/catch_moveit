// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stm32_mavlink_interface:msg/EncoderState.idl
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
#include "stm32_mavlink_interface/msg/detail/encoder_state__struct.h"
#include "stm32_mavlink_interface/msg/detail/encoder_state__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__msg__encoder_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("stm32_mavlink_interface.msg._encoder_state.EncoderState", full_classname_dest, 55) == 0);
  }
  stm32_mavlink_interface__msg__EncoderState * ros_message = _ros_message;
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
  {  // encoder_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->encoder_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // position
    PyObject * field = PyObject_GetAttrString(_pymsg, "position");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->position = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // angle_rad
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_rad");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_rad = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->angle_deg = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // revolutions
    PyObject * field = PyObject_GetAttrString(_pymsg, "revolutions");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->revolutions = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // z_detected
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_detected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->z_detected = (Py_True == field);
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
  {  // error_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_count = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * stm32_mavlink_interface__msg__encoder_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EncoderState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.msg._encoder_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EncoderState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__msg__EncoderState * ros_message = (stm32_mavlink_interface__msg__EncoderState *)raw_ros_message;
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
  {  // encoder_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->encoder_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "encoder_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // position
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->position);
    {
      int rc = PyObject_SetAttrString(_pymessage, "position", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_rad
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_rad);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_rad", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->angle_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "angle_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // revolutions
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->revolutions);
    {
      int rc = PyObject_SetAttrString(_pymessage, "revolutions", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_detected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->z_detected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_detected", field);
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

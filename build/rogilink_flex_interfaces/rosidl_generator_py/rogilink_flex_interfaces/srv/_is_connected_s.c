// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from rogilink_flex_interfaces:srv/IsConnected.idl
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
#include "rogilink_flex_interfaces/srv/detail/is_connected__struct.h"
#include "rogilink_flex_interfaces/srv/detail/is_connected__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rogilink_flex_interfaces__srv__is_connected__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[63];
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
    assert(strncmp("rogilink_flex_interfaces.srv._is_connected.IsConnected_Request", full_classname_dest, 62) == 0);
  }
  rogilink_flex_interfaces__srv__IsConnected_Request * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rogilink_flex_interfaces__srv__is_connected__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IsConnected_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rogilink_flex_interfaces.srv._is_connected");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IsConnected_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

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
// #include "rogilink_flex_interfaces/srv/detail/is_connected__struct.h"
// already included above
// #include "rogilink_flex_interfaces/srv/detail/is_connected__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool rogilink_flex_interfaces__srv__is_connected__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[64];
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
    assert(strncmp("rogilink_flex_interfaces.srv._is_connected.IsConnected_Response", full_classname_dest, 63) == 0);
  }
  rogilink_flex_interfaces__srv__IsConnected_Response * ros_message = _ros_message;
  {  // connected
    PyObject * field = PyObject_GetAttrString(_pymsg, "connected");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->connected = (Py_True == field);
    Py_DECREF(field);
  }
  {  // device_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "device_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->device_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * rogilink_flex_interfaces__srv__is_connected__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of IsConnected_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("rogilink_flex_interfaces.srv._is_connected");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "IsConnected_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  rogilink_flex_interfaces__srv__IsConnected_Response * ros_message = (rogilink_flex_interfaces__srv__IsConnected_Response *)raw_ros_message;
  {  // connected
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->connected ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "connected", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // device_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->device_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "device_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

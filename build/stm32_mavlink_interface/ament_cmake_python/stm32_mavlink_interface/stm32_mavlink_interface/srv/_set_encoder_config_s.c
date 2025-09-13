// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from stm32_mavlink_interface:srv/SetEncoderConfig.idl
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
#include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.h"
#include "stm32_mavlink_interface/srv/detail/set_encoder_config__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__srv__set_encoder_config__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("stm32_mavlink_interface.srv._set_encoder_config.SetEncoderConfig_Request", full_classname_dest, 72) == 0);
  }
  stm32_mavlink_interface__srv__SetEncoderConfig_Request * ros_message = _ros_message;
  {  // encoder_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->encoder_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cpr
    PyObject * field = PyObject_GetAttrString(_pymsg, "cpr");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cpr = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // invert_a
    PyObject * field = PyObject_GetAttrString(_pymsg, "invert_a");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invert_a = (Py_True == field);
    Py_DECREF(field);
  }
  {  // invert_b
    PyObject * field = PyObject_GetAttrString(_pymsg, "invert_b");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->invert_b = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_z");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_z = (Py_True == field);
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
  {  // offset_counts
    PyObject * field = PyObject_GetAttrString(_pymsg, "offset_counts");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->offset_counts = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // wrap_around
    PyObject * field = PyObject_GetAttrString(_pymsg, "wrap_around");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->wrap_around = (Py_True == field);
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
PyObject * stm32_mavlink_interface__srv__set_encoder_config__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetEncoderConfig_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.srv._set_encoder_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetEncoderConfig_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__srv__SetEncoderConfig_Request * ros_message = (stm32_mavlink_interface__srv__SetEncoderConfig_Request *)raw_ros_message;
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
  {  // cpr
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cpr);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cpr", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invert_a
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invert_a ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invert_a", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // invert_b
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->invert_b ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "invert_b", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_z
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_z ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_z", field);
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
  {  // offset_counts
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->offset_counts);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offset_counts", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // wrap_around
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->wrap_around ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "wrap_around", field);
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
// #include "stm32_mavlink_interface/srv/detail/set_encoder_config__struct.h"
// already included above
// #include "stm32_mavlink_interface/srv/detail/set_encoder_config__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool stm32_mavlink_interface__srv__set_encoder_config__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[74];
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
    assert(strncmp("stm32_mavlink_interface.srv._set_encoder_config.SetEncoderConfig_Response", full_classname_dest, 73) == 0);
  }
  stm32_mavlink_interface__srv__SetEncoderConfig_Response * ros_message = _ros_message;
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
PyObject * stm32_mavlink_interface__srv__set_encoder_config__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetEncoderConfig_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("stm32_mavlink_interface.srv._set_encoder_config");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetEncoderConfig_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  stm32_mavlink_interface__srv__SetEncoderConfig_Response * ros_message = (stm32_mavlink_interface__srv__SetEncoderConfig_Response *)raw_ros_message;
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

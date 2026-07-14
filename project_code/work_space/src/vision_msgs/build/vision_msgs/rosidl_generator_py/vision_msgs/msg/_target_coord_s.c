// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vision_msgs:msg/TargetCoord.idl
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
#include "vision_msgs/msg/detail/target_coord__struct.h"
#include "vision_msgs/msg/detail/target_coord__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vision_msgs__msg__target_coord__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
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
    assert(strncmp("vision_msgs.msg._target_coord.TargetCoord", full_classname_dest, 41) == 0);
  }
  vision_msgs__msg__TargetCoord * ros_message = _ros_message;
  {  // p_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // p_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "p_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->p_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // conf
    PyObject * field = PyObject_GetAttrString(_pymsg, "conf");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->conf = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // class_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "class_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->class_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vision_msgs__msg__target_coord__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TargetCoord */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vision_msgs.msg._target_coord");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TargetCoord");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vision_msgs__msg__TargetCoord * ros_message = (vision_msgs__msg__TargetCoord *)raw_ros_message;
  {  // p_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // p_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->p_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "p_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // conf
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->conf);
    {
      int rc = PyObject_SetAttrString(_pymessage, "conf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // class_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->class_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "class_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

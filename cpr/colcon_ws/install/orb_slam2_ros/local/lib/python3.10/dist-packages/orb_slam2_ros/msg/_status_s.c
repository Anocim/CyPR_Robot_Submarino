// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from orb_slam2_ros:msg/Status.idl
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
#include "orb_slam2_ros/msg/detail/status__struct.h"
#include "orb_slam2_ros/msg/detail/status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool orb_slam2_ros__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("orb_slam2_ros.msg._status.Status", full_classname_dest, 32) == 0);
  }
  orb_slam2_ros__msg__Status * ros_message = _ros_message;
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
  {  // left_stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->left_stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // right_stamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_stamp");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->right_stamp)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->state = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // only_tracking
    PyObject * field = PyObject_GetAttrString(_pymsg, "only_tracking");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->only_tracking = (Py_True == field);
    Py_DECREF(field);
  }
  {  // keyframes
    PyObject * field = PyObject_GetAttrString(_pymsg, "keyframes");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->keyframes = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // map_points
    PyObject * field = PyObject_GetAttrString(_pymsg, "map_points");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->map_points = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // tracked
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracked");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tracked = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // tracked_vo
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracked_vo");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tracked_vo = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * orb_slam2_ros__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("orb_slam2_ros.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  orb_slam2_ros__msg__Status * ros_message = (orb_slam2_ros__msg__Status *)raw_ros_message;
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
  {  // left_stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->left_stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_stamp
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->right_stamp);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_stamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // only_tracking
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->only_tracking ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "only_tracking", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // keyframes
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->keyframes);
    {
      int rc = PyObject_SetAttrString(_pymessage, "keyframes", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // map_points
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->map_points);
    {
      int rc = PyObject_SetAttrString(_pymessage, "map_points", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracked
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->tracked);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracked", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracked_vo
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->tracked_vo);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracked_vo", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orca_msgs:msg/Effort.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_H_
#define ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'force'
// Member 'torque'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Effort in the package orca_msgs.
typedef struct orca_msgs__msg__Effort
{
  geometry_msgs__msg__Vector3 force;
  geometry_msgs__msg__Vector3 torque;
} orca_msgs__msg__Effort;

// Struct for a sequence of orca_msgs__msg__Effort.
typedef struct orca_msgs__msg__Effort__Sequence
{
  orca_msgs__msg__Effort * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__msg__Effort__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_H_

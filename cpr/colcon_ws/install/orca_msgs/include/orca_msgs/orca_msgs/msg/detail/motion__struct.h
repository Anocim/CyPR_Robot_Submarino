// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orca_msgs:msg/Motion.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_H_
#define ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'cmd_vel'
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'accel_model'
// Member 'accel_drag'
// Member 'accel_total'
#include "geometry_msgs/msg/detail/accel__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'force'
#include "geometry_msgs/msg/detail/wrench__struct.h"
// Member 'effort'
#include "orca_msgs/msg/detail/effort__struct.h"

/// Struct defined in msg/Motion in the package orca_msgs.
typedef struct orca_msgs__msg__Motion
{
  std_msgs__msg__Header header;
  /// Inputs
  geometry_msgs__msg__Twist cmd_vel;
  double dt;
  /// Outputs
  /// Pose is in the world frame; acceleration, velocity, force and effort are in the robot frame
  /// Accel to move from v0 to v1
  geometry_msgs__msg__Accel accel_model;
  /// Accel to counteract drag
  geometry_msgs__msg__Accel accel_drag;
  geometry_msgs__msg__Accel accel_total;
  geometry_msgs__msg__Twist vel;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Wrench force;
  /// Force normalized to [-1, 1]
  orca_msgs__msg__Effort effort;
} orca_msgs__msg__Motion;

// Struct for a sequence of orca_msgs__msg__Motion.
typedef struct orca_msgs__msg__Motion__Sequence
{
  orca_msgs__msg__Motion * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__msg__Motion__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_H_

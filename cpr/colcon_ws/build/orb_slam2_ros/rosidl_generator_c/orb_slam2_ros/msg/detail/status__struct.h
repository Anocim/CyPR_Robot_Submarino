// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orb_slam2_ros:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_H_
#define ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_SYSTEM_NOT_READY'.
/**
  * LOADING ORB VOCABULARY. PLEASE WAIT...
 */
enum
{
  orb_slam2_ros__msg__Status__STATE_SYSTEM_NOT_READY = -1l
};

/// Constant 'STATE_NO_IMAGES_YET'.
/**
  * WAITING FOR IMAGES
 */
enum
{
  orb_slam2_ros__msg__Status__STATE_NO_IMAGES_YET = 0l
};

/// Constant 'STATE_NOT_INITIALIZED'.
/**
  * TRYING TO INITIALIZE
 */
enum
{
  orb_slam2_ros__msg__Status__STATE_NOT_INITIALIZED = 1l
};

/// Constant 'STATE_OK'.
/**
  * if !mbOnlyTracking SLAM MODE else LOCALIZATION
 */
enum
{
  orb_slam2_ros__msg__Status__STATE_OK = 2l
};

/// Constant 'STATE_LOST'.
/**
  * TRACK LOST. TRYING TO RELOCALIZE
 */
enum
{
  orb_slam2_ros__msg__Status__STATE_LOST = 3l
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'left_stamp'
// Member 'right_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Status in the package orb_slam2_ros.
/**
  * Time when SLAM processing completed
 */
typedef struct orb_slam2_ros__msg__Status
{
  std_msgs__msg__Header header;
  /// Image time(s)
  builtin_interfaces__msg__Time left_stamp;
  builtin_interfaces__msg__Time right_stamp;
  /// mState
  int32_t state;
  /// mbOnlyTracking
  bool only_tracking;
  /// mpMap->KeyFramesInMap()
  int32_t keyframes;
  /// mpMap->MapPointsInMap()
  int32_t map_points;
  /// mnTracked
  int32_t tracked;
  /// mnTrackedVO
  int32_t tracked_vo;
} orb_slam2_ros__msg__Status;

// Struct for a sequence of orb_slam2_ros__msg__Status.
typedef struct orb_slam2_ros__msg__Status__Sequence
{
  orb_slam2_ros__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orb_slam2_ros__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_H_

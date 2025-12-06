// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from orca_msgs:action/TargetMode.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__STRUCT_H_
#define ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ORCA_MODE_DISARMED'.
enum
{
  orca_msgs__action__TargetMode_Goal__ORCA_MODE_DISARMED = 1
};

/// Constant 'ORCA_MODE_ROV'.
enum
{
  orca_msgs__action__TargetMode_Goal__ORCA_MODE_ROV = 2
};

/// Constant 'ORCA_MODE_AUV'.
enum
{
  orca_msgs__action__TargetMode_Goal__ORCA_MODE_AUV = 3
};

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_Goal
{
  uint8_t target_mode;
} orca_msgs__action__TargetMode_Goal;

// Struct for a sequence of orca_msgs__action__TargetMode_Goal.
typedef struct orca_msgs__action__TargetMode_Goal__Sequence
{
  orca_msgs__action__TargetMode_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_Result
{
  uint8_t structure_needs_at_least_one_member;
} orca_msgs__action__TargetMode_Result;

// Struct for a sequence of orca_msgs__action__TargetMode_Result.
typedef struct orca_msgs__action__TargetMode_Result__Sequence
{
  orca_msgs__action__TargetMode_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} orca_msgs__action__TargetMode_Feedback;

// Struct for a sequence of orca_msgs__action__TargetMode_Feedback.
typedef struct orca_msgs__action__TargetMode_Feedback__Sequence
{
  orca_msgs__action__TargetMode_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "orca_msgs/action/detail/target_mode__struct.h"

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  orca_msgs__action__TargetMode_Goal goal;
} orca_msgs__action__TargetMode_SendGoal_Request;

// Struct for a sequence of orca_msgs__action__TargetMode_SendGoal_Request.
typedef struct orca_msgs__action__TargetMode_SendGoal_Request__Sequence
{
  orca_msgs__action__TargetMode_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} orca_msgs__action__TargetMode_SendGoal_Response;

// Struct for a sequence of orca_msgs__action__TargetMode_SendGoal_Response.
typedef struct orca_msgs__action__TargetMode_SendGoal_Response__Sequence
{
  orca_msgs__action__TargetMode_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} orca_msgs__action__TargetMode_GetResult_Request;

// Struct for a sequence of orca_msgs__action__TargetMode_GetResult_Request.
typedef struct orca_msgs__action__TargetMode_GetResult_Request__Sequence
{
  orca_msgs__action__TargetMode_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "orca_msgs/action/detail/target_mode__struct.h"

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_GetResult_Response
{
  int8_t status;
  orca_msgs__action__TargetMode_Result result;
} orca_msgs__action__TargetMode_GetResult_Response;

// Struct for a sequence of orca_msgs__action__TargetMode_GetResult_Response.
typedef struct orca_msgs__action__TargetMode_GetResult_Response__Sequence
{
  orca_msgs__action__TargetMode_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "orca_msgs/action/detail/target_mode__struct.h"

/// Struct defined in action/TargetMode in the package orca_msgs.
typedef struct orca_msgs__action__TargetMode_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  orca_msgs__action__TargetMode_Feedback feedback;
} orca_msgs__action__TargetMode_FeedbackMessage;

// Struct for a sequence of orca_msgs__action__TargetMode_FeedbackMessage.
typedef struct orca_msgs__action__TargetMode_FeedbackMessage__Sequence
{
  orca_msgs__action__TargetMode_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} orca_msgs__action__TargetMode_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__STRUCT_H_

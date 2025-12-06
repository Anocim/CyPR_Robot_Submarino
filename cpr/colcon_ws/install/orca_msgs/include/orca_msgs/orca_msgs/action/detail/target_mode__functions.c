// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orca_msgs:action/TargetMode.idl
// generated code does not contain a copyright notice
#include "orca_msgs/action/detail/target_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
orca_msgs__action__TargetMode_Goal__init(orca_msgs__action__TargetMode_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // target_mode
  return true;
}

void
orca_msgs__action__TargetMode_Goal__fini(orca_msgs__action__TargetMode_Goal * msg)
{
  if (!msg) {
    return;
  }
  // target_mode
}

bool
orca_msgs__action__TargetMode_Goal__are_equal(const orca_msgs__action__TargetMode_Goal * lhs, const orca_msgs__action__TargetMode_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_mode
  if (lhs->target_mode != rhs->target_mode) {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Goal__copy(
  const orca_msgs__action__TargetMode_Goal * input,
  orca_msgs__action__TargetMode_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // target_mode
  output->target_mode = input->target_mode;
  return true;
}

orca_msgs__action__TargetMode_Goal *
orca_msgs__action__TargetMode_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Goal * msg = (orca_msgs__action__TargetMode_Goal *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_Goal));
  bool success = orca_msgs__action__TargetMode_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_Goal__destroy(orca_msgs__action__TargetMode_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_Goal__Sequence__init(orca_msgs__action__TargetMode_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Goal * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_Goal *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_Goal__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_Goal__Sequence__fini(orca_msgs__action__TargetMode_Goal__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_Goal__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_Goal__Sequence *
orca_msgs__action__TargetMode_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Goal__Sequence * array = (orca_msgs__action__TargetMode_Goal__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_Goal__Sequence__destroy(orca_msgs__action__TargetMode_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_Goal__Sequence__are_equal(const orca_msgs__action__TargetMode_Goal__Sequence * lhs, const orca_msgs__action__TargetMode_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Goal__Sequence__copy(
  const orca_msgs__action__TargetMode_Goal__Sequence * input,
  orca_msgs__action__TargetMode_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_Goal * data =
      (orca_msgs__action__TargetMode_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
orca_msgs__action__TargetMode_Result__init(orca_msgs__action__TargetMode_Result * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
orca_msgs__action__TargetMode_Result__fini(orca_msgs__action__TargetMode_Result * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
orca_msgs__action__TargetMode_Result__are_equal(const orca_msgs__action__TargetMode_Result * lhs, const orca_msgs__action__TargetMode_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Result__copy(
  const orca_msgs__action__TargetMode_Result * input,
  orca_msgs__action__TargetMode_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

orca_msgs__action__TargetMode_Result *
orca_msgs__action__TargetMode_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Result * msg = (orca_msgs__action__TargetMode_Result *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_Result));
  bool success = orca_msgs__action__TargetMode_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_Result__destroy(orca_msgs__action__TargetMode_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_Result__Sequence__init(orca_msgs__action__TargetMode_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Result * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_Result *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_Result__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_Result__Sequence__fini(orca_msgs__action__TargetMode_Result__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_Result__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_Result__Sequence *
orca_msgs__action__TargetMode_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Result__Sequence * array = (orca_msgs__action__TargetMode_Result__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_Result__Sequence__destroy(orca_msgs__action__TargetMode_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_Result__Sequence__are_equal(const orca_msgs__action__TargetMode_Result__Sequence * lhs, const orca_msgs__action__TargetMode_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Result__Sequence__copy(
  const orca_msgs__action__TargetMode_Result__Sequence * input,
  orca_msgs__action__TargetMode_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_Result * data =
      (orca_msgs__action__TargetMode_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
orca_msgs__action__TargetMode_Feedback__init(orca_msgs__action__TargetMode_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
orca_msgs__action__TargetMode_Feedback__fini(orca_msgs__action__TargetMode_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
orca_msgs__action__TargetMode_Feedback__are_equal(const orca_msgs__action__TargetMode_Feedback * lhs, const orca_msgs__action__TargetMode_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Feedback__copy(
  const orca_msgs__action__TargetMode_Feedback * input,
  orca_msgs__action__TargetMode_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

orca_msgs__action__TargetMode_Feedback *
orca_msgs__action__TargetMode_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Feedback * msg = (orca_msgs__action__TargetMode_Feedback *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_Feedback));
  bool success = orca_msgs__action__TargetMode_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_Feedback__destroy(orca_msgs__action__TargetMode_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_Feedback__Sequence__init(orca_msgs__action__TargetMode_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Feedback * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_Feedback *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_Feedback__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_Feedback__Sequence__fini(orca_msgs__action__TargetMode_Feedback__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_Feedback__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_Feedback__Sequence *
orca_msgs__action__TargetMode_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_Feedback__Sequence * array = (orca_msgs__action__TargetMode_Feedback__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_Feedback__Sequence__destroy(orca_msgs__action__TargetMode_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_Feedback__Sequence__are_equal(const orca_msgs__action__TargetMode_Feedback__Sequence * lhs, const orca_msgs__action__TargetMode_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_Feedback__Sequence__copy(
  const orca_msgs__action__TargetMode_Feedback__Sequence * input,
  orca_msgs__action__TargetMode_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_Feedback * data =
      (orca_msgs__action__TargetMode_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "orca_msgs/action/detail/target_mode__functions.h"

bool
orca_msgs__action__TargetMode_SendGoal_Request__init(orca_msgs__action__TargetMode_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    orca_msgs__action__TargetMode_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!orca_msgs__action__TargetMode_Goal__init(&msg->goal)) {
    orca_msgs__action__TargetMode_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__action__TargetMode_SendGoal_Request__fini(orca_msgs__action__TargetMode_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  orca_msgs__action__TargetMode_Goal__fini(&msg->goal);
}

bool
orca_msgs__action__TargetMode_SendGoal_Request__are_equal(const orca_msgs__action__TargetMode_SendGoal_Request * lhs, const orca_msgs__action__TargetMode_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!orca_msgs__action__TargetMode_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_SendGoal_Request__copy(
  const orca_msgs__action__TargetMode_SendGoal_Request * input,
  orca_msgs__action__TargetMode_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!orca_msgs__action__TargetMode_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

orca_msgs__action__TargetMode_SendGoal_Request *
orca_msgs__action__TargetMode_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Request * msg = (orca_msgs__action__TargetMode_SendGoal_Request *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_SendGoal_Request));
  bool success = orca_msgs__action__TargetMode_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_SendGoal_Request__destroy(orca_msgs__action__TargetMode_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__init(orca_msgs__action__TargetMode_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Request * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_SendGoal_Request *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_SendGoal_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__fini(orca_msgs__action__TargetMode_SendGoal_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_SendGoal_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_SendGoal_Request__Sequence *
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Request__Sequence * array = (orca_msgs__action__TargetMode_SendGoal_Request__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__destroy(orca_msgs__action__TargetMode_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__are_equal(const orca_msgs__action__TargetMode_SendGoal_Request__Sequence * lhs, const orca_msgs__action__TargetMode_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_SendGoal_Request__Sequence__copy(
  const orca_msgs__action__TargetMode_SendGoal_Request__Sequence * input,
  orca_msgs__action__TargetMode_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_SendGoal_Request * data =
      (orca_msgs__action__TargetMode_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
orca_msgs__action__TargetMode_SendGoal_Response__init(orca_msgs__action__TargetMode_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    orca_msgs__action__TargetMode_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__action__TargetMode_SendGoal_Response__fini(orca_msgs__action__TargetMode_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
orca_msgs__action__TargetMode_SendGoal_Response__are_equal(const orca_msgs__action__TargetMode_SendGoal_Response * lhs, const orca_msgs__action__TargetMode_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_SendGoal_Response__copy(
  const orca_msgs__action__TargetMode_SendGoal_Response * input,
  orca_msgs__action__TargetMode_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

orca_msgs__action__TargetMode_SendGoal_Response *
orca_msgs__action__TargetMode_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Response * msg = (orca_msgs__action__TargetMode_SendGoal_Response *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_SendGoal_Response));
  bool success = orca_msgs__action__TargetMode_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_SendGoal_Response__destroy(orca_msgs__action__TargetMode_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__init(orca_msgs__action__TargetMode_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Response * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_SendGoal_Response *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_SendGoal_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__fini(orca_msgs__action__TargetMode_SendGoal_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_SendGoal_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_SendGoal_Response__Sequence *
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_SendGoal_Response__Sequence * array = (orca_msgs__action__TargetMode_SendGoal_Response__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__destroy(orca_msgs__action__TargetMode_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__are_equal(const orca_msgs__action__TargetMode_SendGoal_Response__Sequence * lhs, const orca_msgs__action__TargetMode_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_SendGoal_Response__Sequence__copy(
  const orca_msgs__action__TargetMode_SendGoal_Response__Sequence * input,
  orca_msgs__action__TargetMode_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_SendGoal_Response * data =
      (orca_msgs__action__TargetMode_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
orca_msgs__action__TargetMode_GetResult_Request__init(orca_msgs__action__TargetMode_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    orca_msgs__action__TargetMode_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__action__TargetMode_GetResult_Request__fini(orca_msgs__action__TargetMode_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
orca_msgs__action__TargetMode_GetResult_Request__are_equal(const orca_msgs__action__TargetMode_GetResult_Request * lhs, const orca_msgs__action__TargetMode_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_GetResult_Request__copy(
  const orca_msgs__action__TargetMode_GetResult_Request * input,
  orca_msgs__action__TargetMode_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

orca_msgs__action__TargetMode_GetResult_Request *
orca_msgs__action__TargetMode_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Request * msg = (orca_msgs__action__TargetMode_GetResult_Request *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_GetResult_Request));
  bool success = orca_msgs__action__TargetMode_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_GetResult_Request__destroy(orca_msgs__action__TargetMode_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_GetResult_Request__Sequence__init(orca_msgs__action__TargetMode_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Request * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_GetResult_Request *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_GetResult_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_GetResult_Request__Sequence__fini(orca_msgs__action__TargetMode_GetResult_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_GetResult_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_GetResult_Request__Sequence *
orca_msgs__action__TargetMode_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Request__Sequence * array = (orca_msgs__action__TargetMode_GetResult_Request__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_GetResult_Request__Sequence__destroy(orca_msgs__action__TargetMode_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_GetResult_Request__Sequence__are_equal(const orca_msgs__action__TargetMode_GetResult_Request__Sequence * lhs, const orca_msgs__action__TargetMode_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_GetResult_Request__Sequence__copy(
  const orca_msgs__action__TargetMode_GetResult_Request__Sequence * input,
  orca_msgs__action__TargetMode_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_GetResult_Request * data =
      (orca_msgs__action__TargetMode_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "orca_msgs/action/detail/target_mode__functions.h"

bool
orca_msgs__action__TargetMode_GetResult_Response__init(orca_msgs__action__TargetMode_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!orca_msgs__action__TargetMode_Result__init(&msg->result)) {
    orca_msgs__action__TargetMode_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__action__TargetMode_GetResult_Response__fini(orca_msgs__action__TargetMode_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  orca_msgs__action__TargetMode_Result__fini(&msg->result);
}

bool
orca_msgs__action__TargetMode_GetResult_Response__are_equal(const orca_msgs__action__TargetMode_GetResult_Response * lhs, const orca_msgs__action__TargetMode_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!orca_msgs__action__TargetMode_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_GetResult_Response__copy(
  const orca_msgs__action__TargetMode_GetResult_Response * input,
  orca_msgs__action__TargetMode_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!orca_msgs__action__TargetMode_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

orca_msgs__action__TargetMode_GetResult_Response *
orca_msgs__action__TargetMode_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Response * msg = (orca_msgs__action__TargetMode_GetResult_Response *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_GetResult_Response));
  bool success = orca_msgs__action__TargetMode_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_GetResult_Response__destroy(orca_msgs__action__TargetMode_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_GetResult_Response__Sequence__init(orca_msgs__action__TargetMode_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Response * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_GetResult_Response *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_GetResult_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_GetResult_Response__Sequence__fini(orca_msgs__action__TargetMode_GetResult_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_GetResult_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_GetResult_Response__Sequence *
orca_msgs__action__TargetMode_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_GetResult_Response__Sequence * array = (orca_msgs__action__TargetMode_GetResult_Response__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_GetResult_Response__Sequence__destroy(orca_msgs__action__TargetMode_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_GetResult_Response__Sequence__are_equal(const orca_msgs__action__TargetMode_GetResult_Response__Sequence * lhs, const orca_msgs__action__TargetMode_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_GetResult_Response__Sequence__copy(
  const orca_msgs__action__TargetMode_GetResult_Response__Sequence * input,
  orca_msgs__action__TargetMode_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_GetResult_Response * data =
      (orca_msgs__action__TargetMode_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "orca_msgs/action/detail/target_mode__functions.h"

bool
orca_msgs__action__TargetMode_FeedbackMessage__init(orca_msgs__action__TargetMode_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    orca_msgs__action__TargetMode_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!orca_msgs__action__TargetMode_Feedback__init(&msg->feedback)) {
    orca_msgs__action__TargetMode_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__action__TargetMode_FeedbackMessage__fini(orca_msgs__action__TargetMode_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  orca_msgs__action__TargetMode_Feedback__fini(&msg->feedback);
}

bool
orca_msgs__action__TargetMode_FeedbackMessage__are_equal(const orca_msgs__action__TargetMode_FeedbackMessage * lhs, const orca_msgs__action__TargetMode_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!orca_msgs__action__TargetMode_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__action__TargetMode_FeedbackMessage__copy(
  const orca_msgs__action__TargetMode_FeedbackMessage * input,
  orca_msgs__action__TargetMode_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!orca_msgs__action__TargetMode_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

orca_msgs__action__TargetMode_FeedbackMessage *
orca_msgs__action__TargetMode_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_FeedbackMessage * msg = (orca_msgs__action__TargetMode_FeedbackMessage *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__action__TargetMode_FeedbackMessage));
  bool success = orca_msgs__action__TargetMode_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__action__TargetMode_FeedbackMessage__destroy(orca_msgs__action__TargetMode_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__action__TargetMode_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__init(orca_msgs__action__TargetMode_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_FeedbackMessage * data = NULL;

  if (size) {
    data = (orca_msgs__action__TargetMode_FeedbackMessage *)allocator.zero_allocate(size, sizeof(orca_msgs__action__TargetMode_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__action__TargetMode_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__action__TargetMode_FeedbackMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__fini(orca_msgs__action__TargetMode_FeedbackMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      orca_msgs__action__TargetMode_FeedbackMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

orca_msgs__action__TargetMode_FeedbackMessage__Sequence *
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__action__TargetMode_FeedbackMessage__Sequence * array = (orca_msgs__action__TargetMode_FeedbackMessage__Sequence *)allocator.allocate(sizeof(orca_msgs__action__TargetMode_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__action__TargetMode_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__destroy(orca_msgs__action__TargetMode_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__action__TargetMode_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__are_equal(const orca_msgs__action__TargetMode_FeedbackMessage__Sequence * lhs, const orca_msgs__action__TargetMode_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__action__TargetMode_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__action__TargetMode_FeedbackMessage__Sequence__copy(
  const orca_msgs__action__TargetMode_FeedbackMessage__Sequence * input,
  orca_msgs__action__TargetMode_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__action__TargetMode_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__action__TargetMode_FeedbackMessage * data =
      (orca_msgs__action__TargetMode_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__action__TargetMode_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__action__TargetMode_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__action__TargetMode_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

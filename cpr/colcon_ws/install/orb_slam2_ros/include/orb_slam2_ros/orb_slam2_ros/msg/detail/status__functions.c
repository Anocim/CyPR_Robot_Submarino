// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orb_slam2_ros:msg/Status.idl
// generated code does not contain a copyright notice
#include "orb_slam2_ros/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `left_stamp`
// Member `right_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
orb_slam2_ros__msg__Status__init(orb_slam2_ros__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    orb_slam2_ros__msg__Status__fini(msg);
    return false;
  }
  // left_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->left_stamp)) {
    orb_slam2_ros__msg__Status__fini(msg);
    return false;
  }
  // right_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->right_stamp)) {
    orb_slam2_ros__msg__Status__fini(msg);
    return false;
  }
  // state
  // only_tracking
  // keyframes
  // map_points
  // tracked
  // tracked_vo
  return true;
}

void
orb_slam2_ros__msg__Status__fini(orb_slam2_ros__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // left_stamp
  builtin_interfaces__msg__Time__fini(&msg->left_stamp);
  // right_stamp
  builtin_interfaces__msg__Time__fini(&msg->right_stamp);
  // state
  // only_tracking
  // keyframes
  // map_points
  // tracked
  // tracked_vo
}

bool
orb_slam2_ros__msg__Status__are_equal(const orb_slam2_ros__msg__Status * lhs, const orb_slam2_ros__msg__Status * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // left_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->left_stamp), &(rhs->left_stamp)))
  {
    return false;
  }
  // right_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->right_stamp), &(rhs->right_stamp)))
  {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // only_tracking
  if (lhs->only_tracking != rhs->only_tracking) {
    return false;
  }
  // keyframes
  if (lhs->keyframes != rhs->keyframes) {
    return false;
  }
  // map_points
  if (lhs->map_points != rhs->map_points) {
    return false;
  }
  // tracked
  if (lhs->tracked != rhs->tracked) {
    return false;
  }
  // tracked_vo
  if (lhs->tracked_vo != rhs->tracked_vo) {
    return false;
  }
  return true;
}

bool
orb_slam2_ros__msg__Status__copy(
  const orb_slam2_ros__msg__Status * input,
  orb_slam2_ros__msg__Status * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // left_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->left_stamp), &(output->left_stamp)))
  {
    return false;
  }
  // right_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->right_stamp), &(output->right_stamp)))
  {
    return false;
  }
  // state
  output->state = input->state;
  // only_tracking
  output->only_tracking = input->only_tracking;
  // keyframes
  output->keyframes = input->keyframes;
  // map_points
  output->map_points = input->map_points;
  // tracked
  output->tracked = input->tracked;
  // tracked_vo
  output->tracked_vo = input->tracked_vo;
  return true;
}

orb_slam2_ros__msg__Status *
orb_slam2_ros__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orb_slam2_ros__msg__Status * msg = (orb_slam2_ros__msg__Status *)allocator.allocate(sizeof(orb_slam2_ros__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orb_slam2_ros__msg__Status));
  bool success = orb_slam2_ros__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orb_slam2_ros__msg__Status__destroy(orb_slam2_ros__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orb_slam2_ros__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orb_slam2_ros__msg__Status__Sequence__init(orb_slam2_ros__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orb_slam2_ros__msg__Status * data = NULL;

  if (size) {
    data = (orb_slam2_ros__msg__Status *)allocator.zero_allocate(size, sizeof(orb_slam2_ros__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orb_slam2_ros__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orb_slam2_ros__msg__Status__fini(&data[i - 1]);
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
orb_slam2_ros__msg__Status__Sequence__fini(orb_slam2_ros__msg__Status__Sequence * array)
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
      orb_slam2_ros__msg__Status__fini(&array->data[i]);
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

orb_slam2_ros__msg__Status__Sequence *
orb_slam2_ros__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orb_slam2_ros__msg__Status__Sequence * array = (orb_slam2_ros__msg__Status__Sequence *)allocator.allocate(sizeof(orb_slam2_ros__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orb_slam2_ros__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orb_slam2_ros__msg__Status__Sequence__destroy(orb_slam2_ros__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orb_slam2_ros__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orb_slam2_ros__msg__Status__Sequence__are_equal(const orb_slam2_ros__msg__Status__Sequence * lhs, const orb_slam2_ros__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orb_slam2_ros__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orb_slam2_ros__msg__Status__Sequence__copy(
  const orb_slam2_ros__msg__Status__Sequence * input,
  orb_slam2_ros__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orb_slam2_ros__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orb_slam2_ros__msg__Status * data =
      (orb_slam2_ros__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orb_slam2_ros__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orb_slam2_ros__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orb_slam2_ros__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

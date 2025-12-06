// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orca_msgs:msg/Motion.idl
// generated code does not contain a copyright notice
#include "orca_msgs/msg/detail/motion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `cmd_vel`
// Member `vel`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `accel_model`
// Member `accel_drag`
// Member `accel_total`
#include "geometry_msgs/msg/detail/accel__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `force`
#include "geometry_msgs/msg/detail/wrench__functions.h"
// Member `effort`
#include "orca_msgs/msg/detail/effort__functions.h"

bool
orca_msgs__msg__Motion__init(orca_msgs__msg__Motion * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // cmd_vel
  if (!geometry_msgs__msg__Twist__init(&msg->cmd_vel)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // dt
  // accel_model
  if (!geometry_msgs__msg__Accel__init(&msg->accel_model)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // accel_drag
  if (!geometry_msgs__msg__Accel__init(&msg->accel_drag)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // accel_total
  if (!geometry_msgs__msg__Accel__init(&msg->accel_total)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__init(&msg->vel)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // force
  if (!geometry_msgs__msg__Wrench__init(&msg->force)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  // effort
  if (!orca_msgs__msg__Effort__init(&msg->effort)) {
    orca_msgs__msg__Motion__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__msg__Motion__fini(orca_msgs__msg__Motion * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // cmd_vel
  geometry_msgs__msg__Twist__fini(&msg->cmd_vel);
  // dt
  // accel_model
  geometry_msgs__msg__Accel__fini(&msg->accel_model);
  // accel_drag
  geometry_msgs__msg__Accel__fini(&msg->accel_drag);
  // accel_total
  geometry_msgs__msg__Accel__fini(&msg->accel_total);
  // vel
  geometry_msgs__msg__Twist__fini(&msg->vel);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // force
  geometry_msgs__msg__Wrench__fini(&msg->force);
  // effort
  orca_msgs__msg__Effort__fini(&msg->effort);
}

bool
orca_msgs__msg__Motion__are_equal(const orca_msgs__msg__Motion * lhs, const orca_msgs__msg__Motion * rhs)
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
  // cmd_vel
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->cmd_vel), &(rhs->cmd_vel)))
  {
    return false;
  }
  // dt
  if (lhs->dt != rhs->dt) {
    return false;
  }
  // accel_model
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->accel_model), &(rhs->accel_model)))
  {
    return false;
  }
  // accel_drag
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->accel_drag), &(rhs->accel_drag)))
  {
    return false;
  }
  // accel_total
  if (!geometry_msgs__msg__Accel__are_equal(
      &(lhs->accel_total), &(rhs->accel_total)))
  {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // force
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->force), &(rhs->force)))
  {
    return false;
  }
  // effort
  if (!orca_msgs__msg__Effort__are_equal(
      &(lhs->effort), &(rhs->effort)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__msg__Motion__copy(
  const orca_msgs__msg__Motion * input,
  orca_msgs__msg__Motion * output)
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
  // cmd_vel
  if (!geometry_msgs__msg__Twist__copy(
      &(input->cmd_vel), &(output->cmd_vel)))
  {
    return false;
  }
  // dt
  output->dt = input->dt;
  // accel_model
  if (!geometry_msgs__msg__Accel__copy(
      &(input->accel_model), &(output->accel_model)))
  {
    return false;
  }
  // accel_drag
  if (!geometry_msgs__msg__Accel__copy(
      &(input->accel_drag), &(output->accel_drag)))
  {
    return false;
  }
  // accel_total
  if (!geometry_msgs__msg__Accel__copy(
      &(input->accel_total), &(output->accel_total)))
  {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // force
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->force), &(output->force)))
  {
    return false;
  }
  // effort
  if (!orca_msgs__msg__Effort__copy(
      &(input->effort), &(output->effort)))
  {
    return false;
  }
  return true;
}

orca_msgs__msg__Motion *
orca_msgs__msg__Motion__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Motion * msg = (orca_msgs__msg__Motion *)allocator.allocate(sizeof(orca_msgs__msg__Motion), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__msg__Motion));
  bool success = orca_msgs__msg__Motion__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__msg__Motion__destroy(orca_msgs__msg__Motion * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__msg__Motion__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__msg__Motion__Sequence__init(orca_msgs__msg__Motion__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Motion * data = NULL;

  if (size) {
    data = (orca_msgs__msg__Motion *)allocator.zero_allocate(size, sizeof(orca_msgs__msg__Motion), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__msg__Motion__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__msg__Motion__fini(&data[i - 1]);
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
orca_msgs__msg__Motion__Sequence__fini(orca_msgs__msg__Motion__Sequence * array)
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
      orca_msgs__msg__Motion__fini(&array->data[i]);
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

orca_msgs__msg__Motion__Sequence *
orca_msgs__msg__Motion__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Motion__Sequence * array = (orca_msgs__msg__Motion__Sequence *)allocator.allocate(sizeof(orca_msgs__msg__Motion__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__msg__Motion__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__msg__Motion__Sequence__destroy(orca_msgs__msg__Motion__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__msg__Motion__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__msg__Motion__Sequence__are_equal(const orca_msgs__msg__Motion__Sequence * lhs, const orca_msgs__msg__Motion__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__msg__Motion__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__msg__Motion__Sequence__copy(
  const orca_msgs__msg__Motion__Sequence * input,
  orca_msgs__msg__Motion__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__msg__Motion);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__msg__Motion * data =
      (orca_msgs__msg__Motion *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__msg__Motion__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__msg__Motion__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__msg__Motion__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

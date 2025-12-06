// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from orca_msgs:msg/Effort.idl
// generated code does not contain a copyright notice
#include "orca_msgs/msg/detail/effort__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `force`
// Member `torque`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
orca_msgs__msg__Effort__init(orca_msgs__msg__Effort * msg)
{
  if (!msg) {
    return false;
  }
  // force
  if (!geometry_msgs__msg__Vector3__init(&msg->force)) {
    orca_msgs__msg__Effort__fini(msg);
    return false;
  }
  // torque
  if (!geometry_msgs__msg__Vector3__init(&msg->torque)) {
    orca_msgs__msg__Effort__fini(msg);
    return false;
  }
  return true;
}

void
orca_msgs__msg__Effort__fini(orca_msgs__msg__Effort * msg)
{
  if (!msg) {
    return;
  }
  // force
  geometry_msgs__msg__Vector3__fini(&msg->force);
  // torque
  geometry_msgs__msg__Vector3__fini(&msg->torque);
}

bool
orca_msgs__msg__Effort__are_equal(const orca_msgs__msg__Effort * lhs, const orca_msgs__msg__Effort * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // force
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->force), &(rhs->force)))
  {
    return false;
  }
  // torque
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->torque), &(rhs->torque)))
  {
    return false;
  }
  return true;
}

bool
orca_msgs__msg__Effort__copy(
  const orca_msgs__msg__Effort * input,
  orca_msgs__msg__Effort * output)
{
  if (!input || !output) {
    return false;
  }
  // force
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->force), &(output->force)))
  {
    return false;
  }
  // torque
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->torque), &(output->torque)))
  {
    return false;
  }
  return true;
}

orca_msgs__msg__Effort *
orca_msgs__msg__Effort__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Effort * msg = (orca_msgs__msg__Effort *)allocator.allocate(sizeof(orca_msgs__msg__Effort), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(orca_msgs__msg__Effort));
  bool success = orca_msgs__msg__Effort__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
orca_msgs__msg__Effort__destroy(orca_msgs__msg__Effort * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    orca_msgs__msg__Effort__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
orca_msgs__msg__Effort__Sequence__init(orca_msgs__msg__Effort__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Effort * data = NULL;

  if (size) {
    data = (orca_msgs__msg__Effort *)allocator.zero_allocate(size, sizeof(orca_msgs__msg__Effort), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = orca_msgs__msg__Effort__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        orca_msgs__msg__Effort__fini(&data[i - 1]);
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
orca_msgs__msg__Effort__Sequence__fini(orca_msgs__msg__Effort__Sequence * array)
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
      orca_msgs__msg__Effort__fini(&array->data[i]);
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

orca_msgs__msg__Effort__Sequence *
orca_msgs__msg__Effort__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  orca_msgs__msg__Effort__Sequence * array = (orca_msgs__msg__Effort__Sequence *)allocator.allocate(sizeof(orca_msgs__msg__Effort__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = orca_msgs__msg__Effort__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
orca_msgs__msg__Effort__Sequence__destroy(orca_msgs__msg__Effort__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    orca_msgs__msg__Effort__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
orca_msgs__msg__Effort__Sequence__are_equal(const orca_msgs__msg__Effort__Sequence * lhs, const orca_msgs__msg__Effort__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!orca_msgs__msg__Effort__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
orca_msgs__msg__Effort__Sequence__copy(
  const orca_msgs__msg__Effort__Sequence * input,
  orca_msgs__msg__Effort__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(orca_msgs__msg__Effort);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    orca_msgs__msg__Effort * data =
      (orca_msgs__msg__Effort *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!orca_msgs__msg__Effort__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          orca_msgs__msg__Effort__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!orca_msgs__msg__Effort__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

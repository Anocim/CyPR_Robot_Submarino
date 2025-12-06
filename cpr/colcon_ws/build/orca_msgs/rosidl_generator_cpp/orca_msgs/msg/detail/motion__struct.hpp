// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orca_msgs:msg/Motion.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_HPP_
#define ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'cmd_vel'
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'accel_model'
// Member 'accel_drag'
// Member 'accel_total'
#include "geometry_msgs/msg/detail/accel__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'force'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"
// Member 'effort'
#include "orca_msgs/msg/detail/effort__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__orca_msgs__msg__Motion __attribute__((deprecated))
#else
# define DEPRECATED__orca_msgs__msg__Motion __declspec(deprecated)
#endif

namespace orca_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Motion_
{
  using Type = Motion_<ContainerAllocator>;

  explicit Motion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    cmd_vel(_init),
    accel_model(_init),
    accel_drag(_init),
    accel_total(_init),
    vel(_init),
    pose(_init),
    force(_init),
    effort(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dt = 0.0;
    }
  }

  explicit Motion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    cmd_vel(_alloc, _init),
    accel_model(_alloc, _init),
    accel_drag(_alloc, _init),
    accel_total(_alloc, _init),
    vel(_alloc, _init),
    pose(_alloc, _init),
    force(_alloc, _init),
    effort(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dt = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cmd_vel_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _cmd_vel_type cmd_vel;
  using _dt_type =
    double;
  _dt_type dt;
  using _accel_model_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_model_type accel_model;
  using _accel_drag_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_drag_type accel_drag;
  using _accel_total_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _accel_total_type accel_total;
  using _vel_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _vel_type vel;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _force_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _force_type force;
  using _effort_type =
    orca_msgs::msg::Effort_<ContainerAllocator>;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cmd_vel(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->cmd_vel = _arg;
    return *this;
  }
  Type & set__dt(
    const double & _arg)
  {
    this->dt = _arg;
    return *this;
  }
  Type & set__accel_model(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel_model = _arg;
    return *this;
  }
  Type & set__accel_drag(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel_drag = _arg;
    return *this;
  }
  Type & set__accel_total(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->accel_total = _arg;
    return *this;
  }
  Type & set__vel(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__force(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__effort(
    const orca_msgs::msg::Effort_<ContainerAllocator> & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orca_msgs::msg::Motion_<ContainerAllocator> *;
  using ConstRawPtr =
    const orca_msgs::msg::Motion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orca_msgs::msg::Motion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orca_msgs::msg::Motion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orca_msgs::msg::Motion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orca_msgs::msg::Motion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orca_msgs::msg::Motion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orca_msgs::msg::Motion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orca_msgs::msg::Motion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orca_msgs::msg::Motion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orca_msgs__msg__Motion
    std::shared_ptr<orca_msgs::msg::Motion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orca_msgs__msg__Motion
    std::shared_ptr<orca_msgs::msg::Motion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Motion_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cmd_vel != other.cmd_vel) {
      return false;
    }
    if (this->dt != other.dt) {
      return false;
    }
    if (this->accel_model != other.accel_model) {
      return false;
    }
    if (this->accel_drag != other.accel_drag) {
      return false;
    }
    if (this->accel_total != other.accel_total) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->force != other.force) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const Motion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Motion_

// alias to use template instance with default allocator
using Motion =
  orca_msgs::msg::Motion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace orca_msgs

#endif  // ORCA_MSGS__MSG__DETAIL__MOTION__STRUCT_HPP_

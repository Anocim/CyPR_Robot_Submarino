// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orca_msgs:msg/Effort.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_HPP_
#define ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'force'
// Member 'torque'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__orca_msgs__msg__Effort __attribute__((deprecated))
#else
# define DEPRECATED__orca_msgs__msg__Effort __declspec(deprecated)
#endif

namespace orca_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Effort_
{
  using Type = Effort_<ContainerAllocator>;

  explicit Effort_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_init),
    torque(_init)
  {
    (void)_init;
  }

  explicit Effort_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : force(_alloc, _init),
    torque(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _force_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _force_type force;
  using _torque_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _torque_type torque;

  // setters for named parameter idiom
  Type & set__force(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->force = _arg;
    return *this;
  }
  Type & set__torque(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->torque = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orca_msgs::msg::Effort_<ContainerAllocator> *;
  using ConstRawPtr =
    const orca_msgs::msg::Effort_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orca_msgs::msg::Effort_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orca_msgs::msg::Effort_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orca_msgs::msg::Effort_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orca_msgs::msg::Effort_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orca_msgs::msg::Effort_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orca_msgs::msg::Effort_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orca_msgs::msg::Effort_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orca_msgs::msg::Effort_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orca_msgs__msg__Effort
    std::shared_ptr<orca_msgs::msg::Effort_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orca_msgs__msg__Effort
    std::shared_ptr<orca_msgs::msg::Effort_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Effort_ & other) const
  {
    if (this->force != other.force) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    return true;
  }
  bool operator!=(const Effort_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Effort_

// alias to use template instance with default allocator
using Effort =
  orca_msgs::msg::Effort_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace orca_msgs

#endif  // ORCA_MSGS__MSG__DETAIL__EFFORT__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orb_slam2_ros:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_HPP_

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
// Member 'left_stamp'
// Member 'right_stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__orb_slam2_ros__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__orb_slam2_ros__msg__Status __declspec(deprecated)
#endif

namespace orb_slam2_ros
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left_stamp(_init),
    right_stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0l;
      this->only_tracking = false;
      this->keyframes = 0l;
      this->map_points = 0l;
      this->tracked = 0l;
      this->tracked_vo = 0l;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left_stamp(_alloc, _init),
    right_stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0l;
      this->only_tracking = false;
      this->keyframes = 0l;
      this->map_points = 0l;
      this->tracked = 0l;
      this->tracked_vo = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _left_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _left_stamp_type left_stamp;
  using _right_stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _right_stamp_type right_stamp;
  using _state_type =
    int32_t;
  _state_type state;
  using _only_tracking_type =
    bool;
  _only_tracking_type only_tracking;
  using _keyframes_type =
    int32_t;
  _keyframes_type keyframes;
  using _map_points_type =
    int32_t;
  _map_points_type map_points;
  using _tracked_type =
    int32_t;
  _tracked_type tracked;
  using _tracked_vo_type =
    int32_t;
  _tracked_vo_type tracked_vo;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__left_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->left_stamp = _arg;
    return *this;
  }
  Type & set__right_stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->right_stamp = _arg;
    return *this;
  }
  Type & set__state(
    const int32_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__only_tracking(
    const bool & _arg)
  {
    this->only_tracking = _arg;
    return *this;
  }
  Type & set__keyframes(
    const int32_t & _arg)
  {
    this->keyframes = _arg;
    return *this;
  }
  Type & set__map_points(
    const int32_t & _arg)
  {
    this->map_points = _arg;
    return *this;
  }
  Type & set__tracked(
    const int32_t & _arg)
  {
    this->tracked = _arg;
    return *this;
  }
  Type & set__tracked_vo(
    const int32_t & _arg)
  {
    this->tracked_vo = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t STATE_SYSTEM_NOT_READY =
    -1;
  static constexpr int32_t STATE_NO_IMAGES_YET =
    0;
  static constexpr int32_t STATE_NOT_INITIALIZED =
    1;
  static constexpr int32_t STATE_OK =
    2;
  static constexpr int32_t STATE_LOST =
    3;

  // pointer types
  using RawPtr =
    orb_slam2_ros::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const orb_slam2_ros::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orb_slam2_ros::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orb_slam2_ros::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orb_slam2_ros__msg__Status
    std::shared_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orb_slam2_ros__msg__Status
    std::shared_ptr<orb_slam2_ros::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->left_stamp != other.left_stamp) {
      return false;
    }
    if (this->right_stamp != other.right_stamp) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->only_tracking != other.only_tracking) {
      return false;
    }
    if (this->keyframes != other.keyframes) {
      return false;
    }
    if (this->map_points != other.map_points) {
      return false;
    }
    if (this->tracked != other.tracked) {
      return false;
    }
    if (this->tracked_vo != other.tracked_vo) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  orb_slam2_ros::msg::Status_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Status_<ContainerAllocator>::STATE_SYSTEM_NOT_READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Status_<ContainerAllocator>::STATE_NO_IMAGES_YET;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Status_<ContainerAllocator>::STATE_NOT_INITIALIZED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Status_<ContainerAllocator>::STATE_OK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Status_<ContainerAllocator>::STATE_LOST;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace orb_slam2_ros

#endif  // ORB_SLAM2_ROS__MSG__DETAIL__STATUS__STRUCT_HPP_

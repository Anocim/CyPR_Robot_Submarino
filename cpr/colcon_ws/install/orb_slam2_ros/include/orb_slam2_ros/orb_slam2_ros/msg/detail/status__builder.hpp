// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orb_slam2_ros:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ORB_SLAM2_ROS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define ORB_SLAM2_ROS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orb_slam2_ros/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orb_slam2_ros
{

namespace msg
{

namespace builder
{

class Init_Status_tracked_vo
{
public:
  explicit Init_Status_tracked_vo(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  ::orb_slam2_ros::msg::Status tracked_vo(::orb_slam2_ros::msg::Status::_tracked_vo_type arg)
  {
    msg_.tracked_vo = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_tracked
{
public:
  explicit Init_Status_tracked(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_tracked_vo tracked(::orb_slam2_ros::msg::Status::_tracked_type arg)
  {
    msg_.tracked = std::move(arg);
    return Init_Status_tracked_vo(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_map_points
{
public:
  explicit Init_Status_map_points(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_tracked map_points(::orb_slam2_ros::msg::Status::_map_points_type arg)
  {
    msg_.map_points = std::move(arg);
    return Init_Status_tracked(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_keyframes
{
public:
  explicit Init_Status_keyframes(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_map_points keyframes(::orb_slam2_ros::msg::Status::_keyframes_type arg)
  {
    msg_.keyframes = std::move(arg);
    return Init_Status_map_points(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_only_tracking
{
public:
  explicit Init_Status_only_tracking(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_keyframes only_tracking(::orb_slam2_ros::msg::Status::_only_tracking_type arg)
  {
    msg_.only_tracking = std::move(arg);
    return Init_Status_keyframes(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_state
{
public:
  explicit Init_Status_state(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_only_tracking state(::orb_slam2_ros::msg::Status::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_Status_only_tracking(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_right_stamp
{
public:
  explicit Init_Status_right_stamp(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_state right_stamp(::orb_slam2_ros::msg::Status::_right_stamp_type arg)
  {
    msg_.right_stamp = std::move(arg);
    return Init_Status_state(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_left_stamp
{
public:
  explicit Init_Status_left_stamp(::orb_slam2_ros::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_right_stamp left_stamp(::orb_slam2_ros::msg::Status::_left_stamp_type arg)
  {
    msg_.left_stamp = std::move(arg);
    return Init_Status_right_stamp(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_left_stamp header(::orb_slam2_ros::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_left_stamp(msg_);
  }

private:
  ::orb_slam2_ros::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orb_slam2_ros::msg::Status>()
{
  return orb_slam2_ros::msg::builder::Init_Status_header();
}

}  // namespace orb_slam2_ros

#endif  // ORB_SLAM2_ROS__MSG__DETAIL__STATUS__BUILDER_HPP_

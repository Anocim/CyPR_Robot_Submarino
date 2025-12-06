// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orca_msgs:msg/Motion.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__MOTION__BUILDER_HPP_
#define ORCA_MSGS__MSG__DETAIL__MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orca_msgs/msg/detail/motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orca_msgs
{

namespace msg
{

namespace builder
{

class Init_Motion_effort
{
public:
  explicit Init_Motion_effort(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  ::orca_msgs::msg::Motion effort(::orca_msgs::msg::Motion::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_force
{
public:
  explicit Init_Motion_force(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_effort force(::orca_msgs::msg::Motion::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_Motion_effort(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_pose
{
public:
  explicit Init_Motion_pose(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_force pose(::orca_msgs::msg::Motion::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Motion_force(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_vel
{
public:
  explicit Init_Motion_vel(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_pose vel(::orca_msgs::msg::Motion::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_Motion_pose(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_accel_total
{
public:
  explicit Init_Motion_accel_total(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_vel accel_total(::orca_msgs::msg::Motion::_accel_total_type arg)
  {
    msg_.accel_total = std::move(arg);
    return Init_Motion_vel(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_accel_drag
{
public:
  explicit Init_Motion_accel_drag(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_accel_total accel_drag(::orca_msgs::msg::Motion::_accel_drag_type arg)
  {
    msg_.accel_drag = std::move(arg);
    return Init_Motion_accel_total(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_accel_model
{
public:
  explicit Init_Motion_accel_model(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_accel_drag accel_model(::orca_msgs::msg::Motion::_accel_model_type arg)
  {
    msg_.accel_model = std::move(arg);
    return Init_Motion_accel_drag(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_dt
{
public:
  explicit Init_Motion_dt(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_accel_model dt(::orca_msgs::msg::Motion::_dt_type arg)
  {
    msg_.dt = std::move(arg);
    return Init_Motion_accel_model(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_cmd_vel
{
public:
  explicit Init_Motion_cmd_vel(::orca_msgs::msg::Motion & msg)
  : msg_(msg)
  {}
  Init_Motion_dt cmd_vel(::orca_msgs::msg::Motion::_cmd_vel_type arg)
  {
    msg_.cmd_vel = std::move(arg);
    return Init_Motion_dt(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

class Init_Motion_header
{
public:
  Init_Motion_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Motion_cmd_vel header(::orca_msgs::msg::Motion::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Motion_cmd_vel(msg_);
  }

private:
  ::orca_msgs::msg::Motion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::msg::Motion>()
{
  return orca_msgs::msg::builder::Init_Motion_header();
}

}  // namespace orca_msgs

#endif  // ORCA_MSGS__MSG__DETAIL__MOTION__BUILDER_HPP_

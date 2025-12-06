// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orca_msgs:msg/Effort.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__EFFORT__BUILDER_HPP_
#define ORCA_MSGS__MSG__DETAIL__EFFORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orca_msgs/msg/detail/effort__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orca_msgs
{

namespace msg
{

namespace builder
{

class Init_Effort_torque
{
public:
  explicit Init_Effort_torque(::orca_msgs::msg::Effort & msg)
  : msg_(msg)
  {}
  ::orca_msgs::msg::Effort torque(::orca_msgs::msg::Effort::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::msg::Effort msg_;
};

class Init_Effort_force
{
public:
  Init_Effort_force()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Effort_torque force(::orca_msgs::msg::Effort::_force_type arg)
  {
    msg_.force = std::move(arg);
    return Init_Effort_torque(msg_);
  }

private:
  ::orca_msgs::msg::Effort msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::msg::Effort>()
{
  return orca_msgs::msg::builder::Init_Effort_force();
}

}  // namespace orca_msgs

#endif  // ORCA_MSGS__MSG__DETAIL__EFFORT__BUILDER_HPP_

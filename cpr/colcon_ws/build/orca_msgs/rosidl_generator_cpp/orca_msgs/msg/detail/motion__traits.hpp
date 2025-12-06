// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from orca_msgs:msg/Motion.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__MSG__DETAIL__MOTION__TRAITS_HPP_
#define ORCA_MSGS__MSG__DETAIL__MOTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "orca_msgs/msg/detail/motion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'cmd_vel'
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'accel_model'
// Member 'accel_drag'
// Member 'accel_total'
#include "geometry_msgs/msg/detail/accel__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'force'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"
// Member 'effort'
#include "orca_msgs/msg/detail/effort__traits.hpp"

namespace orca_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Motion & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: cmd_vel
  {
    out << "cmd_vel: ";
    to_flow_style_yaml(msg.cmd_vel, out);
    out << ", ";
  }

  // member: dt
  {
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
    out << ", ";
  }

  // member: accel_model
  {
    out << "accel_model: ";
    to_flow_style_yaml(msg.accel_model, out);
    out << ", ";
  }

  // member: accel_drag
  {
    out << "accel_drag: ";
    to_flow_style_yaml(msg.accel_drag, out);
    out << ", ";
  }

  // member: accel_total
  {
    out << "accel_total: ";
    to_flow_style_yaml(msg.accel_total, out);
    out << ", ";
  }

  // member: vel
  {
    out << "vel: ";
    to_flow_style_yaml(msg.vel, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    to_flow_style_yaml(msg.force, out);
    out << ", ";
  }

  // member: effort
  {
    out << "effort: ";
    to_flow_style_yaml(msg.effort, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Motion & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: cmd_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_vel:\n";
    to_block_style_yaml(msg.cmd_vel, out, indentation + 2);
  }

  // member: dt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dt: ";
    rosidl_generator_traits::value_to_yaml(msg.dt, out);
    out << "\n";
  }

  // member: accel_model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_model:\n";
    to_block_style_yaml(msg.accel_model, out, indentation + 2);
  }

  // member: accel_drag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_drag:\n";
    to_block_style_yaml(msg.accel_drag, out, indentation + 2);
  }

  // member: accel_total
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accel_total:\n";
    to_block_style_yaml(msg.accel_total, out, indentation + 2);
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel:\n";
    to_block_style_yaml(msg.vel, out, indentation + 2);
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force:\n";
    to_block_style_yaml(msg.force, out, indentation + 2);
  }

  // member: effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "effort:\n";
    to_block_style_yaml(msg.effort, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Motion & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace orca_msgs

namespace rosidl_generator_traits
{

[[deprecated("use orca_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orca_msgs::msg::Motion & msg,
  std::ostream & out, size_t indentation = 0)
{
  orca_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orca_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const orca_msgs::msg::Motion & msg)
{
  return orca_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<orca_msgs::msg::Motion>()
{
  return "orca_msgs::msg::Motion";
}

template<>
inline const char * name<orca_msgs::msg::Motion>()
{
  return "orca_msgs/msg/Motion";
}

template<>
struct has_fixed_size<orca_msgs::msg::Motion>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Accel>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value && has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<orca_msgs::msg::Effort>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<orca_msgs::msg::Motion>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Accel>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value && has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<orca_msgs::msg::Effort>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<orca_msgs::msg::Motion>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ORCA_MSGS__MSG__DETAIL__MOTION__TRAITS_HPP_

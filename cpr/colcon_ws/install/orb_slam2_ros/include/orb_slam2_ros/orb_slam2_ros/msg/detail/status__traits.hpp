// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from orb_slam2_ros:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef ORB_SLAM2_ROS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define ORB_SLAM2_ROS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "orb_slam2_ros/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'left_stamp'
// Member 'right_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace orb_slam2_ros
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: left_stamp
  {
    out << "left_stamp: ";
    to_flow_style_yaml(msg.left_stamp, out);
    out << ", ";
  }

  // member: right_stamp
  {
    out << "right_stamp: ";
    to_flow_style_yaml(msg.right_stamp, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: only_tracking
  {
    out << "only_tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.only_tracking, out);
    out << ", ";
  }

  // member: keyframes
  {
    out << "keyframes: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframes, out);
    out << ", ";
  }

  // member: map_points
  {
    out << "map_points: ";
    rosidl_generator_traits::value_to_yaml(msg.map_points, out);
    out << ", ";
  }

  // member: tracked
  {
    out << "tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked, out);
    out << ", ";
  }

  // member: tracked_vo
  {
    out << "tracked_vo: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked_vo, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: left_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_stamp:\n";
    to_block_style_yaml(msg.left_stamp, out, indentation + 2);
  }

  // member: right_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_stamp:\n";
    to_block_style_yaml(msg.right_stamp, out, indentation + 2);
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: only_tracking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "only_tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.only_tracking, out);
    out << "\n";
  }

  // member: keyframes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyframes: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframes, out);
    out << "\n";
  }

  // member: map_points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_points: ";
    rosidl_generator_traits::value_to_yaml(msg.map_points, out);
    out << "\n";
  }

  // member: tracked
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracked: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked, out);
    out << "\n";
  }

  // member: tracked_vo
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracked_vo: ";
    rosidl_generator_traits::value_to_yaml(msg.tracked_vo, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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

}  // namespace orb_slam2_ros

namespace rosidl_generator_traits
{

[[deprecated("use orb_slam2_ros::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orb_slam2_ros::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  orb_slam2_ros::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orb_slam2_ros::msg::to_yaml() instead")]]
inline std::string to_yaml(const orb_slam2_ros::msg::Status & msg)
{
  return orb_slam2_ros::msg::to_yaml(msg);
}

template<>
inline const char * data_type<orb_slam2_ros::msg::Status>()
{
  return "orb_slam2_ros::msg::Status";
}

template<>
inline const char * name<orb_slam2_ros::msg::Status>()
{
  return "orb_slam2_ros/msg/Status";
}

template<>
struct has_fixed_size<orb_slam2_ros::msg::Status>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<orb_slam2_ros::msg::Status>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<orb_slam2_ros::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ORB_SLAM2_ROS__MSG__DETAIL__STATUS__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from orca_msgs:action/TargetMode.idl
// generated code does not contain a copyright notice

#ifndef ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__BUILDER_HPP_
#define ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "orca_msgs/action/detail/target_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_Goal_target_mode
{
public:
  Init_TargetMode_Goal_target_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::orca_msgs::action::TargetMode_Goal target_mode(::orca_msgs::action::TargetMode_Goal::_target_mode_type arg)
  {
    msg_.target_mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_Goal>()
{
  return orca_msgs::action::builder::Init_TargetMode_Goal_target_mode();
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_Result>()
{
  return ::orca_msgs::action::TargetMode_Result(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_Feedback>()
{
  return ::orca_msgs::action::TargetMode_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_SendGoal_Request_goal
{
public:
  explicit Init_TargetMode_SendGoal_Request_goal(::orca_msgs::action::TargetMode_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::orca_msgs::action::TargetMode_SendGoal_Request goal(::orca_msgs::action::TargetMode_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_SendGoal_Request msg_;
};

class Init_TargetMode_SendGoal_Request_goal_id
{
public:
  Init_TargetMode_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetMode_SendGoal_Request_goal goal_id(::orca_msgs::action::TargetMode_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TargetMode_SendGoal_Request_goal(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_SendGoal_Request>()
{
  return orca_msgs::action::builder::Init_TargetMode_SendGoal_Request_goal_id();
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_SendGoal_Response_stamp
{
public:
  explicit Init_TargetMode_SendGoal_Response_stamp(::orca_msgs::action::TargetMode_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::orca_msgs::action::TargetMode_SendGoal_Response stamp(::orca_msgs::action::TargetMode_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_SendGoal_Response msg_;
};

class Init_TargetMode_SendGoal_Response_accepted
{
public:
  Init_TargetMode_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetMode_SendGoal_Response_stamp accepted(::orca_msgs::action::TargetMode_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_TargetMode_SendGoal_Response_stamp(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_SendGoal_Response>()
{
  return orca_msgs::action::builder::Init_TargetMode_SendGoal_Response_accepted();
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_GetResult_Request_goal_id
{
public:
  Init_TargetMode_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::orca_msgs::action::TargetMode_GetResult_Request goal_id(::orca_msgs::action::TargetMode_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_GetResult_Request>()
{
  return orca_msgs::action::builder::Init_TargetMode_GetResult_Request_goal_id();
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_GetResult_Response_result
{
public:
  explicit Init_TargetMode_GetResult_Response_result(::orca_msgs::action::TargetMode_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::orca_msgs::action::TargetMode_GetResult_Response result(::orca_msgs::action::TargetMode_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_GetResult_Response msg_;
};

class Init_TargetMode_GetResult_Response_status
{
public:
  Init_TargetMode_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetMode_GetResult_Response_result status(::orca_msgs::action::TargetMode_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TargetMode_GetResult_Response_result(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_GetResult_Response>()
{
  return orca_msgs::action::builder::Init_TargetMode_GetResult_Response_status();
}

}  // namespace orca_msgs


namespace orca_msgs
{

namespace action
{

namespace builder
{

class Init_TargetMode_FeedbackMessage_feedback
{
public:
  explicit Init_TargetMode_FeedbackMessage_feedback(::orca_msgs::action::TargetMode_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::orca_msgs::action::TargetMode_FeedbackMessage feedback(::orca_msgs::action::TargetMode_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_FeedbackMessage msg_;
};

class Init_TargetMode_FeedbackMessage_goal_id
{
public:
  Init_TargetMode_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TargetMode_FeedbackMessage_feedback goal_id(::orca_msgs::action::TargetMode_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TargetMode_FeedbackMessage_feedback(msg_);
  }

private:
  ::orca_msgs::action::TargetMode_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::orca_msgs::action::TargetMode_FeedbackMessage>()
{
  return orca_msgs::action::builder::Init_TargetMode_FeedbackMessage_goal_id();
}

}  // namespace orca_msgs

#endif  // ORCA_MSGS__ACTION__DETAIL__TARGET_MODE__BUILDER_HPP_

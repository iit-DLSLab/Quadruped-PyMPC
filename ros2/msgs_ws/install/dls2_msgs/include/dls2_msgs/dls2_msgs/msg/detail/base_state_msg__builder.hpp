// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dls2_msgs:msg/BaseStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__BUILDER_HPP_
#define DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dls2_msgs/msg/detail/base_state_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dls2_msgs
{

namespace msg
{

namespace builder
{

class Init_BaseStateMsg_stance_status
{
public:
  explicit Init_BaseStateMsg_stance_status(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  ::dls2_msgs::msg::BaseStateMsg stance_status(::dls2_msgs::msg::BaseStateMsg::_stance_status_type arg)
  {
    msg_.stance_status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_angular_acceleration
{
public:
  explicit Init_BaseStateMsg_angular_acceleration(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_stance_status angular_acceleration(::dls2_msgs::msg::BaseStateMsg::_angular_acceleration_type arg)
  {
    msg_.angular_acceleration = std::move(arg);
    return Init_BaseStateMsg_stance_status(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_linear_acceleration
{
public:
  explicit Init_BaseStateMsg_linear_acceleration(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_angular_acceleration linear_acceleration(::dls2_msgs::msg::BaseStateMsg::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_BaseStateMsg_angular_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_angular_velocity
{
public:
  explicit Init_BaseStateMsg_angular_velocity(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_linear_acceleration angular_velocity(::dls2_msgs::msg::BaseStateMsg::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_BaseStateMsg_linear_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_linear_velocity
{
public:
  explicit Init_BaseStateMsg_linear_velocity(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_angular_velocity linear_velocity(::dls2_msgs::msg::BaseStateMsg::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_BaseStateMsg_angular_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_orientation
{
public:
  explicit Init_BaseStateMsg_orientation(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_linear_velocity orientation(::dls2_msgs::msg::BaseStateMsg::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_BaseStateMsg_linear_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_position
{
public:
  explicit Init_BaseStateMsg_position(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_orientation position(::dls2_msgs::msg::BaseStateMsg::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_BaseStateMsg_orientation(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_robot_name
{
public:
  explicit Init_BaseStateMsg_robot_name(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_position robot_name(::dls2_msgs::msg::BaseStateMsg::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_BaseStateMsg_position(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_timestamp
{
public:
  explicit Init_BaseStateMsg_timestamp(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_robot_name timestamp(::dls2_msgs::msg::BaseStateMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_BaseStateMsg_robot_name(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_sequence_id
{
public:
  explicit Init_BaseStateMsg_sequence_id(::dls2_msgs::msg::BaseStateMsg & msg)
  : msg_(msg)
  {}
  Init_BaseStateMsg_timestamp sequence_id(::dls2_msgs::msg::BaseStateMsg::_sequence_id_type arg)
  {
    msg_.sequence_id = std::move(arg);
    return Init_BaseStateMsg_timestamp(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

class Init_BaseStateMsg_frame_id
{
public:
  Init_BaseStateMsg_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BaseStateMsg_sequence_id frame_id(::dls2_msgs::msg::BaseStateMsg::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_BaseStateMsg_sequence_id(msg_);
  }

private:
  ::dls2_msgs::msg::BaseStateMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dls2_msgs::msg::BaseStateMsg>()
{
  return dls2_msgs::msg::builder::Init_BaseStateMsg_frame_id();
}

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__BUILDER_HPP_

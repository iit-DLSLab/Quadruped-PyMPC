// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__BUILDER_HPP_
#define DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dls2_msgs/msg/detail/blind_state_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dls2_msgs
{

namespace msg
{

namespace builder
{

class Init_BlindStateMsg_current_feet_positions
{
public:
  explicit Init_BlindStateMsg_current_feet_positions(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  ::dls2_msgs::msg::BlindStateMsg current_feet_positions(::dls2_msgs::msg::BlindStateMsg::_current_feet_positions_type arg)
  {
    msg_.current_feet_positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_feet_contact
{
public:
  explicit Init_BlindStateMsg_feet_contact(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_current_feet_positions feet_contact(::dls2_msgs::msg::BlindStateMsg::_feet_contact_type arg)
  {
    msg_.feet_contact = std::move(arg);
    return Init_BlindStateMsg_current_feet_positions(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_temperature
{
public:
  explicit Init_BlindStateMsg_joints_temperature(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_feet_contact joints_temperature(::dls2_msgs::msg::BlindStateMsg::_joints_temperature_type arg)
  {
    msg_.joints_temperature = std::move(arg);
    return Init_BlindStateMsg_feet_contact(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_effort
{
public:
  explicit Init_BlindStateMsg_joints_effort(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_temperature joints_effort(::dls2_msgs::msg::BlindStateMsg::_joints_effort_type arg)
  {
    msg_.joints_effort = std::move(arg);
    return Init_BlindStateMsg_joints_temperature(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_acceleration
{
public:
  explicit Init_BlindStateMsg_joints_acceleration(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_effort joints_acceleration(::dls2_msgs::msg::BlindStateMsg::_joints_acceleration_type arg)
  {
    msg_.joints_acceleration = std::move(arg);
    return Init_BlindStateMsg_joints_effort(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_velocity
{
public:
  explicit Init_BlindStateMsg_joints_velocity(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_acceleration joints_velocity(::dls2_msgs::msg::BlindStateMsg::_joints_velocity_type arg)
  {
    msg_.joints_velocity = std::move(arg);
    return Init_BlindStateMsg_joints_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_position
{
public:
  explicit Init_BlindStateMsg_joints_position(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_velocity joints_position(::dls2_msgs::msg::BlindStateMsg::_joints_position_type arg)
  {
    msg_.joints_position = std::move(arg);
    return Init_BlindStateMsg_joints_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_joints_name
{
public:
  explicit Init_BlindStateMsg_joints_name(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_position joints_name(::dls2_msgs::msg::BlindStateMsg::_joints_name_type arg)
  {
    msg_.joints_name = std::move(arg);
    return Init_BlindStateMsg_joints_position(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_robot_name
{
public:
  explicit Init_BlindStateMsg_robot_name(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_joints_name robot_name(::dls2_msgs::msg::BlindStateMsg::_robot_name_type arg)
  {
    msg_.robot_name = std::move(arg);
    return Init_BlindStateMsg_joints_name(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_timestamp
{
public:
  explicit Init_BlindStateMsg_timestamp(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_robot_name timestamp(::dls2_msgs::msg::BlindStateMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_BlindStateMsg_robot_name(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_sequence_id
{
public:
  explicit Init_BlindStateMsg_sequence_id(::dls2_msgs::msg::BlindStateMsg & msg)
  : msg_(msg)
  {}
  Init_BlindStateMsg_timestamp sequence_id(::dls2_msgs::msg::BlindStateMsg::_sequence_id_type arg)
  {
    msg_.sequence_id = std::move(arg);
    return Init_BlindStateMsg_timestamp(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

class Init_BlindStateMsg_frame_id
{
public:
  Init_BlindStateMsg_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BlindStateMsg_sequence_id frame_id(::dls2_msgs::msg::BlindStateMsg::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_BlindStateMsg_sequence_id(msg_);
  }

private:
  ::dls2_msgs::msg::BlindStateMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dls2_msgs::msg::BlindStateMsg>()
{
  return dls2_msgs::msg::builder::Init_BlindStateMsg_frame_id();
}

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__BUILDER_HPP_

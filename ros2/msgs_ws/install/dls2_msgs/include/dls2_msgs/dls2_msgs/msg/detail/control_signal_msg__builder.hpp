// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dls2_msgs:msg/ControlSignalMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__BUILDER_HPP_
#define DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dls2_msgs/msg/detail/control_signal_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dls2_msgs
{

namespace msg
{

namespace builder
{

class Init_ControlSignalMsg_signal_reconstruction_method
{
public:
  explicit Init_ControlSignalMsg_signal_reconstruction_method(::dls2_msgs::msg::ControlSignalMsg & msg)
  : msg_(msg)
  {}
  ::dls2_msgs::msg::ControlSignalMsg signal_reconstruction_method(::dls2_msgs::msg::ControlSignalMsg::_signal_reconstruction_method_type arg)
  {
    msg_.signal_reconstruction_method = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dls2_msgs::msg::ControlSignalMsg msg_;
};

class Init_ControlSignalMsg_torques
{
public:
  explicit Init_ControlSignalMsg_torques(::dls2_msgs::msg::ControlSignalMsg & msg)
  : msg_(msg)
  {}
  Init_ControlSignalMsg_signal_reconstruction_method torques(::dls2_msgs::msg::ControlSignalMsg::_torques_type arg)
  {
    msg_.torques = std::move(arg);
    return Init_ControlSignalMsg_signal_reconstruction_method(msg_);
  }

private:
  ::dls2_msgs::msg::ControlSignalMsg msg_;
};

class Init_ControlSignalMsg_timestamp
{
public:
  explicit Init_ControlSignalMsg_timestamp(::dls2_msgs::msg::ControlSignalMsg & msg)
  : msg_(msg)
  {}
  Init_ControlSignalMsg_torques timestamp(::dls2_msgs::msg::ControlSignalMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_ControlSignalMsg_torques(msg_);
  }

private:
  ::dls2_msgs::msg::ControlSignalMsg msg_;
};

class Init_ControlSignalMsg_sequence_id
{
public:
  explicit Init_ControlSignalMsg_sequence_id(::dls2_msgs::msg::ControlSignalMsg & msg)
  : msg_(msg)
  {}
  Init_ControlSignalMsg_timestamp sequence_id(::dls2_msgs::msg::ControlSignalMsg::_sequence_id_type arg)
  {
    msg_.sequence_id = std::move(arg);
    return Init_ControlSignalMsg_timestamp(msg_);
  }

private:
  ::dls2_msgs::msg::ControlSignalMsg msg_;
};

class Init_ControlSignalMsg_frame_id
{
public:
  Init_ControlSignalMsg_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlSignalMsg_sequence_id frame_id(::dls2_msgs::msg::ControlSignalMsg::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_ControlSignalMsg_sequence_id(msg_);
  }

private:
  ::dls2_msgs::msg::ControlSignalMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dls2_msgs::msg::ControlSignalMsg>()
{
  return dls2_msgs::msg::builder::Init_ControlSignalMsg_frame_id();
}

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__BUILDER_HPP_

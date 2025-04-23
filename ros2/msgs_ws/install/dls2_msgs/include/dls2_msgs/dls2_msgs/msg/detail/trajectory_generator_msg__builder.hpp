// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__BUILDER_HPP_
#define DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dls2_msgs/msg/detail/trajectory_generator_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dls2_msgs
{

namespace msg
{

namespace builder
{

class Init_TrajectoryGeneratorMsg_kd
{
public:
  explicit Init_TrajectoryGeneratorMsg_kd(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  ::dls2_msgs::msg::TrajectoryGeneratorMsg kd(::dls2_msgs::msg::TrajectoryGeneratorMsg::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_kp
{
public:
  explicit Init_TrajectoryGeneratorMsg_kp(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_kd kp(::dls2_msgs::msg::TrajectoryGeneratorMsg::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_TrajectoryGeneratorMsg_kd(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_normal_force_min
{
public:
  explicit Init_TrajectoryGeneratorMsg_normal_force_min(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_kp normal_force_min(::dls2_msgs::msg::TrajectoryGeneratorMsg::_normal_force_min_type arg)
  {
    msg_.normal_force_min = std::move(arg);
    return Init_TrajectoryGeneratorMsg_kp(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_normal_force_max
{
public:
  explicit Init_TrajectoryGeneratorMsg_normal_force_max(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_normal_force_min normal_force_max(::dls2_msgs::msg::TrajectoryGeneratorMsg::_normal_force_max_type arg)
  {
    msg_.normal_force_max = std::move(arg);
    return Init_TrajectoryGeneratorMsg_normal_force_min(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_swing_period
{
public:
  explicit Init_TrajectoryGeneratorMsg_swing_period(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_normal_force_max swing_period(::dls2_msgs::msg::TrajectoryGeneratorMsg::_swing_period_type arg)
  {
    msg_.swing_period = std::move(arg);
    return Init_TrajectoryGeneratorMsg_normal_force_max(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_touch_down
{
public:
  explicit Init_TrajectoryGeneratorMsg_touch_down(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_swing_period touch_down(::dls2_msgs::msg::TrajectoryGeneratorMsg::_touch_down_type arg)
  {
    msg_.touch_down = std::move(arg);
    return Init_TrajectoryGeneratorMsg_swing_period(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_nominal_touch_down
{
public:
  explicit Init_TrajectoryGeneratorMsg_nominal_touch_down(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_touch_down nominal_touch_down(::dls2_msgs::msg::TrajectoryGeneratorMsg::_nominal_touch_down_type arg)
  {
    msg_.nominal_touch_down = std::move(arg);
    return Init_TrajectoryGeneratorMsg_touch_down(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_stance_legs
{
public:
  explicit Init_TrajectoryGeneratorMsg_stance_legs(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_nominal_touch_down stance_legs(::dls2_msgs::msg::TrajectoryGeneratorMsg::_stance_legs_type arg)
  {
    msg_.stance_legs = std::move(arg);
    return Init_TrajectoryGeneratorMsg_nominal_touch_down(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_wrench
{
public:
  explicit Init_TrajectoryGeneratorMsg_wrench(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_stance_legs wrench(::dls2_msgs::msg::TrajectoryGeneratorMsg::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return Init_TrajectoryGeneratorMsg_stance_legs(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_joints_effort
{
public:
  explicit Init_TrajectoryGeneratorMsg_joints_effort(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_wrench joints_effort(::dls2_msgs::msg::TrajectoryGeneratorMsg::_joints_effort_type arg)
  {
    msg_.joints_effort = std::move(arg);
    return Init_TrajectoryGeneratorMsg_wrench(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_joints_acceleration
{
public:
  explicit Init_TrajectoryGeneratorMsg_joints_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_joints_effort joints_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg::_joints_acceleration_type arg)
  {
    msg_.joints_acceleration = std::move(arg);
    return Init_TrajectoryGeneratorMsg_joints_effort(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_joints_velocity
{
public:
  explicit Init_TrajectoryGeneratorMsg_joints_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_joints_acceleration joints_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg::_joints_velocity_type arg)
  {
    msg_.joints_velocity = std::move(arg);
    return Init_TrajectoryGeneratorMsg_joints_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_joints_position
{
public:
  explicit Init_TrajectoryGeneratorMsg_joints_position(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_joints_velocity joints_position(::dls2_msgs::msg::TrajectoryGeneratorMsg::_joints_position_type arg)
  {
    msg_.joints_position = std::move(arg);
    return Init_TrajectoryGeneratorMsg_joints_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_angular_acceleration
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_angular_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_joints_position com_angular_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_angular_acceleration_type arg)
  {
    msg_.com_angular_acceleration = std::move(arg);
    return Init_TrajectoryGeneratorMsg_joints_position(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_linear_acceleration
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_linear_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_angular_acceleration com_linear_acceleration(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_linear_acceleration_type arg)
  {
    msg_.com_linear_acceleration = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_angular_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_angular_velocity
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_angular_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_linear_acceleration com_angular_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_angular_velocity_type arg)
  {
    msg_.com_angular_velocity = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_linear_acceleration(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_linear_velocity
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_linear_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_angular_velocity com_linear_velocity(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_linear_velocity_type arg)
  {
    msg_.com_linear_velocity = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_angular_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_orientation
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_orientation(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_linear_velocity com_orientation(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_orientation_type arg)
  {
    msg_.com_orientation = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_linear_velocity(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_com_position
{
public:
  explicit Init_TrajectoryGeneratorMsg_com_position(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_orientation com_position(::dls2_msgs::msg::TrajectoryGeneratorMsg::_com_position_type arg)
  {
    msg_.com_position = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_orientation(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_timestamp
{
public:
  explicit Init_TrajectoryGeneratorMsg_timestamp(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_com_position timestamp(::dls2_msgs::msg::TrajectoryGeneratorMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_TrajectoryGeneratorMsg_com_position(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_sequence_id
{
public:
  explicit Init_TrajectoryGeneratorMsg_sequence_id(::dls2_msgs::msg::TrajectoryGeneratorMsg & msg)
  : msg_(msg)
  {}
  Init_TrajectoryGeneratorMsg_timestamp sequence_id(::dls2_msgs::msg::TrajectoryGeneratorMsg::_sequence_id_type arg)
  {
    msg_.sequence_id = std::move(arg);
    return Init_TrajectoryGeneratorMsg_timestamp(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

class Init_TrajectoryGeneratorMsg_frame_id
{
public:
  Init_TrajectoryGeneratorMsg_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryGeneratorMsg_sequence_id frame_id(::dls2_msgs::msg::TrajectoryGeneratorMsg::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_TrajectoryGeneratorMsg_sequence_id(msg_);
  }

private:
  ::dls2_msgs::msg::TrajectoryGeneratorMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dls2_msgs::msg::TrajectoryGeneratorMsg>()
{
  return dls2_msgs::msg::builder::Init_TrajectoryGeneratorMsg_frame_id();
}

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__BUILDER_HPP_

// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dls2_msgs/msg/detail/trajectory_generator_msg__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dls2_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TrajectoryGeneratorMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dls2_msgs::msg::TrajectoryGeneratorMsg(_init);
}

void TrajectoryGeneratorMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dls2_msgs::msg::TrajectoryGeneratorMsg *>(message_memory);
  typed_message->~TrajectoryGeneratorMsg();
}

size_t size_function__TrajectoryGeneratorMsg__com_position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__com_orientation(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_orientation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_orientation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_orientation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_orientation(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_orientation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_orientation(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__com_linear_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_linear_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_linear_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_linear_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_linear_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_linear_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_linear_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__com_angular_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_angular_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_angular_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_angular_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_angular_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_angular_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_angular_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__com_linear_acceleration(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_linear_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_linear_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__com_angular_acceleration(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__com_angular_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__com_angular_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__joints_position(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__joints_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__joints_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__joints_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__joints_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__joints_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__joints_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__joints_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__joints_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__joints_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__joints_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__joints_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__joints_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__joints_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__joints_acceleration(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__joints_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__joints_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__joints_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__joints_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__joints_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__joints_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__joints_effort(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__joints_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__joints_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__joints_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__joints_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__joints_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__joints_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__wrench(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TrajectoryGeneratorMsg__wrench(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__wrench(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__wrench(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__wrench(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__wrench(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__wrench(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__stance_legs(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__TrajectoryGeneratorMsg__stance_legs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__stance_legs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__stance_legs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__TrajectoryGeneratorMsg__stance_legs(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__stance_legs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__TrajectoryGeneratorMsg__stance_legs(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__nominal_touch_down(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__nominal_touch_down(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__nominal_touch_down(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__nominal_touch_down(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__nominal_touch_down(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__nominal_touch_down(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__nominal_touch_down(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__touch_down(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__touch_down(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__touch_down(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__touch_down(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__touch_down(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__touch_down(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__touch_down(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__swing_period(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__TrajectoryGeneratorMsg__swing_period(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__swing_period(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__swing_period(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__swing_period(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__swing_period(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__swing_period(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__normal_force_max(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__TrajectoryGeneratorMsg__normal_force_max(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__normal_force_max(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__normal_force_max(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__normal_force_max(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__normal_force_max(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__normal_force_max(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__normal_force_min(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__TrajectoryGeneratorMsg__normal_force_min(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__normal_force_min(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__normal_force_min(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__normal_force_min(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__normal_force_min(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__normal_force_min(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__kp(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__kp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__kp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__kp(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__kp(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TrajectoryGeneratorMsg__kd(const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * get_const_function__TrajectoryGeneratorMsg__kd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryGeneratorMsg__kd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 12> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryGeneratorMsg__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TrajectoryGeneratorMsg__kd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryGeneratorMsg__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TrajectoryGeneratorMsg__kd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrajectoryGeneratorMsg_message_member_array[22] = {
  {
    "frame_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, frame_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sequence_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, sequence_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_position,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_position,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_position,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_position,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_orientation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_orientation),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_orientation,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_orientation,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_orientation,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_orientation,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_orientation,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_linear_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_linear_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_linear_velocity,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_linear_velocity,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_linear_velocity,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_linear_velocity,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_linear_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_angular_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_angular_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_angular_velocity,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_angular_velocity,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_angular_velocity,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_angular_velocity,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_angular_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_linear_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_linear_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "com_angular_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, com_angular_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joints_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, joints_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__joints_position,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__joints_position,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__joints_position,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__joints_position,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__joints_position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joints_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, joints_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__joints_velocity,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__joints_velocity,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__joints_velocity,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__joints_velocity,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__joints_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joints_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, joints_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__joints_acceleration,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__joints_acceleration,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__joints_acceleration,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__joints_acceleration,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__joints_acceleration,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joints_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, joints_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__joints_effort,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__joints_effort,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__joints_effort,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__joints_effort,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__joints_effort,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "wrench",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, wrench),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__wrench,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__wrench,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__wrench,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__wrench,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__wrench,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stance_legs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, stance_legs),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__stance_legs,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__stance_legs,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__stance_legs,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__stance_legs,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__stance_legs,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "nominal_touch_down",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, nominal_touch_down),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__nominal_touch_down,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__nominal_touch_down,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__nominal_touch_down,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__nominal_touch_down,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__nominal_touch_down,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "touch_down",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, touch_down),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__touch_down,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__touch_down,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__touch_down,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__touch_down,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__touch_down,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "swing_period",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, swing_period),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__swing_period,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__swing_period,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__swing_period,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__swing_period,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__swing_period,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "normal_force_max",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, normal_force_max),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__normal_force_max,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__normal_force_max,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__normal_force_max,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__normal_force_max,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__normal_force_max,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "normal_force_min",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, normal_force_min),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__normal_force_min,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__normal_force_min,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__normal_force_min,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__normal_force_min,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__normal_force_min,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, kp),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__kp,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__kp,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__kp,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__kp,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__kp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::TrajectoryGeneratorMsg, kd),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryGeneratorMsg__kd,  // size() function pointer
    get_const_function__TrajectoryGeneratorMsg__kd,  // get_const(index) function pointer
    get_function__TrajectoryGeneratorMsg__kd,  // get(index) function pointer
    fetch_function__TrajectoryGeneratorMsg__kd,  // fetch(index, &value) function pointer
    assign_function__TrajectoryGeneratorMsg__kd,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrajectoryGeneratorMsg_message_members = {
  "dls2_msgs::msg",  // message namespace
  "TrajectoryGeneratorMsg",  // message name
  22,  // number of fields
  sizeof(dls2_msgs::msg::TrajectoryGeneratorMsg),
  TrajectoryGeneratorMsg_message_member_array,  // message members
  TrajectoryGeneratorMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  TrajectoryGeneratorMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrajectoryGeneratorMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrajectoryGeneratorMsg_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dls2_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dls2_msgs::msg::TrajectoryGeneratorMsg>()
{
  return &::dls2_msgs::msg::rosidl_typesupport_introspection_cpp::TrajectoryGeneratorMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dls2_msgs, msg, TrajectoryGeneratorMsg)() {
  return &::dls2_msgs::msg::rosidl_typesupport_introspection_cpp::TrajectoryGeneratorMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dls2_msgs/msg/detail/trajectory_generator_msg__rosidl_typesupport_introspection_c.h"
#include "dls2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dls2_msgs/msg/detail/trajectory_generator_msg__functions.h"
#include "dls2_msgs/msg/detail/trajectory_generator_msg__struct.h"


// Include directives for member types
// Member `frame_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dls2_msgs__msg__TrajectoryGeneratorMsg__init(message_memory);
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_fini_function(void * message_memory)
{
  dls2_msgs__msg__TrajectoryGeneratorMsg__fini(message_memory);
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_orientation(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_orientation(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_orientation(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_orientation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_orientation(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_orientation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_orientation(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_linear_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_linear_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_linear_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_angular_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_angular_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_angular_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_linear_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_angular_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_acceleration(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_acceleration(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_acceleration(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_effort(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_effort(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_effort(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__wrench(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__wrench(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__wrench(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__wrench(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__wrench(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__wrench(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__wrench(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__stance_legs(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__stance_legs(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__stance_legs(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__stance_legs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__stance_legs(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__stance_legs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__stance_legs(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__nominal_touch_down(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__nominal_touch_down(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__nominal_touch_down(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__nominal_touch_down(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__nominal_touch_down(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__nominal_touch_down(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__nominal_touch_down(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__touch_down(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__touch_down(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__touch_down(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__touch_down(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__touch_down(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__touch_down(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__touch_down(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__swing_period(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__swing_period(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__swing_period(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__swing_period(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__swing_period(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__swing_period(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__swing_period(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__normal_force_max(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_max(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_max(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__normal_force_max(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_max(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__normal_force_max(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_max(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__normal_force_min(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_min(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_min(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__normal_force_min(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_min(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__normal_force_min(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_min(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__kp(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kp(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kp(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kp(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kp(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__kd(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kd(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kd(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kd(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kd(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_member_array[22] = {
  {
    "frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, frame_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sequence_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, sequence_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_position),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_position,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_position,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_position,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_position,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_orientation",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_orientation),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_orientation,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_orientation,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_orientation,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_orientation,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_orientation,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_linear_velocity),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_linear_velocity,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_velocity,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_velocity,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_linear_velocity,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_linear_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_angular_velocity),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_angular_velocity,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_velocity,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_velocity,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_angular_velocity,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_angular_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_linear_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_linear_acceleration),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_linear_acceleration,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "com_angular_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, com_angular_acceleration),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__com_angular_acceleration,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, joints_position),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_position,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_position,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_position,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_position,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, joints_velocity),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_velocity,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_velocity,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_velocity,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_velocity,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, joints_acceleration),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_acceleration,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_acceleration,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_acceleration,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_acceleration,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_acceleration,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, joints_effort),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__joints_effort,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__joints_effort,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__joints_effort,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__joints_effort,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__joints_effort,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, wrench),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__wrench,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__wrench,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__wrench,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__wrench,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__wrench,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stance_legs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, stance_legs),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__stance_legs,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__stance_legs,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__stance_legs,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__stance_legs,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__stance_legs,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nominal_touch_down",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, nominal_touch_down),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__nominal_touch_down,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__nominal_touch_down,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__nominal_touch_down,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__nominal_touch_down,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__nominal_touch_down,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "touch_down",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, touch_down),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__touch_down,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__touch_down,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__touch_down,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__touch_down,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__touch_down,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "swing_period",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, swing_period),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__swing_period,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__swing_period,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__swing_period,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__swing_period,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__swing_period,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "normal_force_max",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, normal_force_max),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__normal_force_max,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_max,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_max,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__normal_force_max,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__normal_force_max,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "normal_force_min",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, normal_force_min),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__normal_force_min,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__normal_force_min,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__normal_force_min,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__normal_force_min,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__normal_force_min,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, kp),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__kp,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kp,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kp,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__kp,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__kp,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__TrajectoryGeneratorMsg, kd),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__size_function__TrajectoryGeneratorMsg__kd,  // size() function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_const_function__TrajectoryGeneratorMsg__kd,  // get_const(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__get_function__TrajectoryGeneratorMsg__kd,  // get(index) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__fetch_function__TrajectoryGeneratorMsg__kd,  // fetch(index, &value) function pointer
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__assign_function__TrajectoryGeneratorMsg__kd,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_members = {
  "dls2_msgs__msg",  // message namespace
  "TrajectoryGeneratorMsg",  // message name
  22,  // number of fields
  sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg),
  dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_member_array,  // message members
  dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_type_support_handle = {
  0,
  &dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dls2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dls2_msgs, msg, TrajectoryGeneratorMsg)() {
  if (!dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_type_support_handle.typesupport_identifier) {
    dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dls2_msgs__msg__TrajectoryGeneratorMsg__rosidl_typesupport_introspection_c__TrajectoryGeneratorMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

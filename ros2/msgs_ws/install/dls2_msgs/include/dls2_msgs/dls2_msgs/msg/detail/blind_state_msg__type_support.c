// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dls2_msgs/msg/detail/blind_state_msg__rosidl_typesupport_introspection_c.h"
#include "dls2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dls2_msgs/msg/detail/blind_state_msg__functions.h"
#include "dls2_msgs/msg/detail/blind_state_msg__struct.h"


// Include directives for member types
// Member `frame_id`
// Member `robot_name`
// Member `joints_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dls2_msgs__msg__BlindStateMsg__init(message_memory);
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_fini_function(void * message_memory)
{
  dls2_msgs__msg__BlindStateMsg__fini(message_memory);
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_name(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_acceleration(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_acceleration(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_acceleration(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_effort(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_effort(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_effort(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_temperature(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_temperature(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_temperature(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_temperature(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_temperature(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_temperature(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_temperature(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__feet_contact(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__feet_contact(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__feet_contact(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__feet_contact(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__feet_contact(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__feet_contact(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__feet_contact(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__current_feet_positions(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__current_feet_positions(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__current_feet_positions(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__current_feet_positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__current_feet_positions(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__current_feet_positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__current_feet_positions(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_member_array[12] = {
  {
    "frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, frame_id),  // bytes offset in struct
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
    offsetof(dls2_msgs__msg__BlindStateMsg, sequence_id),  // bytes offset in struct
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
    offsetof(dls2_msgs__msg__BlindStateMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, robot_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_name),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_name,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_name,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_name,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_name,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_name,  // assign(index, value) function pointer
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
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_position),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_position,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_position,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_position,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_position,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_position,  // assign(index, value) function pointer
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
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_velocity),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_velocity,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_velocity,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_velocity,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_velocity,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_velocity,  // assign(index, value) function pointer
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
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_acceleration),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_acceleration,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_acceleration,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_acceleration,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_acceleration,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_acceleration,  // assign(index, value) function pointer
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
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_effort),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_effort,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_effort,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_effort,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_effort,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_effort,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joints_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, joints_temperature),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__joints_temperature,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__joints_temperature,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__joints_temperature,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__joints_temperature,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__joints_temperature,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feet_contact",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, feet_contact),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__feet_contact,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__feet_contact,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__feet_contact,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__feet_contact,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__feet_contact,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current_feet_positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__BlindStateMsg, current_feet_positions),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__size_function__BlindStateMsg__current_feet_positions,  // size() function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_const_function__BlindStateMsg__current_feet_positions,  // get_const(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__get_function__BlindStateMsg__current_feet_positions,  // get(index) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__fetch_function__BlindStateMsg__current_feet_positions,  // fetch(index, &value) function pointer
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__assign_function__BlindStateMsg__current_feet_positions,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_members = {
  "dls2_msgs__msg",  // message namespace
  "BlindStateMsg",  // message name
  12,  // number of fields
  sizeof(dls2_msgs__msg__BlindStateMsg),
  dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_member_array,  // message members
  dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_type_support_handle = {
  0,
  &dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dls2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dls2_msgs, msg, BlindStateMsg)() {
  if (!dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_type_support_handle.typesupport_identifier) {
    dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dls2_msgs__msg__BlindStateMsg__rosidl_typesupport_introspection_c__BlindStateMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

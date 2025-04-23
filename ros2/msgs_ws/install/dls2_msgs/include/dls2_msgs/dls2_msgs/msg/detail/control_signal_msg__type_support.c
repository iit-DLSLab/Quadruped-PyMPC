// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dls2_msgs:msg/ControlSignalMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dls2_msgs/msg/detail/control_signal_msg__rosidl_typesupport_introspection_c.h"
#include "dls2_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dls2_msgs/msg/detail/control_signal_msg__functions.h"
#include "dls2_msgs/msg/detail/control_signal_msg__struct.h"


// Include directives for member types
// Member `frame_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dls2_msgs__msg__ControlSignalMsg__init(message_memory);
}

void dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_fini_function(void * message_memory)
{
  dls2_msgs__msg__ControlSignalMsg__fini(message_memory);
}

size_t dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__size_function__ControlSignalMsg__torques(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_const_function__ControlSignalMsg__torques(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_function__ControlSignalMsg__torques(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__fetch_function__ControlSignalMsg__torques(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_const_function__ControlSignalMsg__torques(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__assign_function__ControlSignalMsg__torques(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_function__ControlSignalMsg__torques(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_member_array[5] = {
  {
    "frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__ControlSignalMsg, frame_id),  // bytes offset in struct
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
    offsetof(dls2_msgs__msg__ControlSignalMsg, sequence_id),  // bytes offset in struct
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
    offsetof(dls2_msgs__msg__ControlSignalMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "torques",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__ControlSignalMsg, torques),  // bytes offset in struct
    NULL,  // default value
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__size_function__ControlSignalMsg__torques,  // size() function pointer
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_const_function__ControlSignalMsg__torques,  // get_const(index) function pointer
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__get_function__ControlSignalMsg__torques,  // get(index) function pointer
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__fetch_function__ControlSignalMsg__torques,  // fetch(index, &value) function pointer
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__assign_function__ControlSignalMsg__torques,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "signal_reconstruction_method",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs__msg__ControlSignalMsg, signal_reconstruction_method),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_members = {
  "dls2_msgs__msg",  // message namespace
  "ControlSignalMsg",  // message name
  5,  // number of fields
  sizeof(dls2_msgs__msg__ControlSignalMsg),
  dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_member_array,  // message members
  dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_type_support_handle = {
  0,
  &dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dls2_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dls2_msgs, msg, ControlSignalMsg)() {
  if (!dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_type_support_handle.typesupport_identifier) {
    dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dls2_msgs__msg__ControlSignalMsg__rosidl_typesupport_introspection_c__ControlSignalMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

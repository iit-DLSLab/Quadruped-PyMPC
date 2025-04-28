// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dls2_msgs:msg/BaseStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_H_
#define DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame_id'
// Member 'robot_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BaseStateMsg in the package dls2_msgs.
/**
  * Header
 */
typedef struct dls2_msgs__msg__BaseStateMsg
{
  rosidl_runtime_c__String frame_id;
  uint32_t sequence_id;
  double timestamp;
  rosidl_runtime_c__String robot_name;
  double position[3];
  double orientation[4];
  /// Base velocity
  double linear_velocity[3];
  double angular_velocity[3];
  /// Base acceleration
  double linear_acceleration[3];
  double angular_acceleration[3];
  /// Stance status
  double stance_status[4];
} dls2_msgs__msg__BaseStateMsg;

// Struct for a sequence of dls2_msgs__msg__BaseStateMsg.
typedef struct dls2_msgs__msg__BaseStateMsg__Sequence
{
  dls2_msgs__msg__BaseStateMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dls2_msgs__msg__BaseStateMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_H_

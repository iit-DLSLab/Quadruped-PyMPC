// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_H_
#define DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_H_

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
// Member 'joints_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BlindStateMsg in the package dls2_msgs.
/**
  * Header
 */
typedef struct dls2_msgs__msg__BlindStateMsg
{
  rosidl_runtime_c__String frame_id;
  uint32_t sequence_id;
  double timestamp;
  rosidl_runtime_c__String robot_name;
  rosidl_runtime_c__String joints_name[12];
  double joints_position[12];
  double joints_velocity[12];
  double joints_acceleration[12];
  double joints_effort[12];
  double joints_temperature[12];
  double feet_contact[4];
  double current_feet_positions[12];
} dls2_msgs__msg__BlindStateMsg;

// Struct for a sequence of dls2_msgs__msg__BlindStateMsg.
typedef struct dls2_msgs__msg__BlindStateMsg__Sequence
{
  dls2_msgs__msg__BlindStateMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dls2_msgs__msg__BlindStateMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_H_
#define DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrajectoryGeneratorMsg in the package dls2_msgs.
/**
  * Header
 */
typedef struct dls2_msgs__msg__TrajectoryGeneratorMsg
{
  rosidl_runtime_c__String frame_id;
  uint32_t sequence_id;
  double timestamp;
  double com_position[3];
  double com_orientation[4];
  double com_linear_velocity[3];
  double com_angular_velocity[3];
  double com_linear_acceleration[3];
  double com_angular_acceleration[3];
  double joints_position[12];
  double joints_velocity[12];
  double joints_acceleration[12];
  double joints_effort[12];
  double wrench[6];
  bool stance_legs[4];
  double nominal_touch_down[12];
  double touch_down[12];
  double swing_period[4];
  double normal_force_max[4];
  double normal_force_min[4];
  double kp[12];
  double kd[12];
} dls2_msgs__msg__TrajectoryGeneratorMsg;

// Struct for a sequence of dls2_msgs__msg__TrajectoryGeneratorMsg.
typedef struct dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence
{
  dls2_msgs__msg__TrajectoryGeneratorMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_H_

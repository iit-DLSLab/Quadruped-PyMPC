// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dls2_msgs:msg/ControlSignalMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_H_
#define DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_H_

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

/// Struct defined in msg/ControlSignalMsg in the package dls2_msgs.
/**
  * Header
 */
typedef struct dls2_msgs__msg__ControlSignalMsg
{
  rosidl_runtime_c__String frame_id;
  uint32_t sequence_id;
  double timestamp;
  double torques[12];
  uint64_t signal_reconstruction_method;
} dls2_msgs__msg__ControlSignalMsg;

// Struct for a sequence of dls2_msgs__msg__ControlSignalMsg.
typedef struct dls2_msgs__msg__ControlSignalMsg__Sequence
{
  dls2_msgs__msg__ControlSignalMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dls2_msgs__msg__ControlSignalMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_H_

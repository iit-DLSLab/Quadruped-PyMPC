// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice
#include "dls2_msgs/msg/detail/trajectory_generator_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame_id`
#include "rosidl_runtime_c/string_functions.h"

bool
dls2_msgs__msg__TrajectoryGeneratorMsg__init(dls2_msgs__msg__TrajectoryGeneratorMsg * msg)
{
  if (!msg) {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__init(&msg->frame_id)) {
    dls2_msgs__msg__TrajectoryGeneratorMsg__fini(msg);
    return false;
  }
  // sequence_id
  // timestamp
  // com_position
  // com_orientation
  // com_linear_velocity
  // com_angular_velocity
  // com_linear_acceleration
  // com_angular_acceleration
  // joints_position
  // joints_velocity
  // joints_acceleration
  // joints_effort
  // wrench
  // stance_legs
  // nominal_touch_down
  // touch_down
  // swing_period
  // normal_force_max
  // normal_force_min
  // kp
  // kd
  return true;
}

void
dls2_msgs__msg__TrajectoryGeneratorMsg__fini(dls2_msgs__msg__TrajectoryGeneratorMsg * msg)
{
  if (!msg) {
    return;
  }
  // frame_id
  rosidl_runtime_c__String__fini(&msg->frame_id);
  // sequence_id
  // timestamp
  // com_position
  // com_orientation
  // com_linear_velocity
  // com_angular_velocity
  // com_linear_acceleration
  // com_angular_acceleration
  // joints_position
  // joints_velocity
  // joints_acceleration
  // joints_effort
  // wrench
  // stance_legs
  // nominal_touch_down
  // touch_down
  // swing_period
  // normal_force_max
  // normal_force_min
  // kp
  // kd
}

bool
dls2_msgs__msg__TrajectoryGeneratorMsg__are_equal(const dls2_msgs__msg__TrajectoryGeneratorMsg * lhs, const dls2_msgs__msg__TrajectoryGeneratorMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame_id), &(rhs->frame_id)))
  {
    return false;
  }
  // sequence_id
  if (lhs->sequence_id != rhs->sequence_id) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // com_position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->com_position[i] != rhs->com_position[i]) {
      return false;
    }
  }
  // com_orientation
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->com_orientation[i] != rhs->com_orientation[i]) {
      return false;
    }
  }
  // com_linear_velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->com_linear_velocity[i] != rhs->com_linear_velocity[i]) {
      return false;
    }
  }
  // com_angular_velocity
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->com_angular_velocity[i] != rhs->com_angular_velocity[i]) {
      return false;
    }
  }
  // com_linear_acceleration
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->com_linear_acceleration[i] != rhs->com_linear_acceleration[i]) {
      return false;
    }
  }
  // com_angular_acceleration
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->com_angular_acceleration[i] != rhs->com_angular_acceleration[i]) {
      return false;
    }
  }
  // joints_position
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->joints_position[i] != rhs->joints_position[i]) {
      return false;
    }
  }
  // joints_velocity
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->joints_velocity[i] != rhs->joints_velocity[i]) {
      return false;
    }
  }
  // joints_acceleration
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->joints_acceleration[i] != rhs->joints_acceleration[i]) {
      return false;
    }
  }
  // joints_effort
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->joints_effort[i] != rhs->joints_effort[i]) {
      return false;
    }
  }
  // wrench
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->wrench[i] != rhs->wrench[i]) {
      return false;
    }
  }
  // stance_legs
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->stance_legs[i] != rhs->stance_legs[i]) {
      return false;
    }
  }
  // nominal_touch_down
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->nominal_touch_down[i] != rhs->nominal_touch_down[i]) {
      return false;
    }
  }
  // touch_down
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->touch_down[i] != rhs->touch_down[i]) {
      return false;
    }
  }
  // swing_period
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->swing_period[i] != rhs->swing_period[i]) {
      return false;
    }
  }
  // normal_force_max
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->normal_force_max[i] != rhs->normal_force_max[i]) {
      return false;
    }
  }
  // normal_force_min
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->normal_force_min[i] != rhs->normal_force_min[i]) {
      return false;
    }
  }
  // kp
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->kp[i] != rhs->kp[i]) {
      return false;
    }
  }
  // kd
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->kd[i] != rhs->kd[i]) {
      return false;
    }
  }
  return true;
}

bool
dls2_msgs__msg__TrajectoryGeneratorMsg__copy(
  const dls2_msgs__msg__TrajectoryGeneratorMsg * input,
  dls2_msgs__msg__TrajectoryGeneratorMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__copy(
      &(input->frame_id), &(output->frame_id)))
  {
    return false;
  }
  // sequence_id
  output->sequence_id = input->sequence_id;
  // timestamp
  output->timestamp = input->timestamp;
  // com_position
  for (size_t i = 0; i < 3; ++i) {
    output->com_position[i] = input->com_position[i];
  }
  // com_orientation
  for (size_t i = 0; i < 4; ++i) {
    output->com_orientation[i] = input->com_orientation[i];
  }
  // com_linear_velocity
  for (size_t i = 0; i < 3; ++i) {
    output->com_linear_velocity[i] = input->com_linear_velocity[i];
  }
  // com_angular_velocity
  for (size_t i = 0; i < 3; ++i) {
    output->com_angular_velocity[i] = input->com_angular_velocity[i];
  }
  // com_linear_acceleration
  for (size_t i = 0; i < 3; ++i) {
    output->com_linear_acceleration[i] = input->com_linear_acceleration[i];
  }
  // com_angular_acceleration
  for (size_t i = 0; i < 3; ++i) {
    output->com_angular_acceleration[i] = input->com_angular_acceleration[i];
  }
  // joints_position
  for (size_t i = 0; i < 12; ++i) {
    output->joints_position[i] = input->joints_position[i];
  }
  // joints_velocity
  for (size_t i = 0; i < 12; ++i) {
    output->joints_velocity[i] = input->joints_velocity[i];
  }
  // joints_acceleration
  for (size_t i = 0; i < 12; ++i) {
    output->joints_acceleration[i] = input->joints_acceleration[i];
  }
  // joints_effort
  for (size_t i = 0; i < 12; ++i) {
    output->joints_effort[i] = input->joints_effort[i];
  }
  // wrench
  for (size_t i = 0; i < 6; ++i) {
    output->wrench[i] = input->wrench[i];
  }
  // stance_legs
  for (size_t i = 0; i < 4; ++i) {
    output->stance_legs[i] = input->stance_legs[i];
  }
  // nominal_touch_down
  for (size_t i = 0; i < 12; ++i) {
    output->nominal_touch_down[i] = input->nominal_touch_down[i];
  }
  // touch_down
  for (size_t i = 0; i < 12; ++i) {
    output->touch_down[i] = input->touch_down[i];
  }
  // swing_period
  for (size_t i = 0; i < 4; ++i) {
    output->swing_period[i] = input->swing_period[i];
  }
  // normal_force_max
  for (size_t i = 0; i < 4; ++i) {
    output->normal_force_max[i] = input->normal_force_max[i];
  }
  // normal_force_min
  for (size_t i = 0; i < 4; ++i) {
    output->normal_force_min[i] = input->normal_force_min[i];
  }
  // kp
  for (size_t i = 0; i < 12; ++i) {
    output->kp[i] = input->kp[i];
  }
  // kd
  for (size_t i = 0; i < 12; ++i) {
    output->kd[i] = input->kd[i];
  }
  return true;
}

dls2_msgs__msg__TrajectoryGeneratorMsg *
dls2_msgs__msg__TrajectoryGeneratorMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__TrajectoryGeneratorMsg * msg = (dls2_msgs__msg__TrajectoryGeneratorMsg *)allocator.allocate(sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg));
  bool success = dls2_msgs__msg__TrajectoryGeneratorMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dls2_msgs__msg__TrajectoryGeneratorMsg__destroy(dls2_msgs__msg__TrajectoryGeneratorMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dls2_msgs__msg__TrajectoryGeneratorMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__init(dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__TrajectoryGeneratorMsg * data = NULL;

  if (size) {
    data = (dls2_msgs__msg__TrajectoryGeneratorMsg *)allocator.zero_allocate(size, sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dls2_msgs__msg__TrajectoryGeneratorMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dls2_msgs__msg__TrajectoryGeneratorMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__fini(dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dls2_msgs__msg__TrajectoryGeneratorMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence *
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * array = (dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence *)allocator.allocate(sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__destroy(dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__are_equal(const dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * lhs, const dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dls2_msgs__msg__TrajectoryGeneratorMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence__copy(
  const dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * input,
  dls2_msgs__msg__TrajectoryGeneratorMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dls2_msgs__msg__TrajectoryGeneratorMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dls2_msgs__msg__TrajectoryGeneratorMsg * data =
      (dls2_msgs__msg__TrajectoryGeneratorMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dls2_msgs__msg__TrajectoryGeneratorMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dls2_msgs__msg__TrajectoryGeneratorMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dls2_msgs__msg__TrajectoryGeneratorMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

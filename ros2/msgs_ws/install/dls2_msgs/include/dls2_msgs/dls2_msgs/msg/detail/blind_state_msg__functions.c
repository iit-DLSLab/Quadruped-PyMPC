// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice
#include "dls2_msgs/msg/detail/blind_state_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame_id`
// Member `robot_name`
// Member `joints_name`
#include "rosidl_runtime_c/string_functions.h"

bool
dls2_msgs__msg__BlindStateMsg__init(dls2_msgs__msg__BlindStateMsg * msg)
{
  if (!msg) {
    return false;
  }
  // frame_id
  if (!rosidl_runtime_c__String__init(&msg->frame_id)) {
    dls2_msgs__msg__BlindStateMsg__fini(msg);
    return false;
  }
  // sequence_id
  // timestamp
  // robot_name
  if (!rosidl_runtime_c__String__init(&msg->robot_name)) {
    dls2_msgs__msg__BlindStateMsg__fini(msg);
    return false;
  }
  // joints_name
  for (size_t i = 0; i < 12; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->joints_name[i])) {
      dls2_msgs__msg__BlindStateMsg__fini(msg);
      return false;
    }
  }
  // joints_position
  // joints_velocity
  // joints_acceleration
  // joints_effort
  // joints_temperature
  // feet_contact
  // current_feet_positions
  return true;
}

void
dls2_msgs__msg__BlindStateMsg__fini(dls2_msgs__msg__BlindStateMsg * msg)
{
  if (!msg) {
    return;
  }
  // frame_id
  rosidl_runtime_c__String__fini(&msg->frame_id);
  // sequence_id
  // timestamp
  // robot_name
  rosidl_runtime_c__String__fini(&msg->robot_name);
  // joints_name
  for (size_t i = 0; i < 12; ++i) {
    rosidl_runtime_c__String__fini(&msg->joints_name[i]);
  }
  // joints_position
  // joints_velocity
  // joints_acceleration
  // joints_effort
  // joints_temperature
  // feet_contact
  // current_feet_positions
}

bool
dls2_msgs__msg__BlindStateMsg__are_equal(const dls2_msgs__msg__BlindStateMsg * lhs, const dls2_msgs__msg__BlindStateMsg * rhs)
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
  // robot_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_name), &(rhs->robot_name)))
  {
    return false;
  }
  // joints_name
  for (size_t i = 0; i < 12; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->joints_name[i]), &(rhs->joints_name[i])))
    {
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
  // joints_temperature
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->joints_temperature[i] != rhs->joints_temperature[i]) {
      return false;
    }
  }
  // feet_contact
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->feet_contact[i] != rhs->feet_contact[i]) {
      return false;
    }
  }
  // current_feet_positions
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->current_feet_positions[i] != rhs->current_feet_positions[i]) {
      return false;
    }
  }
  return true;
}

bool
dls2_msgs__msg__BlindStateMsg__copy(
  const dls2_msgs__msg__BlindStateMsg * input,
  dls2_msgs__msg__BlindStateMsg * output)
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
  // robot_name
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_name), &(output->robot_name)))
  {
    return false;
  }
  // joints_name
  for (size_t i = 0; i < 12; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->joints_name[i]), &(output->joints_name[i])))
    {
      return false;
    }
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
  // joints_temperature
  for (size_t i = 0; i < 12; ++i) {
    output->joints_temperature[i] = input->joints_temperature[i];
  }
  // feet_contact
  for (size_t i = 0; i < 4; ++i) {
    output->feet_contact[i] = input->feet_contact[i];
  }
  // current_feet_positions
  for (size_t i = 0; i < 12; ++i) {
    output->current_feet_positions[i] = input->current_feet_positions[i];
  }
  return true;
}

dls2_msgs__msg__BlindStateMsg *
dls2_msgs__msg__BlindStateMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__BlindStateMsg * msg = (dls2_msgs__msg__BlindStateMsg *)allocator.allocate(sizeof(dls2_msgs__msg__BlindStateMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dls2_msgs__msg__BlindStateMsg));
  bool success = dls2_msgs__msg__BlindStateMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dls2_msgs__msg__BlindStateMsg__destroy(dls2_msgs__msg__BlindStateMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dls2_msgs__msg__BlindStateMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dls2_msgs__msg__BlindStateMsg__Sequence__init(dls2_msgs__msg__BlindStateMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__BlindStateMsg * data = NULL;

  if (size) {
    data = (dls2_msgs__msg__BlindStateMsg *)allocator.zero_allocate(size, sizeof(dls2_msgs__msg__BlindStateMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dls2_msgs__msg__BlindStateMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dls2_msgs__msg__BlindStateMsg__fini(&data[i - 1]);
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
dls2_msgs__msg__BlindStateMsg__Sequence__fini(dls2_msgs__msg__BlindStateMsg__Sequence * array)
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
      dls2_msgs__msg__BlindStateMsg__fini(&array->data[i]);
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

dls2_msgs__msg__BlindStateMsg__Sequence *
dls2_msgs__msg__BlindStateMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dls2_msgs__msg__BlindStateMsg__Sequence * array = (dls2_msgs__msg__BlindStateMsg__Sequence *)allocator.allocate(sizeof(dls2_msgs__msg__BlindStateMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dls2_msgs__msg__BlindStateMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dls2_msgs__msg__BlindStateMsg__Sequence__destroy(dls2_msgs__msg__BlindStateMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dls2_msgs__msg__BlindStateMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dls2_msgs__msg__BlindStateMsg__Sequence__are_equal(const dls2_msgs__msg__BlindStateMsg__Sequence * lhs, const dls2_msgs__msg__BlindStateMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dls2_msgs__msg__BlindStateMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dls2_msgs__msg__BlindStateMsg__Sequence__copy(
  const dls2_msgs__msg__BlindStateMsg__Sequence * input,
  dls2_msgs__msg__BlindStateMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dls2_msgs__msg__BlindStateMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dls2_msgs__msg__BlindStateMsg * data =
      (dls2_msgs__msg__BlindStateMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dls2_msgs__msg__BlindStateMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dls2_msgs__msg__BlindStateMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dls2_msgs__msg__BlindStateMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dls2_msgs:msg/BaseStateMsg.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dls2_msgs/msg/detail/base_state_msg__struct.hpp"
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

void BaseStateMsg_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dls2_msgs::msg::BaseStateMsg(_init);
}

void BaseStateMsg_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dls2_msgs::msg::BaseStateMsg *>(message_memory);
  typed_message->~BaseStateMsg();
}

size_t size_function__BaseStateMsg__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BaseStateMsg__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__orientation(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__BaseStateMsg__orientation(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__orientation(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__orientation(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__orientation(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__orientation(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__orientation(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__linear_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BaseStateMsg__linear_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__linear_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__linear_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__linear_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__linear_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__linear_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__angular_velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BaseStateMsg__angular_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__angular_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__angular_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__angular_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__angular_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__angular_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__linear_acceleration(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BaseStateMsg__linear_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__linear_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__linear_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__linear_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__linear_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__linear_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__angular_acceleration(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__BaseStateMsg__angular_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__angular_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__angular_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__angular_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__angular_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__angular_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__BaseStateMsg__stance_status(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__BaseStateMsg__stance_status(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__BaseStateMsg__stance_status(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__BaseStateMsg__stance_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__BaseStateMsg__stance_status(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__BaseStateMsg__stance_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__BaseStateMsg__stance_status(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BaseStateMsg_message_member_array[11] = {
  {
    "frame_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, frame_id),  // bytes offset in struct
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
    offsetof(dls2_msgs::msg::BaseStateMsg, sequence_id),  // bytes offset in struct
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
    offsetof(dls2_msgs::msg::BaseStateMsg, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, robot_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__position,  // size() function pointer
    get_const_function__BaseStateMsg__position,  // get_const(index) function pointer
    get_function__BaseStateMsg__position,  // get(index) function pointer
    fetch_function__BaseStateMsg__position,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "orientation",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, orientation),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__orientation,  // size() function pointer
    get_const_function__BaseStateMsg__orientation,  // get_const(index) function pointer
    get_function__BaseStateMsg__orientation,  // get(index) function pointer
    fetch_function__BaseStateMsg__orientation,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__orientation,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, linear_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__linear_velocity,  // size() function pointer
    get_const_function__BaseStateMsg__linear_velocity,  // get_const(index) function pointer
    get_function__BaseStateMsg__linear_velocity,  // get(index) function pointer
    fetch_function__BaseStateMsg__linear_velocity,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__linear_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, angular_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__angular_velocity,  // size() function pointer
    get_const_function__BaseStateMsg__angular_velocity,  // get_const(index) function pointer
    get_function__BaseStateMsg__angular_velocity,  // get(index) function pointer
    fetch_function__BaseStateMsg__angular_velocity,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__angular_velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "linear_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, linear_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__linear_acceleration,  // size() function pointer
    get_const_function__BaseStateMsg__linear_acceleration,  // get_const(index) function pointer
    get_function__BaseStateMsg__linear_acceleration,  // get(index) function pointer
    fetch_function__BaseStateMsg__linear_acceleration,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__linear_acceleration,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "angular_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, angular_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__angular_acceleration,  // size() function pointer
    get_const_function__BaseStateMsg__angular_acceleration,  // get_const(index) function pointer
    get_function__BaseStateMsg__angular_acceleration,  // get(index) function pointer
    fetch_function__BaseStateMsg__angular_acceleration,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__angular_acceleration,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stance_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(dls2_msgs::msg::BaseStateMsg, stance_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__BaseStateMsg__stance_status,  // size() function pointer
    get_const_function__BaseStateMsg__stance_status,  // get_const(index) function pointer
    get_function__BaseStateMsg__stance_status,  // get(index) function pointer
    fetch_function__BaseStateMsg__stance_status,  // fetch(index, &value) function pointer
    assign_function__BaseStateMsg__stance_status,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BaseStateMsg_message_members = {
  "dls2_msgs::msg",  // message namespace
  "BaseStateMsg",  // message name
  11,  // number of fields
  sizeof(dls2_msgs::msg::BaseStateMsg),
  BaseStateMsg_message_member_array,  // message members
  BaseStateMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  BaseStateMsg_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BaseStateMsg_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BaseStateMsg_message_members,
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
get_message_type_support_handle<dls2_msgs::msg::BaseStateMsg>()
{
  return &::dls2_msgs::msg::rosidl_typesupport_introspection_cpp::BaseStateMsg_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dls2_msgs, msg, BaseStateMsg)() {
  return &::dls2_msgs::msg::rosidl_typesupport_introspection_cpp::BaseStateMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

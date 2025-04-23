// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_HPP_
#define DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dls2_msgs__msg__BlindStateMsg __attribute__((deprecated))
#else
# define DEPRECATED__dls2_msgs__msg__BlindStateMsg __declspec(deprecated)
#endif

namespace dls2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BlindStateMsg_
{
  using Type = BlindStateMsg_<ContainerAllocator>;

  explicit BlindStateMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      this->robot_name = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 12>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->joints_name.begin(), this->joints_name.end(), "");
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_position.begin(), this->joints_position.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_velocity.begin(), this->joints_velocity.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_acceleration.begin(), this->joints_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_effort.begin(), this->joints_effort.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_temperature.begin(), this->joints_temperature.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->feet_contact.begin(), this->feet_contact.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->current_feet_positions.begin(), this->current_feet_positions.end(), 0.0);
    }
  }

  explicit BlindStateMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    robot_name(_alloc),
    joints_name(_alloc),
    joints_position(_alloc),
    joints_velocity(_alloc),
    joints_acceleration(_alloc),
    joints_effort(_alloc),
    joints_temperature(_alloc),
    feet_contact(_alloc),
    current_feet_positions(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      this->robot_name = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 12>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->joints_name.begin(), this->joints_name.end(), "");
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_position.begin(), this->joints_position.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_velocity.begin(), this->joints_velocity.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_acceleration.begin(), this->joints_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_effort.begin(), this->joints_effort.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_temperature.begin(), this->joints_temperature.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->feet_contact.begin(), this->feet_contact.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->current_feet_positions.begin(), this->current_feet_positions.end(), 0.0);
    }
  }

  // field types and members
  using _frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_id_type frame_id;
  using _sequence_id_type =
    uint32_t;
  _sequence_id_type sequence_id;
  using _timestamp_type =
    double;
  _timestamp_type timestamp;
  using _robot_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_name_type robot_name;
  using _joints_name_type =
    std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 12>;
  _joints_name_type joints_name;
  using _joints_position_type =
    std::array<double, 12>;
  _joints_position_type joints_position;
  using _joints_velocity_type =
    std::array<double, 12>;
  _joints_velocity_type joints_velocity;
  using _joints_acceleration_type =
    std::array<double, 12>;
  _joints_acceleration_type joints_acceleration;
  using _joints_effort_type =
    std::array<double, 12>;
  _joints_effort_type joints_effort;
  using _joints_temperature_type =
    std::array<double, 12>;
  _joints_temperature_type joints_temperature;
  using _feet_contact_type =
    std::array<double, 4>;
  _feet_contact_type feet_contact;
  using _current_feet_positions_type =
    std::array<double, 12>;
  _current_feet_positions_type current_feet_positions;

  // setters for named parameter idiom
  Type & set__frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame_id = _arg;
    return *this;
  }
  Type & set__sequence_id(
    const uint32_t & _arg)
  {
    this->sequence_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const double & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__robot_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_name = _arg;
    return *this;
  }
  Type & set__joints_name(
    const std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 12> & _arg)
  {
    this->joints_name = _arg;
    return *this;
  }
  Type & set__joints_position(
    const std::array<double, 12> & _arg)
  {
    this->joints_position = _arg;
    return *this;
  }
  Type & set__joints_velocity(
    const std::array<double, 12> & _arg)
  {
    this->joints_velocity = _arg;
    return *this;
  }
  Type & set__joints_acceleration(
    const std::array<double, 12> & _arg)
  {
    this->joints_acceleration = _arg;
    return *this;
  }
  Type & set__joints_effort(
    const std::array<double, 12> & _arg)
  {
    this->joints_effort = _arg;
    return *this;
  }
  Type & set__joints_temperature(
    const std::array<double, 12> & _arg)
  {
    this->joints_temperature = _arg;
    return *this;
  }
  Type & set__feet_contact(
    const std::array<double, 4> & _arg)
  {
    this->feet_contact = _arg;
    return *this;
  }
  Type & set__current_feet_positions(
    const std::array<double, 12> & _arg)
  {
    this->current_feet_positions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dls2_msgs__msg__BlindStateMsg
    std::shared_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dls2_msgs__msg__BlindStateMsg
    std::shared_ptr<dls2_msgs::msg::BlindStateMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BlindStateMsg_ & other) const
  {
    if (this->frame_id != other.frame_id) {
      return false;
    }
    if (this->sequence_id != other.sequence_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->robot_name != other.robot_name) {
      return false;
    }
    if (this->joints_name != other.joints_name) {
      return false;
    }
    if (this->joints_position != other.joints_position) {
      return false;
    }
    if (this->joints_velocity != other.joints_velocity) {
      return false;
    }
    if (this->joints_acceleration != other.joints_acceleration) {
      return false;
    }
    if (this->joints_effort != other.joints_effort) {
      return false;
    }
    if (this->joints_temperature != other.joints_temperature) {
      return false;
    }
    if (this->feet_contact != other.feet_contact) {
      return false;
    }
    if (this->current_feet_positions != other.current_feet_positions) {
      return false;
    }
    return true;
  }
  bool operator!=(const BlindStateMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BlindStateMsg_

// alias to use template instance with default allocator
using BlindStateMsg =
  dls2_msgs::msg::BlindStateMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dls2_msgs:msg/BaseStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_HPP_
#define DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dls2_msgs__msg__BaseStateMsg __attribute__((deprecated))
#else
# define DEPRECATED__dls2_msgs__msg__BaseStateMsg __declspec(deprecated)
#endif

namespace dls2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BaseStateMsg_
{
  using Type = BaseStateMsg_<ContainerAllocator>;

  explicit BaseStateMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      this->robot_name = "";
      std::fill<typename std::array<double, 3>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->orientation.begin(), this->orientation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->linear_velocity.begin(), this->linear_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->angular_velocity.begin(), this->angular_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->linear_acceleration.begin(), this->linear_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->angular_acceleration.begin(), this->angular_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->stance_status.begin(), this->stance_status.end(), 0.0);
    }
  }

  explicit BaseStateMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    robot_name(_alloc),
    position(_alloc),
    orientation(_alloc),
    linear_velocity(_alloc),
    angular_velocity(_alloc),
    linear_acceleration(_alloc),
    angular_acceleration(_alloc),
    stance_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      this->robot_name = "";
      std::fill<typename std::array<double, 3>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->orientation.begin(), this->orientation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->linear_velocity.begin(), this->linear_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->angular_velocity.begin(), this->angular_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->linear_acceleration.begin(), this->linear_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->angular_acceleration.begin(), this->angular_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->stance_status.begin(), this->stance_status.end(), 0.0);
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
  using _position_type =
    std::array<double, 3>;
  _position_type position;
  using _orientation_type =
    std::array<double, 4>;
  _orientation_type orientation;
  using _linear_velocity_type =
    std::array<double, 3>;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    std::array<double, 3>;
  _angular_velocity_type angular_velocity;
  using _linear_acceleration_type =
    std::array<double, 3>;
  _linear_acceleration_type linear_acceleration;
  using _angular_acceleration_type =
    std::array<double, 3>;
  _angular_acceleration_type angular_acceleration;
  using _stance_status_type =
    std::array<double, 4>;
  _stance_status_type stance_status;

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
  Type & set__position(
    const std::array<double, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__orientation(
    const std::array<double, 4> & _arg)
  {
    this->orientation = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const std::array<double, 3> & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const std::array<double, 3> & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__linear_acceleration(
    const std::array<double, 3> & _arg)
  {
    this->linear_acceleration = _arg;
    return *this;
  }
  Type & set__angular_acceleration(
    const std::array<double, 3> & _arg)
  {
    this->angular_acceleration = _arg;
    return *this;
  }
  Type & set__stance_status(
    const std::array<double, 4> & _arg)
  {
    this->stance_status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dls2_msgs__msg__BaseStateMsg
    std::shared_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dls2_msgs__msg__BaseStateMsg
    std::shared_ptr<dls2_msgs::msg::BaseStateMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BaseStateMsg_ & other) const
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
    if (this->position != other.position) {
      return false;
    }
    if (this->orientation != other.orientation) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->linear_acceleration != other.linear_acceleration) {
      return false;
    }
    if (this->angular_acceleration != other.angular_acceleration) {
      return false;
    }
    if (this->stance_status != other.stance_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const BaseStateMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BaseStateMsg_

// alias to use template instance with default allocator
using BaseStateMsg =
  dls2_msgs::msg::BaseStateMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__STRUCT_HPP_

// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dls2_msgs:msg/TrajectoryGeneratorMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_HPP_
#define DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dls2_msgs__msg__TrajectoryGeneratorMsg __attribute__((deprecated))
#else
# define DEPRECATED__dls2_msgs__msg__TrajectoryGeneratorMsg __declspec(deprecated)
#endif

namespace dls2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryGeneratorMsg_
{
  using Type = TrajectoryGeneratorMsg_<ContainerAllocator>;

  explicit TrajectoryGeneratorMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_position.begin(), this->com_position.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->com_orientation.begin(), this->com_orientation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_linear_velocity.begin(), this->com_linear_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_angular_velocity.begin(), this->com_angular_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_linear_acceleration.begin(), this->com_linear_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_angular_acceleration.begin(), this->com_angular_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_position.begin(), this->joints_position.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_velocity.begin(), this->joints_velocity.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_acceleration.begin(), this->joints_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_effort.begin(), this->joints_effort.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->wrench.begin(), this->wrench.end(), 0.0);
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->stance_legs.begin(), this->stance_legs.end(), false);
      std::fill<typename std::array<double, 12>::iterator, double>(this->nominal_touch_down.begin(), this->nominal_touch_down.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->touch_down.begin(), this->touch_down.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->swing_period.begin(), this->swing_period.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->normal_force_max.begin(), this->normal_force_max.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->normal_force_min.begin(), this->normal_force_min.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
    }
  }

  explicit TrajectoryGeneratorMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    com_position(_alloc),
    com_orientation(_alloc),
    com_linear_velocity(_alloc),
    com_angular_velocity(_alloc),
    com_linear_acceleration(_alloc),
    com_angular_acceleration(_alloc),
    joints_position(_alloc),
    joints_velocity(_alloc),
    joints_acceleration(_alloc),
    joints_effort(_alloc),
    wrench(_alloc),
    stance_legs(_alloc),
    nominal_touch_down(_alloc),
    touch_down(_alloc),
    swing_period(_alloc),
    normal_force_max(_alloc),
    normal_force_min(_alloc),
    kp(_alloc),
    kd(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_position.begin(), this->com_position.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->com_orientation.begin(), this->com_orientation.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_linear_velocity.begin(), this->com_linear_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_angular_velocity.begin(), this->com_angular_velocity.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_linear_acceleration.begin(), this->com_linear_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 3>::iterator, double>(this->com_angular_acceleration.begin(), this->com_angular_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_position.begin(), this->joints_position.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_velocity.begin(), this->joints_velocity.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_acceleration.begin(), this->joints_acceleration.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->joints_effort.begin(), this->joints_effort.end(), 0.0);
      std::fill<typename std::array<double, 6>::iterator, double>(this->wrench.begin(), this->wrench.end(), 0.0);
      std::fill<typename std::array<bool, 4>::iterator, bool>(this->stance_legs.begin(), this->stance_legs.end(), false);
      std::fill<typename std::array<double, 12>::iterator, double>(this->nominal_touch_down.begin(), this->nominal_touch_down.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->touch_down.begin(), this->touch_down.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->swing_period.begin(), this->swing_period.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->normal_force_max.begin(), this->normal_force_max.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->normal_force_min.begin(), this->normal_force_min.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 12>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
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
  using _com_position_type =
    std::array<double, 3>;
  _com_position_type com_position;
  using _com_orientation_type =
    std::array<double, 4>;
  _com_orientation_type com_orientation;
  using _com_linear_velocity_type =
    std::array<double, 3>;
  _com_linear_velocity_type com_linear_velocity;
  using _com_angular_velocity_type =
    std::array<double, 3>;
  _com_angular_velocity_type com_angular_velocity;
  using _com_linear_acceleration_type =
    std::array<double, 3>;
  _com_linear_acceleration_type com_linear_acceleration;
  using _com_angular_acceleration_type =
    std::array<double, 3>;
  _com_angular_acceleration_type com_angular_acceleration;
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
  using _wrench_type =
    std::array<double, 6>;
  _wrench_type wrench;
  using _stance_legs_type =
    std::array<bool, 4>;
  _stance_legs_type stance_legs;
  using _nominal_touch_down_type =
    std::array<double, 12>;
  _nominal_touch_down_type nominal_touch_down;
  using _touch_down_type =
    std::array<double, 12>;
  _touch_down_type touch_down;
  using _swing_period_type =
    std::array<double, 4>;
  _swing_period_type swing_period;
  using _normal_force_max_type =
    std::array<double, 4>;
  _normal_force_max_type normal_force_max;
  using _normal_force_min_type =
    std::array<double, 4>;
  _normal_force_min_type normal_force_min;
  using _kp_type =
    std::array<double, 12>;
  _kp_type kp;
  using _kd_type =
    std::array<double, 12>;
  _kd_type kd;

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
  Type & set__com_position(
    const std::array<double, 3> & _arg)
  {
    this->com_position = _arg;
    return *this;
  }
  Type & set__com_orientation(
    const std::array<double, 4> & _arg)
  {
    this->com_orientation = _arg;
    return *this;
  }
  Type & set__com_linear_velocity(
    const std::array<double, 3> & _arg)
  {
    this->com_linear_velocity = _arg;
    return *this;
  }
  Type & set__com_angular_velocity(
    const std::array<double, 3> & _arg)
  {
    this->com_angular_velocity = _arg;
    return *this;
  }
  Type & set__com_linear_acceleration(
    const std::array<double, 3> & _arg)
  {
    this->com_linear_acceleration = _arg;
    return *this;
  }
  Type & set__com_angular_acceleration(
    const std::array<double, 3> & _arg)
  {
    this->com_angular_acceleration = _arg;
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
  Type & set__wrench(
    const std::array<double, 6> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }
  Type & set__stance_legs(
    const std::array<bool, 4> & _arg)
  {
    this->stance_legs = _arg;
    return *this;
  }
  Type & set__nominal_touch_down(
    const std::array<double, 12> & _arg)
  {
    this->nominal_touch_down = _arg;
    return *this;
  }
  Type & set__touch_down(
    const std::array<double, 12> & _arg)
  {
    this->touch_down = _arg;
    return *this;
  }
  Type & set__swing_period(
    const std::array<double, 4> & _arg)
  {
    this->swing_period = _arg;
    return *this;
  }
  Type & set__normal_force_max(
    const std::array<double, 4> & _arg)
  {
    this->normal_force_max = _arg;
    return *this;
  }
  Type & set__normal_force_min(
    const std::array<double, 4> & _arg)
  {
    this->normal_force_min = _arg;
    return *this;
  }
  Type & set__kp(
    const std::array<double, 12> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::array<double, 12> & _arg)
  {
    this->kd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dls2_msgs__msg__TrajectoryGeneratorMsg
    std::shared_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dls2_msgs__msg__TrajectoryGeneratorMsg
    std::shared_ptr<dls2_msgs::msg::TrajectoryGeneratorMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryGeneratorMsg_ & other) const
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
    if (this->com_position != other.com_position) {
      return false;
    }
    if (this->com_orientation != other.com_orientation) {
      return false;
    }
    if (this->com_linear_velocity != other.com_linear_velocity) {
      return false;
    }
    if (this->com_angular_velocity != other.com_angular_velocity) {
      return false;
    }
    if (this->com_linear_acceleration != other.com_linear_acceleration) {
      return false;
    }
    if (this->com_angular_acceleration != other.com_angular_acceleration) {
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
    if (this->wrench != other.wrench) {
      return false;
    }
    if (this->stance_legs != other.stance_legs) {
      return false;
    }
    if (this->nominal_touch_down != other.nominal_touch_down) {
      return false;
    }
    if (this->touch_down != other.touch_down) {
      return false;
    }
    if (this->swing_period != other.swing_period) {
      return false;
    }
    if (this->normal_force_max != other.normal_force_max) {
      return false;
    }
    if (this->normal_force_min != other.normal_force_min) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryGeneratorMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryGeneratorMsg_

// alias to use template instance with default allocator
using TrajectoryGeneratorMsg =
  dls2_msgs::msg::TrajectoryGeneratorMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__TRAJECTORY_GENERATOR_MSG__STRUCT_HPP_

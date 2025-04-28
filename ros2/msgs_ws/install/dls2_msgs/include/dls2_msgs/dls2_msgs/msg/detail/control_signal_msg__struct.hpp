// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dls2_msgs:msg/ControlSignalMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_HPP_
#define DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dls2_msgs__msg__ControlSignalMsg __attribute__((deprecated))
#else
# define DEPRECATED__dls2_msgs__msg__ControlSignalMsg __declspec(deprecated)
#endif

namespace dls2_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControlSignalMsg_
{
  using Type = ControlSignalMsg_<ContainerAllocator>;

  explicit ControlSignalMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      std::fill<typename std::array<double, 12>::iterator, double>(this->torques.begin(), this->torques.end(), 0.0);
      this->signal_reconstruction_method = 0ull;
    }
  }

  explicit ControlSignalMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame_id(_alloc),
    torques(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_id = "";
      this->sequence_id = 0ul;
      this->timestamp = 0.0;
      std::fill<typename std::array<double, 12>::iterator, double>(this->torques.begin(), this->torques.end(), 0.0);
      this->signal_reconstruction_method = 0ull;
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
  using _torques_type =
    std::array<double, 12>;
  _torques_type torques;
  using _signal_reconstruction_method_type =
    uint64_t;
  _signal_reconstruction_method_type signal_reconstruction_method;

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
  Type & set__torques(
    const std::array<double, 12> & _arg)
  {
    this->torques = _arg;
    return *this;
  }
  Type & set__signal_reconstruction_method(
    const uint64_t & _arg)
  {
    this->signal_reconstruction_method = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dls2_msgs__msg__ControlSignalMsg
    std::shared_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dls2_msgs__msg__ControlSignalMsg
    std::shared_ptr<dls2_msgs::msg::ControlSignalMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlSignalMsg_ & other) const
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
    if (this->torques != other.torques) {
      return false;
    }
    if (this->signal_reconstruction_method != other.signal_reconstruction_method) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlSignalMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlSignalMsg_

// alias to use template instance with default allocator
using ControlSignalMsg =
  dls2_msgs::msg::ControlSignalMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dls2_msgs

#endif  // DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__STRUCT_HPP_

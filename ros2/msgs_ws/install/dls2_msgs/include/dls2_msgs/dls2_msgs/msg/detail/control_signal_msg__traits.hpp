// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dls2_msgs:msg/ControlSignalMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__TRAITS_HPP_
#define DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dls2_msgs/msg/detail/control_signal_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dls2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlSignalMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame_id
  {
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << ", ";
  }

  // member: sequence_id
  {
    out << "sequence_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_id, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: torques
  {
    if (msg.torques.size() == 0) {
      out << "torques: []";
    } else {
      out << "torques: [";
      size_t pending_items = msg.torques.size();
      for (auto item : msg.torques) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: signal_reconstruction_method
  {
    out << "signal_reconstruction_method: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_reconstruction_method, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlSignalMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_id, out);
    out << "\n";
  }

  // member: sequence_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence_id: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence_id, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: torques
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.torques.size() == 0) {
      out << "torques: []\n";
    } else {
      out << "torques:\n";
      for (auto item : msg.torques) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: signal_reconstruction_method
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "signal_reconstruction_method: ";
    rosidl_generator_traits::value_to_yaml(msg.signal_reconstruction_method, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlSignalMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dls2_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dls2_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dls2_msgs::msg::ControlSignalMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  dls2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dls2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dls2_msgs::msg::ControlSignalMsg & msg)
{
  return dls2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dls2_msgs::msg::ControlSignalMsg>()
{
  return "dls2_msgs::msg::ControlSignalMsg";
}

template<>
inline const char * name<dls2_msgs::msg::ControlSignalMsg>()
{
  return "dls2_msgs/msg/ControlSignalMsg";
}

template<>
struct has_fixed_size<dls2_msgs::msg::ControlSignalMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dls2_msgs::msg::ControlSignalMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dls2_msgs::msg::ControlSignalMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DLS2_MSGS__MSG__DETAIL__CONTROL_SIGNAL_MSG__TRAITS_HPP_

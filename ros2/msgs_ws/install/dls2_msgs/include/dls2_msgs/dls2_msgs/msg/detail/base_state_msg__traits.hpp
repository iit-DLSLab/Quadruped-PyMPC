// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dls2_msgs:msg/BaseStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__TRAITS_HPP_
#define DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dls2_msgs/msg/detail/base_state_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dls2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BaseStateMsg & msg,
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

  // member: robot_name
  {
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: orientation
  {
    if (msg.orientation.size() == 0) {
      out << "orientation: []";
    } else {
      out << "orientation: [";
      size_t pending_items = msg.orientation.size();
      for (auto item : msg.orientation) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: linear_velocity
  {
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []";
    } else {
      out << "linear_velocity: [";
      size_t pending_items = msg.linear_velocity.size();
      for (auto item : msg.linear_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: angular_velocity
  {
    if (msg.angular_velocity.size() == 0) {
      out << "angular_velocity: []";
    } else {
      out << "angular_velocity: [";
      size_t pending_items = msg.angular_velocity.size();
      for (auto item : msg.angular_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: linear_acceleration
  {
    if (msg.linear_acceleration.size() == 0) {
      out << "linear_acceleration: []";
    } else {
      out << "linear_acceleration: [";
      size_t pending_items = msg.linear_acceleration.size();
      for (auto item : msg.linear_acceleration) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: angular_acceleration
  {
    if (msg.angular_acceleration.size() == 0) {
      out << "angular_acceleration: []";
    } else {
      out << "angular_acceleration: [";
      size_t pending_items = msg.angular_acceleration.size();
      for (auto item : msg.angular_acceleration) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: stance_status
  {
    if (msg.stance_status.size() == 0) {
      out << "stance_status: []";
    } else {
      out << "stance_status: [";
      size_t pending_items = msg.stance_status.size();
      for (auto item : msg.stance_status) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BaseStateMsg & msg,
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

  // member: robot_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_name: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_name, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.orientation.size() == 0) {
      out << "orientation: []\n";
    } else {
      out << "orientation:\n";
      for (auto item : msg.orientation) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.linear_velocity.size() == 0) {
      out << "linear_velocity: []\n";
    } else {
      out << "linear_velocity:\n";
      for (auto item : msg.linear_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angular_velocity.size() == 0) {
      out << "angular_velocity: []\n";
    } else {
      out << "angular_velocity:\n";
      for (auto item : msg.angular_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: linear_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.linear_acceleration.size() == 0) {
      out << "linear_acceleration: []\n";
    } else {
      out << "linear_acceleration:\n";
      for (auto item : msg.linear_acceleration) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: angular_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angular_acceleration.size() == 0) {
      out << "angular_acceleration: []\n";
    } else {
      out << "angular_acceleration:\n";
      for (auto item : msg.angular_acceleration) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: stance_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.stance_status.size() == 0) {
      out << "stance_status: []\n";
    } else {
      out << "stance_status:\n";
      for (auto item : msg.stance_status) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BaseStateMsg & msg, bool use_flow_style = false)
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
  const dls2_msgs::msg::BaseStateMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  dls2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dls2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dls2_msgs::msg::BaseStateMsg & msg)
{
  return dls2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dls2_msgs::msg::BaseStateMsg>()
{
  return "dls2_msgs::msg::BaseStateMsg";
}

template<>
inline const char * name<dls2_msgs::msg::BaseStateMsg>()
{
  return "dls2_msgs/msg/BaseStateMsg";
}

template<>
struct has_fixed_size<dls2_msgs::msg::BaseStateMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dls2_msgs::msg::BaseStateMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dls2_msgs::msg::BaseStateMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DLS2_MSGS__MSG__DETAIL__BASE_STATE_MSG__TRAITS_HPP_

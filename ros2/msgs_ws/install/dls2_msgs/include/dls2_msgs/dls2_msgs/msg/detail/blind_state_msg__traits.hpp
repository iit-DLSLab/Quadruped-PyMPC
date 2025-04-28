// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dls2_msgs:msg/BlindStateMsg.idl
// generated code does not contain a copyright notice

#ifndef DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__TRAITS_HPP_
#define DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dls2_msgs/msg/detail/blind_state_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dls2_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BlindStateMsg & msg,
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

  // member: joints_name
  {
    if (msg.joints_name.size() == 0) {
      out << "joints_name: []";
    } else {
      out << "joints_name: [";
      size_t pending_items = msg.joints_name.size();
      for (auto item : msg.joints_name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joints_position
  {
    if (msg.joints_position.size() == 0) {
      out << "joints_position: []";
    } else {
      out << "joints_position: [";
      size_t pending_items = msg.joints_position.size();
      for (auto item : msg.joints_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joints_velocity
  {
    if (msg.joints_velocity.size() == 0) {
      out << "joints_velocity: []";
    } else {
      out << "joints_velocity: [";
      size_t pending_items = msg.joints_velocity.size();
      for (auto item : msg.joints_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joints_acceleration
  {
    if (msg.joints_acceleration.size() == 0) {
      out << "joints_acceleration: []";
    } else {
      out << "joints_acceleration: [";
      size_t pending_items = msg.joints_acceleration.size();
      for (auto item : msg.joints_acceleration) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joints_effort
  {
    if (msg.joints_effort.size() == 0) {
      out << "joints_effort: []";
    } else {
      out << "joints_effort: [";
      size_t pending_items = msg.joints_effort.size();
      for (auto item : msg.joints_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joints_temperature
  {
    if (msg.joints_temperature.size() == 0) {
      out << "joints_temperature: []";
    } else {
      out << "joints_temperature: [";
      size_t pending_items = msg.joints_temperature.size();
      for (auto item : msg.joints_temperature) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: feet_contact
  {
    if (msg.feet_contact.size() == 0) {
      out << "feet_contact: []";
    } else {
      out << "feet_contact: [";
      size_t pending_items = msg.feet_contact.size();
      for (auto item : msg.feet_contact) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: current_feet_positions
  {
    if (msg.current_feet_positions.size() == 0) {
      out << "current_feet_positions: []";
    } else {
      out << "current_feet_positions: [";
      size_t pending_items = msg.current_feet_positions.size();
      for (auto item : msg.current_feet_positions) {
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
  const BlindStateMsg & msg,
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

  // member: joints_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_name.size() == 0) {
      out << "joints_name: []\n";
    } else {
      out << "joints_name:\n";
      for (auto item : msg.joints_name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joints_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_position.size() == 0) {
      out << "joints_position: []\n";
    } else {
      out << "joints_position:\n";
      for (auto item : msg.joints_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joints_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_velocity.size() == 0) {
      out << "joints_velocity: []\n";
    } else {
      out << "joints_velocity:\n";
      for (auto item : msg.joints_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joints_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_acceleration.size() == 0) {
      out << "joints_acceleration: []\n";
    } else {
      out << "joints_acceleration:\n";
      for (auto item : msg.joints_acceleration) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joints_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_effort.size() == 0) {
      out << "joints_effort: []\n";
    } else {
      out << "joints_effort:\n";
      for (auto item : msg.joints_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joints_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints_temperature.size() == 0) {
      out << "joints_temperature: []\n";
    } else {
      out << "joints_temperature:\n";
      for (auto item : msg.joints_temperature) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: feet_contact
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.feet_contact.size() == 0) {
      out << "feet_contact: []\n";
    } else {
      out << "feet_contact:\n";
      for (auto item : msg.feet_contact) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: current_feet_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.current_feet_positions.size() == 0) {
      out << "current_feet_positions: []\n";
    } else {
      out << "current_feet_positions:\n";
      for (auto item : msg.current_feet_positions) {
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

inline std::string to_yaml(const BlindStateMsg & msg, bool use_flow_style = false)
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
  const dls2_msgs::msg::BlindStateMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  dls2_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dls2_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dls2_msgs::msg::BlindStateMsg & msg)
{
  return dls2_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dls2_msgs::msg::BlindStateMsg>()
{
  return "dls2_msgs::msg::BlindStateMsg";
}

template<>
inline const char * name<dls2_msgs::msg::BlindStateMsg>()
{
  return "dls2_msgs/msg/BlindStateMsg";
}

template<>
struct has_fixed_size<dls2_msgs::msg::BlindStateMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dls2_msgs::msg::BlindStateMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dls2_msgs::msg::BlindStateMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DLS2_MSGS__MSG__DETAIL__BLIND_STATE_MSG__TRAITS_HPP_

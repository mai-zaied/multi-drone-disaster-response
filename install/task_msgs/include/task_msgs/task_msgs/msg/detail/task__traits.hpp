// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from task_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#ifndef TASK_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_
#define TASK_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "task_msgs/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace task_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Task & msg,
  std::ostream & out)
{
  out << "{";
  // member: task_id
  {
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << ", ";
  }

  // member: task_type
  {
    out << "task_type: ";
    rosidl_generator_traits::value_to_yaml(msg.task_type, out);
    out << ", ";
  }

  // member: drone_id
  {
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: priority
  {
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << ", ";
  }

  // member: payload
  {
    out << "payload: ";
    rosidl_generator_traits::value_to_yaml(msg.payload, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: task_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_id: ";
    rosidl_generator_traits::value_to_yaml(msg.task_id, out);
    out << "\n";
  }

  // member: task_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_type: ";
    rosidl_generator_traits::value_to_yaml(msg.task_type, out);
    out << "\n";
  }

  // member: drone_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drone_id: ";
    rosidl_generator_traits::value_to_yaml(msg.drone_id, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: priority
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "priority: ";
    rosidl_generator_traits::value_to_yaml(msg.priority, out);
    out << "\n";
  }

  // member: payload
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "payload: ";
    rosidl_generator_traits::value_to_yaml(msg.payload, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Task & msg, bool use_flow_style = false)
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

}  // namespace task_msgs

namespace rosidl_generator_traits
{

[[deprecated("use task_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const task_msgs::msg::Task & msg,
  std::ostream & out, size_t indentation = 0)
{
  task_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use task_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const task_msgs::msg::Task & msg)
{
  return task_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<task_msgs::msg::Task>()
{
  return "task_msgs::msg::Task";
}

template<>
inline const char * name<task_msgs::msg::Task>()
{
  return "task_msgs/msg/Task";
}

template<>
struct has_fixed_size<task_msgs::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<task_msgs::msg::Task>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<task_msgs::msg::Task>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TASK_MSGS__MSG__DETAIL__TASK__TRAITS_HPP_

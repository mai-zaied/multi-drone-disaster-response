// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from task_msgs:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_msgs/msg/task.hpp"


#ifndef TASK_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_
#define TASK_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "task_msgs/msg/detail/task__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace task_msgs
{

namespace msg
{

namespace builder
{

class Init_Task_payload
{
public:
  explicit Init_Task_payload(::task_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  ::task_msgs::msg::Task payload(::task_msgs::msg::Task::_payload_type arg)
  {
    msg_.payload = std::move(arg);
    return std::move(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

class Init_Task_priority
{
public:
  explicit Init_Task_priority(::task_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_payload priority(::task_msgs::msg::Task::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return Init_Task_payload(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

class Init_Task_timestamp
{
public:
  explicit Init_Task_timestamp(::task_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_priority timestamp(::task_msgs::msg::Task::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Task_priority(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

class Init_Task_drone_id
{
public:
  explicit Init_Task_drone_id(::task_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_timestamp drone_id(::task_msgs::msg::Task::_drone_id_type arg)
  {
    msg_.drone_id = std::move(arg);
    return Init_Task_timestamp(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

class Init_Task_task_type
{
public:
  explicit Init_Task_task_type(::task_msgs::msg::Task & msg)
  : msg_(msg)
  {}
  Init_Task_drone_id task_type(::task_msgs::msg::Task::_task_type_type arg)
  {
    msg_.task_type = std::move(arg);
    return Init_Task_drone_id(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

class Init_Task_task_id
{
public:
  Init_Task_task_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Task_task_type task_id(::task_msgs::msg::Task::_task_id_type arg)
  {
    msg_.task_id = std::move(arg);
    return Init_Task_task_type(msg_);
  }

private:
  ::task_msgs::msg::Task msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::task_msgs::msg::Task>()
{
  return task_msgs::msg::builder::Init_Task_task_id();
}

}  // namespace task_msgs

#endif  // TASK_MSGS__MSG__DETAIL__TASK__BUILDER_HPP_

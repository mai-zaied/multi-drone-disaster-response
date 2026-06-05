// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from task_msgs:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_msgs/msg/task.hpp"


#ifndef TASK_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_
#define TASK_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__task_msgs__msg__Task __attribute__((deprecated))
#else
# define DEPRECATED__task_msgs__msg__Task __declspec(deprecated)
#endif

namespace task_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Task_
{
  using Type = Task_<ContainerAllocator>;

  explicit Task_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
      this->task_type = "";
      this->drone_id = "";
      this->priority = 0;
      this->payload = "";
    }
  }

  explicit Task_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : task_id(_alloc),
    task_type(_alloc),
    drone_id(_alloc),
    timestamp(_alloc, _init),
    payload(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->task_id = "";
      this->task_type = "";
      this->drone_id = "";
      this->priority = 0;
      this->payload = "";
    }
  }

  // field types and members
  using _task_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_id_type task_id;
  using _task_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _task_type_type task_type;
  using _drone_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _drone_id_type drone_id;
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _priority_type =
    uint8_t;
  _priority_type priority;
  using _payload_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _payload_type payload;

  // setters for named parameter idiom
  Type & set__task_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_id = _arg;
    return *this;
  }
  Type & set__task_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->task_type = _arg;
    return *this;
  }
  Type & set__drone_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->drone_id = _arg;
    return *this;
  }
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__priority(
    const uint8_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }
  Type & set__payload(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->payload = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    task_msgs::msg::Task_<ContainerAllocator> *;
  using ConstRawPtr =
    const task_msgs::msg::Task_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<task_msgs::msg::Task_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<task_msgs::msg::Task_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      task_msgs::msg::Task_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<task_msgs::msg::Task_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      task_msgs::msg::Task_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<task_msgs::msg::Task_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<task_msgs::msg::Task_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<task_msgs::msg::Task_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__task_msgs__msg__Task
    std::shared_ptr<task_msgs::msg::Task_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__task_msgs__msg__Task
    std::shared_ptr<task_msgs::msg::Task_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Task_ & other) const
  {
    if (this->task_id != other.task_id) {
      return false;
    }
    if (this->task_type != other.task_type) {
      return false;
    }
    if (this->drone_id != other.drone_id) {
      return false;
    }
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    if (this->payload != other.payload) {
      return false;
    }
    return true;
  }
  bool operator!=(const Task_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Task_

// alias to use template instance with default allocator
using Task =
  task_msgs::msg::Task_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace task_msgs

#endif  // TASK_MSGS__MSG__DETAIL__TASK__STRUCT_HPP_

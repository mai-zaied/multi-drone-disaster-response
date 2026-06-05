// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from task_msgs:msg/Task.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "task_msgs/msg/task.h"


#ifndef TASK_MSGS__MSG__DETAIL__TASK__STRUCT_H_
#define TASK_MSGS__MSG__DETAIL__TASK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'task_id'
// Member 'task_type'
// Member 'drone_id'
// Member 'payload'
#include "rosidl_runtime_c/string.h"
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Task in the package task_msgs.
/**
  * Task.msg — a unit of work in the fog-assisted UAV system
  * Used for offloading decisions across drone, fog, and cloud tiers.
 */
typedef struct task_msgs__msg__Task
{
  /// Unique identifier for this task instance, e.g. "drone0-status-0042"
  rosidl_runtime_c__String task_id;
  /// Task category, drives offloading decision and routing.
  /// Allowed values: STATUS_REPORT, BATTERY_CHECK, VICTIM_DETECTION, LOG_UPLOAD,
  ///                 METRICS_REPORT, DETECTION_ARCHIVE
  rosidl_runtime_c__String task_type;
  /// Originating drone identifier ("drone0", "drone1", "drone2")
  rosidl_runtime_c__String drone_id;
  /// Time of task creation at the producer
  builtin_interfaces__msg__Time timestamp;
  /// Priority level: 0 = low, 1 = normal, 2 = high, 3 = critical
  uint8_t priority;
  /// JSON-encoded task payload. Schema depends on task_type.
  /// Example for STATUS_REPORT:
  ///   {"battery": 87.4, "nav_state": 4, "arming_state": 1, "position": [1.2, -0.5, -10.3]}
  rosidl_runtime_c__String payload;
} task_msgs__msg__Task;

// Struct for a sequence of task_msgs__msg__Task.
typedef struct task_msgs__msg__Task__Sequence
{
  task_msgs__msg__Task * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_msgs__msg__Task__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TASK_MSGS__MSG__DETAIL__TASK__STRUCT_H_

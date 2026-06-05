// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from task_msgs:msg/Task.idl
// generated code does not contain a copyright notice

#include "task_msgs/msg/detail/task__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_task_msgs
const rosidl_type_hash_t *
task_msgs__msg__Task__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x64, 0xa8, 0x23, 0x17, 0xe6, 0xd4, 0x33, 0xc7,
      0x58, 0x24, 0xf8, 0x23, 0x5b, 0x6f, 0xaf, 0x4e,
      0x2d, 0x3f, 0x18, 0x70, 0xe5, 0x10, 0x33, 0xa3,
      0xbb, 0x28, 0xfb, 0x19, 0xad, 0x6b, 0x18, 0xbc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char task_msgs__msg__Task__TYPE_NAME[] = "task_msgs/msg/Task";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char task_msgs__msg__Task__FIELD_NAME__task_id[] = "task_id";
static char task_msgs__msg__Task__FIELD_NAME__task_type[] = "task_type";
static char task_msgs__msg__Task__FIELD_NAME__drone_id[] = "drone_id";
static char task_msgs__msg__Task__FIELD_NAME__timestamp[] = "timestamp";
static char task_msgs__msg__Task__FIELD_NAME__priority[] = "priority";
static char task_msgs__msg__Task__FIELD_NAME__payload[] = "payload";

static rosidl_runtime_c__type_description__Field task_msgs__msg__Task__FIELDS[] = {
  {
    {task_msgs__msg__Task__FIELD_NAME__task_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_msgs__msg__Task__FIELD_NAME__task_type, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_msgs__msg__Task__FIELD_NAME__drone_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_msgs__msg__Task__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {task_msgs__msg__Task__FIELD_NAME__priority, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {task_msgs__msg__Task__FIELD_NAME__payload, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription task_msgs__msg__Task__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
task_msgs__msg__Task__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {task_msgs__msg__Task__TYPE_NAME, 18, 18},
      {task_msgs__msg__Task__FIELDS, 6, 6},
    },
    {task_msgs__msg__Task__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Task.msg \\xe2\\x80\\x94 a unit of work in the fog-assisted UAV system\n"
  "# Used for offloading decisions across drone, fog, and cloud tiers.\n"
  "\n"
  "# Unique identifier for this task instance, e.g. \"drone0-status-0042\"\n"
  "string task_id\n"
  "\n"
  "# Task category, drives offloading decision and routing.\n"
  "# Allowed values: STATUS_REPORT, BATTERY_CHECK, VICTIM_DETECTION, LOG_UPLOAD,\n"
  "#                 METRICS_REPORT, DETECTION_ARCHIVE\n"
  "string task_type\n"
  "\n"
  "# Originating drone identifier (\"drone0\", \"drone1\", \"drone2\")\n"
  "string drone_id\n"
  "\n"
  "# Time of task creation at the producer\n"
  "builtin_interfaces/Time timestamp\n"
  "\n"
  "# Priority level: 0 = low, 1 = normal, 2 = high, 3 = critical\n"
  "uint8 priority\n"
  "\n"
  "# JSON-encoded task payload. Schema depends on task_type.\n"
  "# Example for STATUS_REPORT:\n"
  "#   {\"battery\": 87.4, \"nav_state\": 4, \"arming_state\": 1, \"position\": [1.2, -0.5, -10.3]}\n"
  "string payload";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
task_msgs__msg__Task__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {task_msgs__msg__Task__TYPE_NAME, 18, 18},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 841, 841},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
task_msgs__msg__Task__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *task_msgs__msg__Task__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

# Task.msg — a unit of work in the fog-assisted UAV system
# Used for offloading decisions across drone, fog, and cloud tiers.

# Unique identifier for this task instance, e.g. "drone0-status-0042"
string task_id

# Task category, drives offloading decision and routing.
# Allowed values: STATUS_REPORT, BATTERY_CHECK, VICTIM_DETECTION, LOG_UPLOAD,
#                 METRICS_REPORT, DETECTION_ARCHIVE
string task_type

# Originating drone identifier ("drone0", "drone1", "drone2")
string drone_id

# Time of task creation at the producer
builtin_interfaces/Time timestamp

# Priority level: 0 = low, 1 = normal, 2 = high, 3 = critical
uint8 priority

# JSON-encoded task payload. Schema depends on task_type.
# Example for STATUS_REPORT:
#   {"battery": 87.4, "nav_state": 4, "arming_state": 1, "position": [1.2, -0.5, -10.3]}
string payload
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to task_msgs__msg__Task
/// Task.msg — a unit of work in the fog-assisted UAV system
/// Used for offloading decisions across drone, fog, and cloud tiers.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task {
    /// Unique identifier for this task instance, e.g. "drone0-status-0042"
    pub task_id: std::string::String,

    /// Task category, drives offloading decision and routing.
    /// Allowed values: STATUS_REPORT, BATTERY_CHECK, VICTIM_DETECTION, LOG_UPLOAD,
    ///                 METRICS_REPORT, DETECTION_ARCHIVE
    pub task_type: std::string::String,

    /// Originating drone identifier ("drone0", "drone1", "drone2")
    pub drone_id: std::string::String,

    /// Time of task creation at the producer
    pub timestamp: builtin_interfaces::msg::Time,

    /// Priority level: 0 = low, 1 = normal, 2 = high, 3 = critical
    pub priority: u8,

    /// JSON-encoded task payload. Schema depends on task_type.
    /// Example for STATUS_REPORT:
    ///   {"battery": 87.4, "nav_state": 4, "arming_state": 1, "position": [1.2, -0.5, -10.3]}
    pub payload: std::string::String,

}



impl Default for Task {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Task::default())
  }
}

impl rosidl_runtime_rs::Message for Task {
  type RmwMsg = super::msg::rmw::Task;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        task_type: msg.task_type.as_str().into(),
        drone_id: msg.drone_id.as_str().into(),
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.timestamp)).into_owned(),
        priority: msg.priority,
        payload: msg.payload.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        task_id: msg.task_id.as_str().into(),
        task_type: msg.task_type.as_str().into(),
        drone_id: msg.drone_id.as_str().into(),
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.timestamp)).into_owned(),
      priority: msg.priority,
        payload: msg.payload.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      task_id: msg.task_id.to_string(),
      task_type: msg.task_type.to_string(),
      drone_id: msg.drone_id.to_string(),
      timestamp: builtin_interfaces::msg::Time::from_rmw_message(msg.timestamp),
      priority: msg.priority,
      payload: msg.payload.to_string(),
    }
  }
}



#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "task_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__task_msgs__msg__Task() -> *const std::ffi::c_void;
}

#[link(name = "task_msgs__rosidl_generator_c")]
extern "C" {
    fn task_msgs__msg__Task__init(msg: *mut Task) -> bool;
    fn task_msgs__msg__Task__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Task>, size: usize) -> bool;
    fn task_msgs__msg__Task__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Task>);
    fn task_msgs__msg__Task__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Task>, out_seq: *mut rosidl_runtime_rs::Sequence<Task>) -> bool;
}

// Corresponds to task_msgs__msg__Task
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Task.msg — a unit of work in the fog-assisted UAV system
/// Used for offloading decisions across drone, fog, and cloud tiers.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Task {
    /// Unique identifier for this task instance, e.g. "drone0-status-0042"
    pub task_id: rosidl_runtime_rs::String,

    /// Task category, drives offloading decision and routing.
    /// Allowed values: STATUS_REPORT, BATTERY_CHECK, VICTIM_DETECTION, LOG_UPLOAD,
    ///                 METRICS_REPORT, DETECTION_ARCHIVE
    pub task_type: rosidl_runtime_rs::String,

    /// Originating drone identifier ("drone0", "drone1", "drone2")
    pub drone_id: rosidl_runtime_rs::String,

    /// Time of task creation at the producer
    pub timestamp: builtin_interfaces::msg::rmw::Time,

    /// Priority level: 0 = low, 1 = normal, 2 = high, 3 = critical
    pub priority: u8,

    /// JSON-encoded task payload. Schema depends on task_type.
    /// Example for STATUS_REPORT:
    ///   {"battery": 87.4, "nav_state": 4, "arming_state": 1, "position": [1.2, -0.5, -10.3]}
    pub payload: rosidl_runtime_rs::String,

}



impl Default for Task {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !task_msgs__msg__Task__init(&mut msg as *mut _) {
        panic!("Call to task_msgs__msg__Task__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Task {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { task_msgs__msg__Task__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { task_msgs__msg__Task__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { task_msgs__msg__Task__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Task {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Task where Self: Sized {
  const TYPE_NAME: &'static str = "task_msgs/msg/Task";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__task_msgs__msg__Task() }
  }
}



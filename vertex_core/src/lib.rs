//! `vertex_core` — the ROS-agnostic core of the Vertex/ROS 2 integration.
//!
//! It owns everything correctness-critical and testable without a ROS install:
//! the managed-node [`lifecycle`] state machine, the Tokio↔channel [`bridge`],
//! the [`engine_task`] that drives `tashi_vertex::Engine`, the [`convert`]sion
//! from Vertex events into plain records that mirror the `vertex_ros2_msgs`
//! shapes, [`config`] parsing, and the lifecycle [`controller`].
//!
//! The sibling `vertex_ros2` crate is the thin ROS layer: it maps these types
//! onto `rclrs` publishers/subscribers/services and the generated message
//! types, and ships the `vertex_node` binary. That crate only builds inside a
//! colcon/ROS 2 workspace; this one builds and is exercised by plain
//! `cargo test`. See the workspace README for the design overview.

pub mod bridge;
pub mod config;
pub mod controller;
pub mod convert;
pub mod engine_task;
pub mod lifecycle;
pub mod status;

pub use config::{BridgeConfig, Config, ConfigError, PeerConfig, VertexOptions};
pub use controller::{Controller, ControllerError};
pub use convert::{EventRecord, StampedTime, SyncPointRecord, TransactionRecord};
pub use lifecycle::{LifecycleError, LifecycleState, Transition};
pub use status::{Status, StatusSnapshot};

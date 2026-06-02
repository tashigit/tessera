//! The bounded channels that bridge the `rclrs` executor thread and the Tokio
//! runtime in both directions (design §4.3).
//!
//! * inbound: ROS `/vertex/tx` subscriber → [`TxSender`] → engine task
//! * outbound: engine task → [`EventReceiver`] / [`SyncPointReceiver`] → ROS
//!   publishers
//!
//! All channels are bounded; backpressure on the inbound side surfaces as
//! `tx_rejected_total`, and a saturated outbound event channel briefly blocks
//! the engine task (and emits a diagnostic warning) rather than growing without
//! bound.

use tokio::sync::mpsc;

use crate::convert::{EventRecord, SyncPointRecord};

/// Fixed sync-point channel capacity. Sync points are infrequent and cheap.
pub const SYNC_POINT_CHANNEL_CAPACITY: usize = 64;

pub type TxSender = mpsc::Sender<Vec<u8>>;
pub type TxReceiver = mpsc::Receiver<Vec<u8>>;
pub type EventSender = mpsc::Sender<EventRecord>;
pub type EventReceiver = mpsc::Receiver<EventRecord>;
pub type SyncPointSender = mpsc::Sender<SyncPointRecord>;
pub type SyncPointReceiver = mpsc::Receiver<SyncPointRecord>;

/// The sender half kept by the node side (ROS → Vertex).
pub fn tx_channel(capacity: usize) -> (TxSender, TxReceiver) {
    mpsc::channel(capacity.max(1))
}

/// The Vertex → ROS event channel.
pub fn event_channel(capacity: usize) -> (EventSender, EventReceiver) {
    mpsc::channel(capacity.max(1))
}

/// The Vertex → ROS sync-point channel.
pub fn sync_point_channel() -> (SyncPointSender, SyncPointReceiver) {
    mpsc::channel(SYNC_POINT_CHANNEL_CAPACITY)
}

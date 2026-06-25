//! Conversion from `tashi-vertex` types into plain, ROS-agnostic records.
//!
//! These records mirror the `vertex_ros2_msgs` message shapes (design §4.5) but
//! carry no dependency on the generated ROS types, so they are fully testable
//! here. The `ros` module maps them 1:1 onto the real messages.

use tashi_vertex::Event;

/// A timestamp split into seconds + nanoseconds, matching
/// `builtin_interfaces/msg/Time` (`int32 sec`, `uint32 nanosec`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StampedTime {
    pub sec: i32,
    pub nanosec: u32,
}

/// Convert a Vertex `u64` timestamp into a [`StampedTime`].
///
/// IMPORTANT (design §4.5 / §9.5): `tashi-vertex` documents `created_at` /
/// `consensus_at` as "Unix timestamps" but does **not** document the unit. This
/// integration pins the unit to **nanoseconds since the Unix epoch**. If the
/// upstream unit turns out to be micro- or milliseconds, change only this
/// function (and the matching test). The whole codebase routes through here so
/// there is a single place to fix.
pub const NANOS_PER_SEC: u64 = 1_000_000_000;

#[inline]
pub fn nanos_to_time(ts_nanos: u64) -> StampedTime {
    StampedTime {
        sec: (ts_nanos / NANOS_PER_SEC) as i32,
        nanosec: (ts_nanos % NANOS_PER_SEC) as u32,
    }
}

/// A single transaction as carried inside an outbound [`EventRecord`].
///
/// Vertex stores transaction payloads as opaque bytes and does not transmit the
/// `created_at`/`tag` metadata that the inbound `VertexTransaction` message
/// carries. On the outbound side we therefore only have `payload`; `created_at`
/// and `tag` are left at their defaults. See README "Semantics".
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransactionRecord {
    pub payload: Vec<u8>,
}

/// An outbound, totally-ordered consensus event. Mirrors `VertexEvent`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EventRecord {
    pub consensus_at: StampedTime,
    pub created_at: StampedTime,
    pub hash: [u8; 32],
    pub creator_pub_der: Vec<u8>,
    pub whitened_signature: Vec<u8>,
    pub transactions: Vec<TransactionRecord>,
}

impl EventRecord {
    /// Build an [`EventRecord`] from a Vertex [`Event`], eagerly copying every
    /// borrowed slice out of FFI-owned memory (design §4.8). After this returns
    /// the caller may drop the `Event` safely.
    pub fn from_event(event: &Event) -> Self {
        // Iterate by index 0..n — `transactions()` is an ExactSizeIterator, so
        // per-event order is preserved automatically (design §4.7).
        let transactions = event
            .transactions()
            .map(|tx| TransactionRecord {
                payload: tx.to_vec(),
            })
            .collect();

        EventRecord {
            consensus_at: nanos_to_time(event.consensus_at()),
            created_at: nanos_to_time(event.created_at()),
            hash: *event.hash(),
            creator_pub_der: event.creator().to_der_vec().unwrap_or_default(),
            // Re-enabled against tashi-vertex v0.14.0 (TAS-92): the FFI getter
            // `tv_event_get_whitened_signature` reads a `Box<[u8; Signature::
            // LENGTH]>` field (engine `src/engine/event.rs:75`) via `as_ptr()`/
            // `len()` — always non-null, fixed length, so it cannot segfault.
            // The earlier null-deref was a pre-0.14.0 observation.
            whitened_signature: event.whitened_signature().to_vec(),
            transactions,
        }
    }
}

/// An outbound sync point. Mirrors `VertexSyncPoint`.
///
/// `payload` is empty until upstream exposes sync-point accessors (design §9.2).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SyncPointRecord {
    pub observed_at: StampedTime,
    pub payload: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nanos_split_exactly_on_second_boundary() {
        assert_eq!(nanos_to_time(0), StampedTime { sec: 0, nanosec: 0 });
        assert_eq!(
            nanos_to_time(NANOS_PER_SEC),
            StampedTime { sec: 1, nanosec: 0 }
        );
    }

    #[test]
    fn nanos_split_keeps_sub_second_remainder() {
        // 3.5 seconds.
        let t = nanos_to_time(3 * NANOS_PER_SEC + 500_000_000);
        assert_eq!(t, StampedTime { sec: 3, nanosec: 500_000_000 });
    }

    #[test]
    fn nanos_split_max_remainder() {
        let t = nanos_to_time(2 * NANOS_PER_SEC + (NANOS_PER_SEC - 1));
        assert_eq!(t.sec, 2);
        assert_eq!(t.nanosec, (NANOS_PER_SEC - 1) as u32);
    }
}

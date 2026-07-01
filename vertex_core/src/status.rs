//! Live status snapshot, lock-free for readers.
//!
//! The `/vertex/status` service handler and the `/diagnostics` publisher both
//! read this. It is kept in an [`ArcSwap`] so reads never block the producing
//! task and the service handler returns instantly (design §4.3).

use std::sync::Arc;

use arc_swap::ArcSwap;

use tashi_vertex::Error as VertexError;

/// Map a `tashi_vertex::Error` to the stable `i32` code surfaced in
/// `VertexStatus.last_error_code`. Values mirror the upstream C `TVResult`
/// discriminants; `0` means OK.
pub fn error_code(err: VertexError) -> i32 {
    match err {
        VertexError::Argument => -1,
        VertexError::ArgumentNull => -2,
        VertexError::KeyFromDer => -3,
        VertexError::Context => -4,
        VertexError::BufferTooSmall => -5,
        VertexError::Base58Decode => -6,
        VertexError::SocketBind => -7,
        VertexError::EngineStart => -8,
        VertexError::MessageReceive => -9,
        VertexError::TransactionSendClosed => -10,
        VertexError::TransactionDataTooLarge => -11,
    }
}

/// An immutable snapshot of node status. Cloned out on every read.
/// `Default` is "not running, all counters zero, no error".
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct StatusSnapshot {
    pub running: bool,
    pub bind_address: String,
    pub peer_count: u32,
    pub tx_submitted_total: u64,
    pub tx_rejected_total: u64,
    pub events_published_total: u64,
    pub last_error_code: i32,
    pub last_error_message: String,
}

/// Shared, atomically-swappable status. Cheap to clone (`Arc`).
#[derive(Clone, Default)]
pub struct Status {
    inner: Arc<ArcSwap<StatusSnapshot>>,
}

impl Status {
    pub fn new() -> Self {
        Status {
            inner: Arc::new(ArcSwap::from_pointee(StatusSnapshot::default())),
        }
    }

    /// Read the current snapshot.
    pub fn load(&self) -> StatusSnapshot {
        StatusSnapshot::clone(&self.inner.load())
    }

    /// Apply a mutation to a copy of the current snapshot and publish it.
    ///
    /// `ArcSwap::rcu` retries the closure if another thread swaps concurrently,
    /// so updates compose without locks.
    pub fn update(&self, f: impl Fn(&mut StatusSnapshot)) {
        self.inner.rcu(|cur| {
            let mut next = StatusSnapshot::clone(cur);
            f(&mut next);
            next
        });
    }

    pub fn set_running(&self, running: bool, bind_address: &str, peer_count: u32) {
        self.update(|s| {
            s.running = running;
            s.bind_address = bind_address.to_string();
            s.peer_count = peer_count;
        });
    }

    pub fn set_stopped(&self) {
        self.update(|s| {
            s.running = false;
            s.peer_count = 0;
        });
    }

    pub fn record_submitted(&self) {
        self.update(|s| s.tx_submitted_total += 1);
    }

    pub fn record_rejected(&self, err: Option<VertexError>) {
        self.update(|s| {
            s.tx_rejected_total += 1;
            if let Some(e) = err {
                s.last_error_code = error_code(e);
                s.last_error_message = e.to_string();
            }
        });
    }

    pub fn record_event_published(&self) {
        self.update(|s| s.events_published_total += 1);
    }

    pub fn record_error(&self, err: VertexError) {
        self.update(|s| {
            s.last_error_code = error_code(err);
            s.last_error_message = err.to_string();
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_are_zeroed_and_not_running() {
        let s = Status::new();
        let snap = s.load();
        assert!(!snap.running);
        assert_eq!(snap.tx_submitted_total, 0);
        assert_eq!(snap.last_error_code, 0);
    }

    #[test]
    fn counters_accumulate() {
        let s = Status::new();
        s.record_submitted();
        s.record_submitted();
        s.record_event_published();
        s.record_rejected(Some(VertexError::Argument));

        let snap = s.load();
        assert_eq!(snap.tx_submitted_total, 2);
        assert_eq!(snap.events_published_total, 1);
        assert_eq!(snap.tx_rejected_total, 1);
        assert_eq!(snap.last_error_code, -1);
        assert!(!snap.last_error_message.is_empty());
    }

    #[test]
    fn running_state_is_published() {
        let s = Status::new();
        s.set_running(true, "127.0.0.1:9000", 3);
        let snap = s.load();
        assert!(snap.running);
        assert_eq!(snap.bind_address, "127.0.0.1:9000");
        assert_eq!(snap.peer_count, 3);

        s.set_stopped();
        assert!(!s.load().running);
    }
}

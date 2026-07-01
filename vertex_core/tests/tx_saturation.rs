//! Integration test for the design §7 acceptance criterion:
//! *"N submitted transactions are either accepted or counted as rejected"* —
//! "submits 10× the configured capacity in a tight loop, asserts
//! `submitted + rejected == N`".
//!
//! This proves the backpressure accounting is leak-free: every byte slice handed
//! to `Controller::submit` is counted exactly once. A payload is either
//!   * rejected at the bounded inbound channel (`try_send` fails → counted
//!     immediately on the caller side, design §4.6), or
//!   * carried across the channel and counted by the engine thread once
//!     `send_transaction` returns (success → submitted, engine error → rejected).
//! So once the engine thread has drained the channel, the two counters must sum
//! to exactly the number of submissions, no matter how the race between the
//! tight submit loop and the drain plays out.

use std::time::Duration;

use vertex_core::config::{BridgeConfig, Config, VertexOptions};
use vertex_core::Controller;

/// Deliberately tiny inbound channel so a tight submit loop overflows it.
const TX_CAPACITY: usize = 4;
/// "10× the configured capacity" (design §7).
const N: usize = TX_CAPACITY * 10;

fn make_config(port: u16) -> Config {
    let key = tashi_vertex::KeySecret::generate();
    let options = VertexOptions {
        heartbeat_us: 50_000,
        report_gossip_events: false,
        ..VertexOptions::default()
    };
    Config {
        bind_address: format!("127.0.0.1:{port}"),
        key,
        peers: Vec::new(), // solo session
        joining_running_session: false,
        bridge: BridgeConfig {
            tx_channel_capacity: TX_CAPACITY,
            ..BridgeConfig::default()
        },
        options,
    }
}

#[test]
fn over_capacity_submissions_are_all_accounted_for() {
    let mut controller = Controller::new();
    controller.configure(make_config(47391)).expect("configure");
    controller.activate().expect("activate");
    assert!(controller.status().load().running);

    // Flood the bounded inbound channel far faster than the engine thread can
    // drain it: N synchronous `try_send`s with no yield, against a depth-4
    // channel whose drain does real FFI work per item.
    for i in 0..N {
        controller.submit(format!("flood-{i:04}").into_bytes());
    }

    // The caller-side rejections are counted synchronously, but the items that
    // *did* make it into the channel are counted on the engine thread as it
    // drains. Wait for the accounting to settle, then assert the §7 invariant.
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    let snap = loop {
        let snap = controller.status().load();
        let total = snap.tx_submitted_total + snap.tx_rejected_total;
        if total as usize == N || std::time::Instant::now() >= deadline {
            break snap;
        }
        std::thread::sleep(Duration::from_millis(20));
    };

    // THE criterion: nothing is lost or double-counted.
    assert_eq!(
        snap.tx_submitted_total + snap.tx_rejected_total,
        N as u64,
        "every submission must be counted exactly once \
         (submitted={}, rejected={}, N={N})",
        snap.tx_submitted_total,
        snap.tx_rejected_total,
    );

    // And saturation was genuinely exercised: with a depth-4 channel and a tight
    // loop of 40 submissions, the channel must have rejected at least one.
    assert!(
        snap.tx_rejected_total > 0,
        "expected the depth-{TX_CAPACITY} channel to reject under a {N}-deep \
         flood, but tx_rejected_total was 0"
    );

    controller.deactivate().expect("deactivate");
    controller.cleanup().expect("cleanup");
    controller.shutdown().expect("shutdown");
}

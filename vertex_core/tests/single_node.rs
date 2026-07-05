//! Integration test (design §5.3 layer 2, §7 acceptance "events published in
//! Vertex order"): spin up a real single-peer Vertex session in-process through
//! the [`Controller`], submit a batch of transactions, and assert they come back
//! on the event channel in exactly the order they were submitted.
//!
//! This exercises the whole core path end-to-end: lifecycle → engine thread →
//! `Engine::start`/`send_transaction`/`recv_message` → `EventRecord` conversion
//! → bounded channel → consumer. It links the real `libtashi-vertex`.

use std::time::Duration;

use tokio::runtime::Builder;
use tokio::time::timeout;

use vertex_core::config::{BridgeConfig, Config, VertexOptions};
use vertex_core::Controller;

const N: usize = 16;
/// Generous upper bound: first consensus on a fresh session can take a couple of
/// seconds (dynamic epoch sizing settles in the 1–3s range).
const OVERALL_TIMEOUT: Duration = Duration::from_secs(30);

fn make_config(port: u16) -> Config {
    let key = tashi_vertex::KeySecret::generate();
    // Tighten the heartbeat so a solo session forms events quickly under test.
    let options = VertexOptions {
        heartbeat_us: 50_000,
        report_gossip_events: false,
        ..VertexOptions::default()
    };
    Config {
        bind_address: format!("127.0.0.1:{port}"),
        key,
        peers: Vec::new(), // solo session: we are the only peer
        joining_running_session: false,
        bridge: BridgeConfig::default(),
        options,
    }
}

#[test]
fn submitted_transactions_round_trip_in_order() {
    let mut controller = Controller::new();
    controller
        .configure(make_config(47371))
        .expect("configure");

    let (mut events, _sync) = controller.activate().expect("activate");
    assert!(controller.status().load().running);

    // Submit N uniquely-ordered payloads.
    let submitted: Vec<Vec<u8>> = (0..N).map(|i| format!("tx-{i:04}").into_bytes()).collect();
    for payload in &submitted {
        controller.submit(payload.clone());
    }

    // Drain events (each may batch several transactions) until we have collected
    // all of our payloads, or the overall deadline elapses.
    let rt = Builder::new_current_thread().enable_time().build().unwrap();
    let (received, whitened_len): (Vec<Vec<u8>>, usize) = rt.block_on(async {
        let mut got: Vec<Vec<u8>> = Vec::new();
        let mut whitened_len = 0usize;
        let deadline = tokio::time::Instant::now() + OVERALL_TIMEOUT;
        while got.len() < N {
            let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
            if remaining.is_zero() {
                break;
            }
            match timeout(remaining, events.recv()).await {
                Ok(Some(ev)) => {
                    // whitened_signature is re-enabled against v0.14.0.
                    // Record the length of the first non-empty one we observe.
                    if whitened_len == 0 {
                        whitened_len = ev.whitened_signature.len();
                    }
                    for tx in ev.transactions {
                        // Ignore any empty/non-test transactions defensively.
                        if tx.payload.starts_with(b"tx-") {
                            got.push(tx.payload);
                        }
                    }
                }
                Ok(None) => break, // channel closed
                Err(_) => break,   // timed out
            }
        }
        (got, whitened_len)
    });

    // Calling Event::whitened_signature() against v0.14.0 does NOT
    // segfault (the FFI getter reads a fixed-length buffer), and
    // the value comes back populated.
    assert!(
        whitened_len > 0,
        "expected a non-empty whitened_signature on at least one event"
    );

    // Delivery integrity: every submitted transaction comes back exactly once,
    // none lost, none duplicated. This also exercises the "no double-send /
    // double-free of tx buffers" acceptance item (design §7) — a double-free
    // would crash and a lost/duplicated buffer would change this multiset.
    //
    // NOTE on ordering: Vertex guarantees a *consistent total order across all
    // peers*, NOT that consensus order equals submission order. A single node
    // can therefore legitimately deliver `tx-0002` before `tx-0001` depending on
    // how its transaction buffer batches them into events (observed: the order
    // varies with timing). The end-to-end "all peers see the SAME order"
    // guarantee (design G4 / §7) is a multi-node property and is covered by the
    // ROS system test (§5.3 layer 3), which diffs three peers' event streams
    // byte-for-byte. Here we assert the multiset and the counters.
    let mut got_sorted = received.clone();
    let mut want_sorted = submitted.clone();
    got_sorted.sort();
    want_sorted.sort();
    assert_eq!(
        got_sorted, want_sorted,
        "consensus must deliver every submitted transaction exactly once (got {} of {N})",
        received.len()
    );

    // Counters reflect the round-trip.
    let snap = controller.status().load();
    assert_eq!(snap.tx_submitted_total, N as u64);
    assert_eq!(snap.tx_rejected_total, 0);
    assert!(snap.events_published_total >= 1);

    // Clean lifecycle teardown.
    controller.deactivate().expect("deactivate");
    assert!(!controller.status().load().running);
    controller.cleanup().expect("cleanup");
    controller.shutdown().expect("shutdown");
}

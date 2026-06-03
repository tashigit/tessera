//! Multi-node system test — the core of TAS-76 / design §7 acceptance:
//! *"Events published in Vertex order, transactions ordered within an event"*,
//! verified by **diffing the event streams of all peers byte-for-byte**.
//!
//! The full ROS-level version runs three `vertex_node` processes under
//! `launch_test` (see `vertex_ros2/test/`, requires ROS 2 Jazzy). This test
//! proves the same guarantee where it actually lives — in consensus — by
//! starting three **real** in-process Vertex engines networked over localhost
//! UDP through three `Controller`s, with no ROS dependency, so it runs in plain
//! `cargo test`.
//!
//! The single-node test (`single_node.rs`) could only prove delivery integrity
//! (Vertex order ≠ submission order). With ≥3 peers we can finally assert the
//! real property: every peer observes the *same totally-ordered event stream*.

use std::collections::HashSet;
use std::time::Duration;

use tokio::runtime::Builder;
use tokio::time::timeout;

use tashi_vertex::{KeyPublic, KeySecret};
use vertex_core::bridge::EventReceiver;
use vertex_core::config::{BridgeConfig, Config, PeerConfig, VertexOptions};
use vertex_core::convert::EventRecord;
use vertex_core::Controller;

/// One peer's identity and bind address.
struct Node {
    key: KeySecret,
    public: KeyPublic,
    addr: String,
}

fn options() -> VertexOptions {
    VertexOptions {
        heartbeat_us: 50_000, // tighten so a small session forms events quickly
        report_gossip_events: false,
        ..VertexOptions::default()
    }
}

/// Build the config for node `i`: bind its own address, list every *other* node
/// as a peer (the controller inserts `self`).
fn config_for(i: usize, nodes: &[Node]) -> Config {
    let me = &nodes[i];
    let peers = nodes
        .iter()
        .enumerate()
        .filter(|(j, _)| *j != i)
        .map(|(_, n)| PeerConfig {
            address: n.addr.clone(),
            public: n.public,
        })
        .collect();

    // KeySecret is not Clone; re-parse from this node's DER so each Controller
    // owns its own copy.
    let key = KeySecret::from_der(&me.key.to_der_vec().unwrap()).unwrap();

    Config {
        bind_address: me.addr.clone(),
        key,
        peers,
        joining_running_session: false,
        bridge: BridgeConfig::default(),
        options: options(),
    }
}

/// Drain `rx` until every payload in `needed` has been observed (across events),
/// or the deadline elapses. Returns the full ordered list of events collected.
async fn collect_until(
    rx: &mut EventReceiver,
    needed: &HashSet<Vec<u8>>,
    overall: Duration,
) -> Vec<EventRecord> {
    let mut events = Vec::new();
    let mut seen: HashSet<Vec<u8>> = HashSet::new();
    let deadline = tokio::time::Instant::now() + overall;
    while seen.len() < needed.len() {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        if remaining.is_zero() {
            break;
        }
        match timeout(remaining, rx.recv()).await {
            Ok(Some(ev)) => {
                for tx in &ev.transactions {
                    if needed.contains(&tx.payload) {
                        seen.insert(tx.payload.clone());
                    }
                }
                events.push(ev);
            }
            Ok(None) | Err(_) => break,
        }
    }
    events
}

#[test]
fn three_peers_observe_identical_event_streams() {
    const BASE_PORT: u16 = 47411;
    const PER_NODE_TX: usize = 4; // each of 3 nodes submits this many → 12 total

    // --- identities ---
    let nodes: Vec<Node> = (0..3)
        .map(|i| {
            let key = KeySecret::generate();
            let public = key.public();
            Node {
                key,
                public,
                addr: format!("127.0.0.1:{}", BASE_PORT + i as u16),
            }
        })
        .collect();

    // --- bring up all three engines ---
    let mut controllers: Vec<Controller> = Vec::new();
    let mut receivers: Vec<EventReceiver> = Vec::new();
    for i in 0..nodes.len() {
        let mut c = Controller::new();
        c.configure(config_for(i, &nodes)).expect("configure");
        let (event_rx, _sync_rx) = c.activate().expect("activate");
        controllers.push(c);
        receivers.push(event_rx);
    }

    // --- submit a distinct, attributable payload set from every peer ---
    let mut needed: HashSet<Vec<u8>> = HashSet::new();
    for (i, c) in controllers.iter().enumerate() {
        for k in 0..PER_NODE_TX {
            let payload = format!("node{i}-tx{k:03}").into_bytes();
            needed.insert(payload.clone());
            c.submit(payload);
        }
    }

    // --- collect each peer's event stream until all txs have landed ---
    let rt = Builder::new_current_thread().enable_time().build().unwrap();
    let streams: Vec<Vec<EventRecord>> = rt.block_on(async {
        let mut out = Vec::new();
        for rx in receivers.iter_mut() {
            out.push(collect_until(rx, &needed, Duration::from_secs(40)).await);
        }
        out
    });

    // Every peer must have delivered every submitted transaction.
    for (i, s) in streams.iter().enumerate() {
        let delivered: HashSet<Vec<u8>> = s
            .iter()
            .flat_map(|ev| ev.transactions.iter().map(|t| t.payload.clone()))
            .collect();
        for want in &needed {
            assert!(
                delivered.contains(want),
                "node {i} never delivered {:?}",
                String::from_utf8_lossy(want)
            );
        }
    }

    // THE guarantee: all peers observe the same totally-ordered event stream.
    // Consensus is deterministic and every event's fields (consensus_at,
    // created_at, hash, creator, transactions) are event-intrinsic — identical
    // for every observer — so the ordered prefixes must be byte-for-byte equal.
    let min_len = streams.iter().map(|s| s.len()).min().unwrap();
    assert!(min_len > 0, "no events collected");
    for idx in 0..min_len {
        let e0 = &streams[0][idx];
        for (n, s) in streams.iter().enumerate().skip(1) {
            assert_eq!(
                &s[idx], e0,
                "event #{idx} differs between peer 0 and peer {n} \
                 (consensus order divergence)"
            );
        }
    }

    // Clean teardown of all peers.
    for mut c in controllers {
        let _ = c.shutdown();
    }
}

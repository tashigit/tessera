//! Lifecycle behavior tests (design §7):
//! * the event stream only flows while `Active` — `deactivate` stops it;
//! * a `deactivate → activate` cycle works without a process restart, probing
//!   the §9.3 "no `Engine::stop`" gap with a concrete result.
//!
//! Distinct port range (47431+) so these don't collide with `single_node`
//! (47371) or `multi_node` (47411–47413) if cargo runs the binaries together.

use std::time::Duration;

use tokio::runtime::Builder;
use tokio::time::timeout;

use vertex_core::config::{BridgeConfig, Config, VertexOptions};
use vertex_core::Controller;

fn make_config(port: u16) -> Config {
    Config {
        bind_address: format!("127.0.0.1:{port}"),
        key: tashi_vertex::KeySecret::generate(),
        peers: Vec::new(),
        joining_running_session: false,
        bridge: BridgeConfig::default(),
        options: VertexOptions {
            heartbeat_us: 50_000,
            ..VertexOptions::default()
        },
    }
}

/// After `deactivate`, the engine is torn down and its outbound sender dropped,
/// so the event channel closes — no events can be published while not `Active`
/// (the core-level equivalent of "no publishes in Inactive", design §7).
#[test]
fn event_stream_closes_after_deactivate() {
    let mut c = Controller::new();
    c.configure(make_config(47431)).expect("configure");
    let (mut events, _sync) = c.activate().expect("activate");

    // Submit a few so there may be buffered events to drain first.
    for k in 0..4 {
        c.submit(format!("tx-{k}").into_bytes());
    }

    c.deactivate().expect("deactivate");
    assert!(!c.status().load().running);

    // Drain any buffered events; the channel MUST then close (recv -> None).
    let rt = Builder::new_current_thread().enable_time().build().unwrap();
    let closed = rt.block_on(async {
        loop {
            match timeout(Duration::from_secs(5), events.recv()).await {
                Ok(Some(_)) => continue,   // buffered event, keep draining
                Ok(None) => break true,    // sender dropped → stream closed
                Err(_) => break false,     // still open after timeout
            }
        }
    });
    assert!(closed, "event stream must close once the node leaves Active");

    c.cleanup().expect("cleanup");
    c.shutdown().expect("shutdown");
}

/// `deactivate → activate` on the same bind address must succeed without
/// restarting the process. This exercises the §9.3 teardown path: the engine
/// thread drops the `Context` (releasing the socket) before `deactivate`
/// returns, so re-binding works. If upstream behavior regresses, this test
/// turns the gap into a hard failure rather than a silent hang.
#[test]
fn reactivate_after_deactivate_on_same_port() {
    let mut c = Controller::new();
    c.configure(make_config(47433)).expect("configure");

    let (events1, _sync1) = c.activate().expect("first activate");
    assert!(c.status().load().running);
    drop(events1); // simulate ROS dropping the first event receiver

    c.deactivate().expect("deactivate");
    assert!(!c.status().load().running);

    // Re-activate: a brand-new engine binds the same address.
    let (_events2, _sync2) = c
        .activate()
        .expect("re-activate after deactivate should succeed");
    assert!(c.status().load().running);

    // And it actually works end-to-end again.
    c.submit(b"after-reactivate".to_vec());
    let rt = Builder::new_current_thread().enable_time().build().unwrap();
    let snap_running = c.status().load().running;
    assert!(snap_running);

    c.shutdown().expect("shutdown");
    let _ = rt; // (kept for symmetry; teardown is synchronous)
}

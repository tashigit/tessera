//! The `vertex_node` ROS 2 node: maps the tested [`vertex_core`] controller onto
//! the ROS-facing contract (design §3.3).
//!
//! | Resource | Direction | Type |
//! |---|---|---|
//! | `/vertex/tx` | sub | `vertex_ros2_msgs/msg/VertexTransaction` |
//! | `/vertex/event` | pub | `vertex_ros2_msgs/msg/VertexEvent` |
//! | `/vertex/sync_point` | pub | `vertex_ros2_msgs/msg/VertexSyncPoint` |
//! | `/vertex/status` | service | `vertex_ros2_msgs/srv/VertexStatus` |
//! | `/diagnostics` | pub | `diagnostic_msgs/msg/DiagnosticArray` |
//! | `/vertex/transition` | service | lifecycle control (see below) |
//! | `/vertex/lifecycle/state` | pub (latched) | `std_msgs/msg/String` |
//!
//! # Lifecycle (design §8.4 fallback #2)
//!
//! `rclrs` does not ship a `LifecycleNode`. Re-checked 2026-06-09 (TAS-76 scope
//! item 2) against `ros2-rust/ros2_rust` `main` (post-v0.7.0): `rclrs/src` still
//! has no `lifecycle`/`LifecycleNode` module — it carries nodes, services,
//! parameters, timers, and (newly) actions, but no managed-node support.
//! **Decision: keep the `/vertex/transition` service fallback.**
//!
//! So, rather than block on upstream, we expose the managed-node state machine
//! through a `/vertex/transition` service (verbs: `configure`/`activate`/
//! `deactivate`/`cleanup`/`shutdown`) and publish the current primary state on a latched
//! `/vertex/lifecycle/state` topic. The state machine itself is
//! [`vertex_core::lifecycle`], identical to what a real `LifecycleNode` would
//! drive — so migrating to native lifecycle later is a localized change here,
//! not in the engine path. This is a sibling control plane, NOT the
//! `ros2 lifecycle` CLI; the deviation is documented in the README.
//!
//! NOTE: this is the colcon-built layer. `vertex_core` carries all the logic the
//! test suite verifies; this file is the thin `rclrs` adapter.

use std::sync::{Arc, Mutex};

use rclrs::*;

use builtin_interfaces::msg::Time as RosTime;
use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
use std_msgs::msg::String as RosString;
use vertex_ros2_msgs::msg::{
    VertexEvent as RosEvent, VertexSyncPoint as RosSyncPoint, VertexTransaction as RosTransaction,
};
use vertex_ros2_msgs::srv::{
    VertexStatus, VertexStatus_Request, VertexStatus_Response, VertexTransition,
    VertexTransition_Request, VertexTransition_Response,
};

use vertex_core::convert::{EventRecord, StampedTime, SyncPointRecord};
use vertex_core::lifecycle::{LifecycleState, Transition};
use vertex_core::{Controller, StatusSnapshot};

use crate::params;

const TOPIC_TX: &str = "/vertex/tx";
const TOPIC_EVENT: &str = "/vertex/event";
const TOPIC_SYNC: &str = "/vertex/sync_point";
const TOPIC_STATE: &str = "/vertex/lifecycle/state";
const TOPIC_DIAG: &str = "/diagnostics";
const SRV_STATUS: &str = "/vertex/status";
const SRV_TRANSITION: &str = "/vertex/transition";

// ---------------------------------------------------------------------------
// core record -> ROS message conversion
// ---------------------------------------------------------------------------

fn to_ros_time(t: StampedTime) -> RosTime {
    RosTime {
        sec: t.sec,
        nanosec: t.nanosec,
    }
}

fn to_ros_event(rec: EventRecord) -> RosEvent {
    let mut hash = [0u8; 32];
    hash.copy_from_slice(&rec.hash);
    RosEvent {
        consensus_at: to_ros_time(rec.consensus_at),
        created_at: to_ros_time(rec.created_at),
        hash,
        creator_pub_der: rec.creator_pub_der.into(),
        whitened_signature: rec.whitened_signature.into(),
        transactions: rec
            .transactions
            .into_iter()
            .map(|t| RosTransaction {
                // Vertex carries only opaque payload bytes; created_at/tag are
                // not recoverable per-transaction (see README "Semantics").
                created_at: RosTime::default(),
                payload: t.payload.into(),
                tag: String::new().into(),
            })
            .collect(),
    }
}

fn to_ros_sync_point(rec: SyncPointRecord) -> RosSyncPoint {
    RosSyncPoint {
        observed_at: to_ros_time(rec.observed_at),
        payload: rec.payload.into(),
    }
}

// ---------------------------------------------------------------------------
// node
// ---------------------------------------------------------------------------

/// QoS for a latched (transient-local, keep-last-1) state topic so late
/// subscribers immediately receive the current lifecycle state.
fn latched_qos() -> QoSProfile {
    QoSProfile {
        history: QoSHistoryPolicy::KeepLast { depth: 1 },
        durability: QoSDurabilityPolicy::TransientLocal,
        reliability: QoSReliabilityPolicy::Reliable,
        ..QOS_PROFILE_DEFAULT
    }
}

pub struct VertexNode {
    // `rclrs`'s `Node`/`Publisher`/`Subscription`/`Service` are already
    // `Arc`-wrapped type aliases (e.g. `Publisher<T>` == `Arc<PublisherState<T>>`),
    // so they are stored directly — no extra `Arc<…>` wrapper.
    node: Node,
    controller: Arc<Mutex<Controller>>,
    event_pub: Publisher<RosEvent>,
    sync_pub: Publisher<RosSyncPoint>,
    diag_pub: Publisher<DiagnosticArray>,
    state_pub: Publisher<RosString>,
    diag_period_s: f64,
    // Kept alive for as long as the node runs.
    _tx_sub: Subscription<RosTransaction>,
    _status_srv: Service<VertexStatus>,
    _transition_srv: Service<VertexTransition>,
}

impl VertexNode {
    pub fn new(executor: &Executor) -> Result<Arc<Self>, RclrsError> {
        let node = executor.create_node("vertex_node")?;
        let controller = Arc::new(Mutex::new(Controller::new()));

        let event_pub = node.create_publisher::<RosEvent>(TOPIC_EVENT)?;
        let sync_pub = node.create_publisher::<RosSyncPoint>(TOPIC_SYNC)?;
        let diag_pub = node.create_publisher::<DiagnosticArray>(TOPIC_DIAG)?;
        let state_pub =
            node.create_publisher::<RosString>(TOPIC_STATE.qos(latched_qos()))?;

        // --- /vertex/tx subscription: forward opaque bytes into the engine ---
        let ctrl_tx = controller.clone();
        let tx_sub = node.create_subscription::<RosTransaction, _>(
            TOPIC_TX,
            move |msg: RosTransaction| {
                // created_at / tag are advisory and ignored in v0.1.
                let payload: Vec<u8> = msg.payload.to_vec();
                if let Ok(c) = ctrl_tx.lock() {
                    c.submit(payload);
                }
            },
        )?;

        // --- /vertex/status service ---
        let ctrl_status = controller.clone();
        let status_srv = node.create_service::<VertexStatus, _>(
            SRV_STATUS,
            move |_req: VertexStatus_Request| -> VertexStatus_Response {
                let snap = ctrl_status
                    .lock()
                    .map(|c| c.status().load())
                    .unwrap_or_default();
                status_response(&snap)
            },
        )?;

        // Diagnostics period (also declares the parameter).
        let diag_period_s: f64 = node
            .declare_parameter("diagnostics.period_s")
            .default(1.0)
            .mandatory()
            .map(|p| p.get())
            .unwrap_or(1.0);

        let this_pre = Arc::new(VertexNodePre {
            node: node.clone(),
            controller: controller.clone(),
            event_pub: event_pub.clone(),
            sync_pub: sync_pub.clone(),
            state_pub: state_pub.clone(),
        });

        // --- /vertex/transition service: lifecycle control plane (§8.4) ---
        let pre = this_pre.clone();
        let transition_srv = node.create_service::<VertexTransition, _>(
            SRV_TRANSITION,
            move |req: VertexTransition_Request| -> VertexTransition_Response {
                let verb = req.transition.to_string();
                let outcome = pre.apply_transition_str(&verb);
                VertexTransition_Response {
                    success: outcome.success,
                    state: outcome.state.label().to_string().into(),
                    message: outcome.message.into(),
                }
            },
        )?;

        let this = Arc::new(VertexNode {
            node,
            controller,
            event_pub,
            sync_pub,
            diag_pub,
            state_pub,
            diag_period_s,
            _tx_sub: tx_sub,
            _status_srv: status_srv,
            _transition_srv: transition_srv,
        });

        this.publish_state(LifecycleState::Unconfigured);
        this.spawn_diagnostics();
        Ok(this)
    }

    fn publish_state(&self, state: LifecycleState) {
        let _ = self.state_pub.publish(RosString {
            data: state.label().to_string().into(),
        });
    }

    /// Publish a `diagnostic_msgs/DiagnosticArray` at `diagnostics.period_s`
    /// (design §5.2). Runs on its own thread; reads the lock-free snapshot.
    fn spawn_diagnostics(self: &Arc<Self>) {
        let weak = Arc::downgrade(self);
        let period = std::time::Duration::from_secs_f64(self.diag_period_s.max(0.05));
        std::thread::Builder::new()
            .name("vertex-diagnostics".into())
            .spawn(move || loop {
                std::thread::sleep(period);
                let Some(node) = weak.upgrade() else { break };
                let snap = node
                    .controller
                    .lock()
                    .map(|c| c.status().load())
                    .unwrap_or_default();
                let _ = node.diag_pub.publish(diagnostics_msg(&snap));
            })
            .ok();
    }
}

/// Result of a lifecycle transition, mapped onto `VertexTransition.srv`.
struct TransitionOutcome {
    success: bool,
    state: LifecycleState,
    message: String,
}

/// The subset of node state the transition/pump logic needs, shared so the
/// service closure does not capture the whole `VertexNode` (avoids a cycle).
struct VertexNodePre {
    node: Node,
    controller: Arc<Mutex<Controller>>,
    event_pub: Publisher<RosEvent>,
    sync_pub: Publisher<RosSyncPoint>,
    state_pub: Publisher<RosString>,
}

impl VertexNodePre {
    fn publish_state(&self, state: LifecycleState) {
        let _ = self.state_pub.publish(RosString {
            data: state.label().to_string().into(),
        });
    }

    fn apply_transition_str(&self, verb: &str) -> TransitionOutcome {
        let mut ctrl = match self.controller.lock() {
            Ok(c) => c,
            Err(_) => {
                return TransitionOutcome {
                    success: false,
                    state: LifecycleState::Finalized,
                    message: "controller lock poisoned".into(),
                }
            }
        };

        let Some(transition) = Transition::parse(verb) else {
            return TransitionOutcome {
                success: false,
                state: ctrl.state(),
                message: format!("unknown transition {verb:?}"),
            };
        };

        let result: Result<(), String> = match transition {
            Transition::Configure => match params::build_config(&self.node) {
                Ok(cfg) => ctrl.configure(cfg).map_err(|e| e.to_string()),
                Err(e) => Err(format!("configure failed: {e}")),
            },
            Transition::Activate => match ctrl.activate() {
                Ok((event_rx, sync_rx)) => {
                    self.spawn_pumps(event_rx, sync_rx);
                    Ok(())
                }
                Err(e) => Err(e.to_string()),
            },
            Transition::Deactivate => ctrl.deactivate().map_err(|e| e.to_string()),
            Transition::Cleanup => ctrl.cleanup().map_err(|e| e.to_string()),
            Transition::Shutdown => ctrl.shutdown().map_err(|e| e.to_string()),
        };

        let state = ctrl.state();
        drop(ctrl);
        self.publish_state(state);

        match result {
            Ok(()) => TransitionOutcome {
                success: true,
                state,
                message: String::new(),
            },
            Err(message) => {
                log_error(&self.node, &message);
                TransitionOutcome {
                    success: false,
                    state,
                    message,
                }
            }
        }
    }

    /// Spawn the two outbound pumps: consensus events and sync points. Each
    /// blocks on its channel and publishes onto ROS; both exit when the engine
    /// stops (the controller drops the senders → `blocking_recv` returns `None`).
    fn spawn_pumps(
        &self,
        mut event_rx: vertex_core::bridge::EventReceiver,
        mut sync_rx: vertex_core::bridge::SyncPointReceiver,
    ) {
        let event_pub = self.event_pub.clone();
        std::thread::Builder::new()
            .name("vertex-event-pump".into())
            .spawn(move || {
                while let Some(rec) = event_rx.blocking_recv() {
                    let _ = event_pub.publish(to_ros_event(rec));
                }
            })
            .ok();

        let sync_pub = self.sync_pub.clone();
        std::thread::Builder::new()
            .name("vertex-sync-pump".into())
            .spawn(move || {
                while let Some(rec) = sync_rx.blocking_recv() {
                    let _ = sync_pub.publish(to_ros_sync_point(rec));
                }
            })
            .ok();
    }
}

fn status_response(snap: &StatusSnapshot) -> VertexStatus_Response {
    VertexStatus_Response {
        running: snap.running,
        bind_address: snap.bind_address.clone().into(),
        peer_count: snap.peer_count,
        tx_submitted_total: snap.tx_submitted_total,
        tx_rejected_total: snap.tx_rejected_total,
        events_published_total: snap.events_published_total,
        last_error_code: snap.last_error_code,
        last_error_message: snap.last_error_message.clone().into(),
    }
}

fn diagnostics_msg(snap: &StatusSnapshot) -> DiagnosticArray {
    let level = if snap.last_error_code != 0 {
        DiagnosticStatus::WARN
    } else {
        DiagnosticStatus::OK
    };
    let kv = |k: &str, v: String| KeyValue {
        key: k.to_string().into(),
        value: v.into(),
    };
    let status = DiagnosticStatus {
        level,
        name: "vertex_node".to_string().into(),
        message: if snap.running { "running" } else { "stopped" }
            .to_string()
            .into(),
        hardware_id: snap.bind_address.clone().into(),
        values: vec![
            kv("running", snap.running.to_string()),
            kv("peer_count", snap.peer_count.to_string()),
            kv("tx_submitted_total", snap.tx_submitted_total.to_string()),
            kv("tx_rejected_total", snap.tx_rejected_total.to_string()),
            kv(
                "events_published_total",
                snap.events_published_total.to_string(),
            ),
            kv("last_error_code", snap.last_error_code.to_string()),
            kv("last_error_message", snap.last_error_message.clone()),
        ]
        .into(),
    };
    DiagnosticArray {
        header: Default::default(),
        status: vec![status].into(),
    }
}

fn log_error(node: &Node, msg: &str) {
    // rclrs logging macro; falls back to eprintln if unavailable in the distro.
    let _ = node;
    eprintln!("[vertex_node] ERROR: {msg}");
}

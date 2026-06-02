//! The lifecycle controller: orchestrates the ROS 2 managed-node callbacks
//! (`on_configure` / `on_activate` / `on_deactivate` / `on_cleanup` /
//! `on_shutdown`, design §4.4) and owns the engine's execution thread.
//!
//! # Why a dedicated thread
//!
//! `tashi_vertex` handles (`Context`, `Engine`, `Socket`, `Peers`, `Options`)
//! wrap `NonNull` and are therefore `!Send`. They can never be moved across
//! threads or `tokio::spawn`ed. So the engine lives on one dedicated OS thread
//! running its own current-thread Tokio runtime, exactly like the upstream
//! `pingback` example drives the engine via `block_on`. Only `Send` data
//! (payload `Vec<u8>`, [`EventRecord`](crate::EventRecord), DER-encoded keys)
//! crosses the channel boundary between that thread and the ROS executor.

use std::sync::mpsc as std_mpsc;
use std::thread;

use tashi_vertex::{Context, Engine, KeyPublic, KeySecret, PeerCapabilities, Peers, Socket};
use thiserror::Error;
use tokio_util::sync::CancellationToken;

use crate::bridge::{self, EventReceiver, SyncPointReceiver, TxSender};
use crate::config::{Config, ConfigError, VertexOptions};
use crate::engine_task::EngineTask;
use crate::lifecycle::{LifecycleError, LifecycleState, Transition};
use crate::status::Status;

#[derive(Debug, Error)]
pub enum ControllerError {
    #[error(transparent)]
    Lifecycle(#[from] LifecycleError),
    #[error(transparent)]
    Config(#[from] ConfigError),
    #[error("vertex engine error: {0:?}")]
    Vertex(tashi_vertex::Error),
    #[error("engine startup failed: {0}")]
    Startup(String),
    #[error("no configuration; call configure() first")]
    NotConfigured,
    #[error("node is not active")]
    NotActive,
}

impl From<tashi_vertex::Error> for ControllerError {
    fn from(e: tashi_vertex::Error) -> Self {
        ControllerError::Vertex(e)
    }
}

/// `Send` bundle of everything needed to start the engine, derived from a
/// [`Config`] on the ROS thread and moved onto the engine thread. Keys travel
/// as DER bytes and are re-parsed on the far side, so no `!Send` Vertex type
/// crosses the boundary.
struct EngineParams {
    bind_address: String,
    secret_der: Vec<u8>,
    peers: Vec<(String, Vec<u8>)>, // (address, public-key DER)
    options: VertexOptions,
    joining_running_session: bool,
}

impl EngineParams {
    fn from_config(cfg: &Config) -> Result<Self, ControllerError> {
        let secret_der = cfg.key.to_der_vec().map_err(ControllerError::Vertex)?;
        let mut peers = Vec::with_capacity(cfg.peers.len());
        for p in &cfg.peers {
            let der = p.public.to_der_vec().map_err(ControllerError::Vertex)?;
            peers.push((p.address.clone(), der));
        }
        Ok(EngineParams {
            bind_address: cfg.bind_address.clone(),
            secret_der,
            peers,
            options: cfg.options.clone(),
            joining_running_session: cfg.joining_running_session,
        })
    }
}

/// Build a live engine on the engine thread (design Appendix A). Returns the
/// `Engine` together with the `Context` it borrows; the caller MUST keep the
/// `Context` alive for at least as long as the `Engine` (design §4.8).
async fn build_engine(params: &EngineParams) -> Result<(Engine, Context), ControllerError> {
    let ctx = Context::new()?;
    let socket = Socket::bind(&ctx, &params.bind_address).await?;
    let options = params.options.build();
    let key = KeySecret::from_der(&params.secret_der)?;

    let mut peers = Peers::with_capacity(params.peers.len() + 1)?;
    for (addr, der) in &params.peers {
        let pk = KeyPublic::from_der(der)?;
        peers.insert(addr, &pk, PeerCapabilities::default())?;
    }
    // Always insert ourself.
    peers.insert(&params.bind_address, &key.public(), PeerCapabilities::default())?;

    let engine = Engine::start(
        &ctx,
        socket,
        options,
        &key,
        peers,
        params.joining_running_session,
    )?;
    Ok((engine, ctx))
}

/// State for an active session: the running engine thread and the inbound sender.
struct Active {
    cancel: CancellationToken,
    thread: thread::JoinHandle<()>,
    tx_in: TxSender,
}

/// The lifecycle controller. Not tied to ROS — the `ros` layer holds one of
/// these and drives it from lifecycle-transition service calls.
pub struct Controller {
    status: Status,
    state: LifecycleState,
    config: Option<Config>,
    active: Option<Active>,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    pub fn new() -> Self {
        Controller {
            status: Status::new(),
            state: LifecycleState::Unconfigured,
            config: None,
            active: None,
        }
    }

    pub fn state(&self) -> LifecycleState {
        self.state
    }

    pub fn status(&self) -> &Status {
        &self.status
    }

    /// `on_configure`: Unconfigured → Inactive. Stores the validated config; no
    /// I/O, no engine yet.
    pub fn configure(&mut self, config: Config) -> Result<(), ControllerError> {
        let next = self.state.target(Transition::Configure)?;
        self.config = Some(config);
        self.state = next;
        Ok(())
    }

    /// `on_activate`: Inactive → Active. Spawns the engine thread, binds the
    /// socket, starts the engine, and begins the receive loop. Returns the
    /// outbound receivers for the ROS layer to pump onto `/vertex/event` and
    /// `/vertex/sync_point`.
    pub fn activate(
        &mut self,
    ) -> Result<(EventReceiver, SyncPointReceiver), ControllerError> {
        let next = self.state.target(Transition::Activate)?;
        let cfg = self.config.as_ref().ok_or(ControllerError::NotConfigured)?;

        let params = EngineParams::from_config(cfg)?;
        let bind_address = cfg.bind_address.clone();
        let peer_count = cfg.peer_count();

        let (tx_in_tx, tx_in_rx) = bridge::tx_channel(cfg.bridge.tx_channel_capacity);
        let (event_tx, event_rx) = bridge::event_channel(cfg.bridge.event_channel_capacity);
        let (sync_tx, sync_rx) = bridge::sync_point_channel();

        let cancel = CancellationToken::new();
        let cancel_engine = cancel.clone();
        let status_engine = self.status.clone();

        // The engine thread reports startup success/failure before entering the
        // receive loop, so activation fails synchronously if bind/start fails.
        let (startup_tx, startup_rx) = std_mpsc::sync_channel::<Result<(), String>>(1);

        let thread = thread::Builder::new()
            .name("vertex-engine".into())
            .spawn(move || {
                let rt = match tokio::runtime::Builder::new_current_thread()
                    .enable_time()
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        let _ = startup_tx.send(Err(format!("tokio runtime: {e}")));
                        return;
                    }
                };

                rt.block_on(async move {
                    match build_engine(&params).await {
                        Ok((engine, ctx)) => {
                            // Signal success, then run until cancelled/closed.
                            let _ = startup_tx.send(Ok(()));
                            let task = EngineTask {
                                engine,
                                tx_in: tx_in_rx,
                                event_out: event_tx,
                                sync_out: sync_tx,
                                status: status_engine,
                                cancel: cancel_engine,
                            };
                            let _reason = task.run().await;
                            // Engine drops with `task`; keep ctx alive until now
                            // (design §4.8 — dropping the Context is the only
                            // teardown path; there is no Engine::stop, §9.3).
                            drop(ctx);
                        }
                        Err(e) => {
                            let _ = startup_tx.send(Err(e.to_string()));
                        }
                    }
                });
            })
            .map_err(|e| ControllerError::Startup(format!("spawn engine thread: {e}")))?;

        // Wait for the engine to come up (or fail).
        match startup_rx.recv() {
            Ok(Ok(())) => {}
            Ok(Err(msg)) => {
                let _ = thread.join();
                return Err(ControllerError::Startup(msg));
            }
            Err(_) => {
                let _ = thread.join();
                return Err(ControllerError::Startup(
                    "engine thread exited before signalling startup".into(),
                ));
            }
        }

        self.active = Some(Active {
            cancel,
            thread,
            tx_in: tx_in_tx,
        });
        self.status.set_running(true, &bind_address, peer_count);
        self.state = next;
        Ok((event_rx, sync_rx))
    }

    /// `on_deactivate`: Active → Inactive. Cancels the receive loop, closes the
    /// inbound channel, and joins the engine thread. The config is retained.
    pub fn deactivate(&mut self) -> Result<(), ControllerError> {
        let next = self.state.target(Transition::Deactivate)?;
        self.stop_active();
        self.status.set_stopped();
        self.state = next;
        Ok(())
    }

    /// `on_cleanup`: Inactive → Unconfigured. Drops the stored config.
    pub fn cleanup(&mut self) -> Result<(), ControllerError> {
        let next = self.state.target(Transition::Cleanup)?;
        self.config = None;
        self.status.set_stopped();
        self.state = next;
        Ok(())
    }

    /// `on_shutdown`: any → Finalized. Best-effort, idempotent teardown.
    pub fn shutdown(&mut self) -> Result<(), ControllerError> {
        let next = self.state.target(Transition::Shutdown)?;
        self.stop_active();
        self.config = None;
        self.status.set_stopped();
        self.state = next;
        Ok(())
    }

    /// Submit opaque bytes from a `/vertex/tx` callback. A full or closed
    /// inbound channel is backpressure and counts toward `tx_rejected_total`
    /// (design §4.6); accepted submissions are counted on the engine thread
    /// once `send_transaction` succeeds.
    pub fn submit(&self, payload: Vec<u8>) {
        match &self.active {
            Some(active) => {
                if active.tx_in.try_send(payload).is_err() {
                    self.status.record_rejected(None);
                }
            }
            None => self.status.record_rejected(None),
        }
    }

    fn stop_active(&mut self) {
        if let Some(active) = self.active.take() {
            active.cancel.cancel();
            drop(active.tx_in); // close inbound so the task observes EOF too
            let _ = active.thread.join();
        }
    }
}

impl Drop for Controller {
    fn drop(&mut self) {
        self.stop_active();
    }
}

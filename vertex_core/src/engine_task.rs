//! The pair of cooperative tasks that own the `tashi_vertex::Engine`.
//!
//! # Why not one `select!` loop
//!
//! The design doc (§4.7) sketches a single `tokio::select!` that races
//! `recv_message()` against the inbound channel. That is **unsound** against the
//! actual `tashi-vertex` FFI: `recv_message()`'s future registers a pointer to
//! its own stack/heap cell with the C library on first poll (`tv_message_recv`'s
//! `user_data`). `select!` drops the losing branch each iteration, so whenever
//! the inbound branch wins, the in-flight `recv` future is dropped and the C
//! callback later writes into freed memory — a segfault. `recv_message` is not
//! cancellation-safe.
//!
//! So instead we run two tasks on a single-thread `LocalSet` (the engine handle
//! is `!Send`, so everything stays on one thread) sharing an `Rc<Engine>`:
//!
//! * **recv loop** — calls `recv_message().await` in a plain loop that is *never*
//!   cancelled mid-flight. It exits only *between* messages (a stop flag) or when
//!   its outbound channel closes. Vertex's heartbeat (≤500 ms) guarantees the
//!   loop wakes regularly to observe the stop flag.
//! * **send loop** — `select!`s only over cancellation-safe futures
//!   (`tx_in.recv()` and the cancel token), so dropping the losing branch is safe.
//!
//! Ordering is still exact: a single recv loop forwards events in the order
//! `recv_message` yields them, the bounded channel is FIFO, and
//! `Event::transactions()` iterates by index 0..n within an event.

use std::rc::Rc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

use tashi_vertex::{Engine, Message, Transaction};
use tokio::task::{self, LocalSet};
use tokio_util::sync::CancellationToken;

use crate::bridge::{EventSender, SyncPointSender, TxReceiver};
use crate::convert::{nanos_to_time, EventRecord, StampedTime, SyncPointRecord};
use crate::status::Status;

/// Inputs to the engine task. Grouped to keep the signature readable.
pub struct EngineTask {
    pub engine: Engine,
    pub tx_in: TxReceiver,
    pub event_out: EventSender,
    pub sync_out: SyncPointSender,
    pub status: Status,
    pub cancel: CancellationToken,
}

fn now_stamped() -> StampedTime {
    match SystemTime::now().duration_since(UNIX_EPOCH) {
        Ok(d) => nanos_to_time(d.as_nanos().min(u64::MAX as u128) as u64),
        Err(_) => StampedTime::default(),
    }
}

impl EngineTask {
    /// Drive the engine until cancellation. Consumes `self`; on cancellation it
    /// calls `Engine::stop` to wind the engine down, then drops the handle on
    /// return (design §4.8). This future must be
    /// awaited via `block_on`/`run_until` on a current-thread runtime — it spawns
    /// `!Send` local tasks and must never itself be cancelled while a
    /// `recv_message` is in flight.
    pub async fn run(self) -> () {
        let EngineTask {
            engine,
            tx_in,
            event_out,
            sync_out,
            status,
            cancel,
        } = self;

        let engine = Rc::new(engine);
        let stop = Rc::new(AtomicBool::new(false));

        let local = LocalSet::new();
        local
            .run_until(async move {
                let recv = task::spawn_local(recv_loop(
                    engine.clone(),
                    event_out,
                    sync_out,
                    status.clone(),
                    stop.clone(),
                ));
                let send = task::spawn_local(send_loop(
                    engine.clone(),
                    tx_in,
                    status.clone(),
                    cancel.clone(),
                ));

                // Wait for the shutdown signal, then wind both loops down without
                // cancelling an in-flight recv.
                cancel.cancelled().await;
                stop.store(true, Ordering::Relaxed);
                // Signal the engine to wind down (TAS-93 / §9.3). This makes an
                // in-flight `recv_message` resolve to `None` even when the
                // session has stalled and no heartbeat would wake the loop, so
                // the recv loop is guaranteed to observe `stop` and exit. Without
                // it a stalled engine could block `recv_message` forever.
                let _ = engine.stop();

                let _ = send.await; // returns promptly: it observes `cancel`
                let _ = recv.await; // returns once the engine winds down
            })
            .await;
        // `engine` (Rc) drops here, then the caller drops the Context.
    }
}

/// Forward consensus messages to ROS. Never cancelled mid-`recv_message`; exits
/// only between messages (stop flag) or when an output channel closes.
async fn recv_loop(
    engine: Rc<Engine>,
    event_out: EventSender,
    sync_out: SyncPointSender,
    status: Status,
    stop: Rc<AtomicBool>,
) {
    loop {
        if stop.load(Ordering::Relaxed) {
            break;
        }
        match engine.recv_message().await {
            Ok(Some(Message::Event(ev))) => {
                let record = EventRecord::from_event(&ev);
                drop(ev); // FFI-owned bytes already copied into `record`
                if event_out.send(record).await.is_err() {
                    break; // ROS dropped the /vertex/event receiver
                }
                status.record_event_published();
            }
            Ok(Some(Message::SyncPoint(_sp))) => {
                let record = SyncPointRecord {
                    observed_at: now_stamped(),
                    payload: Vec::new(), // empty until §9.2 lands
                };
                // A slow sync-point consumer must not wedge consensus.
                let _ = sync_out.try_send(record);
            }
            // A message kind we do not model: ignore and keep listening
            // (design §4.7, "None => continue").
            Ok(None) => continue,
            Err(e) => {
                status.record_error(e);
                break;
            }
        }
    }
}

/// Drain inbound transactions onto the engine. Only selects over
/// cancellation-safe futures, so it can be torn down cleanly.
async fn send_loop(
    engine: Rc<Engine>,
    mut tx_in: TxReceiver,
    status: Status,
    cancel: CancellationToken,
) {
    loop {
        tokio::select! {
            biased;
            _ = cancel.cancelled() => break,
            maybe_payload = tx_in.recv() => match maybe_payload {
                Some(payload) => submit(&engine, payload, &status),
                None => break, // ROS dropped the inbound sender
            }
        }
    }
}

/// Allocate a Vertex transaction, copy the payload in, and submit it. Success →
/// `tx_submitted_total`; engine rejection → `tx_rejected_total`.
fn submit(engine: &Engine, payload: Vec<u8>, status: &Status) {
    let mut tx = Transaction::allocate(payload.len());
    tx.copy_from_slice(&payload);
    match engine.send_transaction(tx) {
        Ok(()) => status.record_submitted(),
        Err(e) => status.record_rejected(Some(e)),
    }
}

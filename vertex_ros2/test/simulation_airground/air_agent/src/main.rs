//! air_agent — one per drone in the air/ground simulation.
//!
//! A full member of the same Vertex committee as tessera's `vertex_node`
//! processes, reached by linking `tashi-vertex` directly. There is no rclrs
//! here, no DDS, no message package, no rosbridge: the drone tier shares with
//! the ground tier exactly two things, the engine protocol version and the
//! record schema. That is the claim simulation 3 exists to demonstrate, and
//! keeping this binary ROS-free is what makes it checkable.
//!
//! The `!Send` FFI handles (`Engine`, `Context`, `Socket`) mean the engine
//! cannot move between threads, so everything runs on a current-thread runtime
//! in one `select!` loop rather than spawned tasks. This mirrors what
//! `vertex_core` does with its dedicated engine thread, minus the channel
//! bridge, because here there is nothing to bridge to.
//!
//! Usage:
//!   air_agent --id drone_0 --bind 127.0.0.1:47633 \
//!             --key <base58-secret> \
//!             --peer <base58-public>@127.0.0.1:47631 [--peer ...] \
//!             --link 127.0.0.1:48633 \
//!             [--conduct honest|false-clear|phantom-hazards] \
//!             [--log logs/drone_0_airground.log]

mod fold;
mod link;
mod mission;

use std::fs::{File, OpenOptions};
use std::io::Write as _;
use std::time::Duration;

use serde_json::{json, Value};
use tashi_vertex::{Context, Engine, KeyPublic, KeySecret, Message, Options, Peers, Socket,
                   Transaction};

use fold::{AirGroundState, make_blocks, make_sectors};
use link::{Command, Link, Telemetry};
use mission::{Conduct, Plan, health_record, nearest_claimable_block, sector_at, survey_records,
              READY_BATTERY, RTB_BATTERY, SURVEY_ALT};

/// Grid geometry. Mirrors `airground.launch.py` and `fixtures/conformance.json`;
/// every process in the fleet must key sectors identically or the shared state
/// means nothing.
const NX: usize = 4;
const NY: usize = 3;
const MIN_X: f64 = -20.0;
const MIN_Y: f64 = -15.0;
const CELL_W: f64 = 10.0;
const CELL_H: f64 = 10.0;
const BLOCK_W: usize = 2;
const BLOCK_H: usize = 1;

/// How often the agent beacons health and reconsiders what to claim.
const TICK: Duration = Duration::from_millis(500);
/// Beacons per tick is one; this many ticks between them keeps the log
/// readable without letting a silence lease fire on a healthy drone.
const BEACON_EVERY: u32 = 4;

struct Args {
    id: String,
    bind: String,
    key: KeySecret,
    peers: Vec<(String, KeyPublic)>,
    link: String,
    conduct: Conduct,
    log: Option<String>,
}

fn usage() -> ! {
    eprintln!(
        "usage: air_agent --id <name> --bind <addr:port> --key <b58secret> \\\n\
         \x20         --peer <b58public>@<addr:port> [--peer ...] --link <addr:port> \\\n\
         \x20         [--conduct honest|false-clear|phantom-hazards] [--log <path>]"
    );
    std::process::exit(2);
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let (mut id, mut bind, mut key, mut link, mut log) = (None, None, None, None, None);
    let mut conduct = Conduct::Honest;
    let mut peers = Vec::new();

    let mut i = 0;
    while i < argv.len() {
        let need = |i: usize| -> String {
            argv.get(i + 1).cloned().unwrap_or_else(|| usage())
        };
        match argv[i].as_str() {
            "--id" => id = Some(need(i)),
            "--bind" => bind = Some(need(i)),
            "--key" => key = Some(need(i)),
            "--link" => link = Some(need(i)),
            "--log" => log = Some(need(i)),
            "--conduct" => conduct = Conduct::parse(&need(i)).unwrap_or_else(|| usage()),
            "--peer" => {
                let spec = need(i);
                let Some((pub_b58, addr)) = spec.split_once('@') else { usage() };
                let public: KeyPublic = pub_b58.parse().unwrap_or_else(|e| {
                    eprintln!("air_agent: bad peer public key: {e:?}");
                    std::process::exit(2);
                });
                peers.push((addr.to_string(), public));
            }
            other => {
                eprintln!("air_agent: unknown argument {other}");
                usage();
            }
        }
        i += 2;
    }

    let key: KeySecret = key.unwrap_or_else(|| usage()).parse().unwrap_or_else(|e| {
        eprintln!("air_agent: bad secret key: {e:?}");
        std::process::exit(2);
    });

    Args {
        id: id.unwrap_or_else(|| usage()),
        bind: bind.unwrap_or_else(|| usage()),
        key,
        peers,
        link: link.unwrap_or_else(|| usage()),
        conduct,
        log,
    }
}

/// Per-node consensus log, the same EVENT/STATE/TX/DECIDE shape the two
/// existing simulations write, so `verify_consensus_logs.py` can diff a
/// drone's stream against a bot's and prove they saw one ordered history.
struct Journal(Option<File>);

impl Journal {
    fn open(path: Option<&str>) -> Self {
        Journal(path.and_then(|p| {
            if let Some(dir) = std::path::Path::new(p).parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            OpenOptions::new().create(true).append(true).open(p).ok()
        }))
    }

    fn line(&mut self, kind: &str, body: &str) {
        if let Some(f) = self.0.as_mut() {
            let _ = writeln!(f, "{kind} {body}");
            let _ = f.flush();
        }
    }
}

fn submit(engine: &Engine, journal: &mut Journal, rec: &Value) {
    // serde_json's default Map is a BTreeMap, so this is sorted-key compact
    // JSON: byte-identical to vertex_fleet.state.encode on the Python side.
    let bytes = rec.to_string().into_bytes();
    let mut tx = Transaction::allocate(bytes.len());
    tx.copy_from_slice(&bytes);
    match engine.send_transaction(tx) {
        Ok(()) => journal.line("TX", &rec.to_string()),
        Err(e) => eprintln!("air_agent: send_transaction failed: {e:?}"),
    }
}

/// Send a command if the airframe is attached. Before its controller
/// connects (or between a Webots reload) the drone still runs its consensus
/// loop; it just has nothing to steer.
async fn command(wire: &mut Option<Link>, cmd: Command) {
    if let Some(w) = wire.as_mut() {
        if let Err(e) = w.send(cmd).await {
            eprintln!("air_agent: link send failed: {e}");
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = parse_args();
    let me = args.id.clone();

    let (sectors, centers) = make_sectors(NX, NY, MIN_X, MIN_Y, CELL_W, CELL_H);
    let (blocks, block_cells) = make_blocks(NX, NY, BLOCK_W, BLOCK_H);
    let mut state = AirGroundState::new(sectors, blocks, block_cells.clone());
    let mut journal = Journal::open(args.log.as_deref());

    // --- join the committee -------------------------------------------------
    let mut peers = Peers::with_capacity(args.peers.len() + 1)?;
    for (addr, public) in &args.peers {
        peers.insert(addr, public, Default::default())?;
    }
    peers.insert(&args.bind, &args.key.public(), Default::default())?;

    let context = Context::new()?;
    let socket = Socket::bind(&context, &args.bind).await?;
    let mut options = Options::default();
    // Match the launch file's vertex_node setting so every peer in the mesh
    // gossips at the same cadence.
    options.set_heartbeat_us(50_000);
    let engine = Engine::start(&context, socket, options, &args.key, peers, false)?;
    eprintln!(
        "[{me}] joined the committee on {} with {} peers, conduct {:?}",
        args.bind,
        args.peers.len(),
        args.conduct
    );

    // --- listen for the airframe, but do not block on it --------------------
    // The agent takes part in consensus from the moment it starts, beaconing
    // not-ok until telemetry arrives, exactly as the ground coordinators do
    // before their robots connect. Blocking here instead would make a drone
    // whose controller has not started a silent committee member that never
    // even pumps recv_message, which with n = 4 is a fault the fleet should
    // not have to absorb for a merely-late Webots.
    let listener = Link::listen(&args.link).await?;
    let mut wire: Option<Link> = None;
    eprintln!("[{me}] listening for its Webots controller on {}", args.link);

    // --- state the agent owns locally, never shared except as records -------
    let mut latest: Option<Telemetry> = None;
    let mut plan: Option<Plan> = None;
    let mut beacon_seq: i64 = 0;
    let mut ticks: u32 = 0;
    let mut grounded_locally = false;
    let mut ticker = tokio::time::interval(TICK);

    loop {
        tokio::select! {
            // ---- consensus: the only thing that mutates shared state ----
            msg = engine.recv_message() => {
                match msg {
                    Ok(Some(Message::Event(ev))) => {
                        journal.line("EVENT", &format!("{} txs={}",
                            hex32(ev.hash()), ev.transaction_count()));
                        for payload in ev.transactions() {
                            let Ok(text) = std::str::from_utf8(payload) else { continue };
                            let Ok(rec) = serde_json::from_str::<Value>(text) else { continue };
                            state.apply(&rec);
                        }
                        journal.line("STATE", &state.snapshot().to_string());
                    }
                    Ok(Some(Message::SyncPoint(_))) => {}
                    Ok(None) => { eprintln!("[{me}] event stream closed"); break }
                    Err(e) => { eprintln!("[{me}] recv_message: {e:?}"); break }
                }
            }

            // ---- the airframe connecting (or reconnecting after a reload) ----
            accepted = listener.accept(), if wire.is_none() => {
                match accepted {
                    Ok((stream, _)) => {
                        eprintln!("[{me}] controller connected");
                        wire = Some(Link::wrap(stream));
                    }
                    Err(e) => eprintln!("[{me}] accept failed: {e}"),
                }
            }

            // ---- telemetry: local truth, feeds proposals ----
            t = async { wire.as_mut().unwrap().recv().await }, if wire.is_some() => {
                match t {
                    Some(t) => {
                        latest = Some(t);
                        // Sample the ground under the current waypoint and step
                        // the pass along if we have arrived.
                        if let Some(p) = plan.as_mut() {
                            let cell = sector_at(t.x, t.y, NX, NY, MIN_X, MIN_Y, CELL_W, CELL_H);
                            p.advance(&t, cell.as_deref());
                        }
                    }
                    None => {
                        // Webots closed or reloaded. Drop the stale telemetry
                        // so health goes not-ok and the fold releases our
                        // block, then wait for the controller to come back.
                        eprintln!("[{me}] controller disconnected, waiting for it to return");
                        wire = None;
                        latest = None;
                        plan = None;
                    }
                }
            }

            // ---- tick: beacon, then decide what to propose ----
            _ = ticker.tick() => {
                ticks += 1;

                if ticks % BEACON_EVERY == 0 {
                    beacon_seq += 1;
                    let rec = health_record(&me, beacon_seq, latest.as_ref(), state.epoch);
                    submit(&engine, &mut journal, &rec);
                }

                let Some(t) = latest else { continue };

                // Battery lease. Physically motivated, but the same lease
                // shape the arena scenario uses for silence: hand the work
                // back through consensus, do not just stop flying.
                if !grounded_locally && t.battery < RTB_BATTERY {
                    grounded_locally = true;
                    plan = None;
                    submit(&engine, &mut journal, &json!({
                        "op": "rtb", "agent": me, "epoch": state.epoch}));
                    journal.line("DECIDE", "rtb: battery low, releasing block");
                    command(&mut wire, Command::Land).await;
                    continue;
                }
                if grounded_locally {
                    if t.battery >= READY_BATTERY {
                        grounded_locally = false;
                        submit(&engine, &mut journal, &json!({
                            "op": "ready", "agent": me, "epoch": state.epoch}));
                        journal.line("DECIDE", "ready: recharged, resuming survey");
                    }
                    continue;
                }

                // Consensus decides whether we still hold the block: if a
                // health blip or an rtb released it, the local plan is void.
                let held = state.my_block(&me);
                if held.as_deref() != plan.as_ref().map(|p| p.block.as_str()) {
                    plan = held.as_ref().map(|b| {
                        journal.line("DECIDE", &format!("planning survey of {b}"));
                        Plan::new(b, block_cells.get(b).map_or(&[], |v| v), &centers)
                    });
                }

                match plan.as_ref() {
                    // Holding a block and still flying it.
                    Some(p) if !p.finished() => {
                        if let Some((x, y)) = p.current() {
                            command(&mut wire, Command::Goto { x, y, z: SURVEY_ALT }).await;
                        }
                    }
                    // Pass complete: report what the ranger saw, then the survey.
                    Some(p) => {
                        for rec in survey_records(&me, p, &block_cells, args.conduct, state.epoch) {
                            submit(&engine, &mut journal, &rec);
                        }
                        journal.line("DECIDE", &format!(
                            "surveyed {}, {} sighting(s)", p.block, p.sighted.len()));
                        plan = None;
                        command(&mut wire, Command::Hold).await;
                    }
                    // Idle: try to take work.
                    None => {
                        match nearest_claimable_block(&state, &t, &block_cells, &centers) {
                            Some(b) => submit(&engine, &mut journal, &json!({
                                "op": "survey_claim", "agent": me,
                                "block": b, "epoch": state.epoch})),
                            None => { command(&mut wire, Command::Hold).await; }
                        }
                    }
                }
            }
        }
    }

    Ok(())
}

fn hex32(h: &[u8; 32]) -> String {
    h.iter().map(|b| format!("{b:02x}")).collect()
}

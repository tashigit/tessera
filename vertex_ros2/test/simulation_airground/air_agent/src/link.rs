//! The drone's link to its Webots controller: newline-delimited JSON over TCP.
//!
//! Deliberately NOT rosbridge. The ground tier reaches its Pioneers through
//! rosbridge because those robots are already ROS citizens, but the air tier
//! shares nothing with ROS: no rclrs, no DDS, no message packages, not even a
//! WebSocket. A drone needs exactly two things to take part in the fleet, the
//! Vertex wire protocol and the record schema, and keeping this link as dumb
//! as a socket is what makes that visible.
//!
//! The controller (`controllers/mavic_surveyor/`) runs natively on the host
//! inside Webots and connects in; the agent listens.
//!
//! Controller -> agent, one object per line:
//!   {"t":"telemetry","x":1.0,"y":2.0,"z":12.0,"clearance":12.1,
//!    "battery":0.87,"age":0.05}
//!
//! Agent -> controller, one object per line:
//!   {"t":"goto","x":1.0,"y":2.0,"z":12.0}
//!   {"t":"hold"}
//!   {"t":"land"}

use std::io;

use serde_json::Value;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};

/// One telemetry sample from the airframe.
#[derive(Debug, Clone, Copy)]
pub struct Telemetry {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Downward range to the ground. Over flat arena floor this is roughly the
    /// altitude; over a pit it reads materially longer, which is the whole
    /// reason the air tier exists. A horizontal lidar cannot see a hole.
    pub clearance: f64,
    /// Remaining flight time, 1.0 full to 0.0 empty. Drives the `rtb` lease.
    pub battery: f64,
    /// Seconds since the controller last refreshed its sensors. Feeds the
    /// health beacon exactly as the ground tier's stream ages do.
    pub age: f64,
}

impl Telemetry {
    fn parse(line: &str) -> Option<Self> {
        let v: Value = serde_json::from_str(line).ok()?;
        if v.get("t").and_then(Value::as_str)? != "telemetry" {
            return None;
        }
        let f = |k: &str| v.get(k).and_then(Value::as_f64);
        Some(Telemetry {
            x: f("x")?,
            y: f("y")?,
            z: f("z")?,
            clearance: f("clearance").unwrap_or(f64::NAN),
            battery: f("battery").unwrap_or(1.0),
            age: f("age").unwrap_or(0.0),
        })
    }
}

/// What the agent tells the airframe to do.
pub enum Command {
    Goto { x: f64, y: f64, z: f64 },
    Hold,
    Land,
}

impl Command {
    fn encode(&self) -> String {
        match *self {
            Command::Goto { x, y, z } => {
                format!("{{\"t\":\"goto\",\"x\":{x},\"y\":{y},\"z\":{z}}}\n")
            }
            Command::Hold => "{\"t\":\"hold\"}\n".to_string(),
            Command::Land => "{\"t\":\"land\"}\n".to_string(),
        }
    }
}

pub struct Link {
    reader: BufReader<OwnedReadHalf>,
    writer: OwnedWriteHalf,
    line: String,
}

impl Link {
    /// Start listening. The agent keeps the listener for the whole run rather
    /// than accepting once and dropping it: a Webots world reload restarts the
    /// controller, and the drone should pick it back up instead of dying.
    pub async fn listen(bind: &str) -> io::Result<TcpListener> {
        TcpListener::bind(bind).await
    }

    pub fn wrap(stream: TcpStream) -> Self {
        // Telemetry is small and latency matters more than packing, so send
        // each line immediately instead of waiting for Nagle.
        let _ = stream.set_nodelay(true);
        let (r, w) = stream.into_split();
        Self {
            reader: BufReader::new(r),
            writer: w,
            line: String::new(),
        }
    }

    /// Next telemetry sample, or `None` once the controller disconnects
    /// (Webots closed, or the world was reloaded).
    ///
    /// Cancel-safe enough for `select!`: a line half-read when the future is
    /// dropped is lost, which for a telemetry stream that resends at a fixed
    /// rate costs one sample and nothing else.
    pub async fn recv(&mut self) -> Option<Telemetry> {
        loop {
            self.line.clear();
            match self.reader.read_line(&mut self.line).await {
                Ok(0) => return None,
                Err(e) => {
                    eprintln!("link: read failed: {e}");
                    return None;
                }
                Ok(_) => {
                    if let Some(t) = Telemetry::parse(self.line.trim()) {
                        return Some(t);
                    }
                    // Anything unparseable is skipped rather than fatal: the
                    // controller is a separate process on the host and may be
                    // mid-restart.
                }
            }
        }
    }

    pub async fn send(&mut self, cmd: Command) -> io::Result<()> {
        self.writer.write_all(cmd.encode().as_bytes()).await
    }
}

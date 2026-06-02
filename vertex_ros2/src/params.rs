//! Declare the ROS 2 parameters (design §6, §4.6) and assemble a
//! [`vertex_core::Config`].
//!
//! NOTE: this module is part of the ROS binding layer and is built by colcon
//! against `rclrs` (Jazzy baseline). The parameter *names, types, defaults, and
//! the mapping onto Vertex* are the contract; the `rclrs` parameter-declaration
//! calls follow the ros2-rust API for the pinned distribution.

use std::sync::Arc;

use rclrs::Node;
use vertex_core::config::{BridgeConfig, Config, ConfigError, PeerConfig, VertexOptions};

/// Errors assembling a [`Config`] from parameters.
#[derive(Debug)]
pub enum ParamError {
    Config(ConfigError),
    Missing(&'static str),
    Io(String),
    Range(String),
}

impl std::fmt::Display for ParamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParamError::Config(e) => write!(f, "{e}"),
            ParamError::Missing(p) => write!(f, "required parameter `{p}` is not set"),
            ParamError::Io(e) => write!(f, "{e}"),
            ParamError::Range(e) => write!(f, "{e}"),
        }
    }
}
impl std::error::Error for ParamError {}

impl From<ConfigError> for ParamError {
    fn from(e: ConfigError) -> Self {
        ParamError::Config(e)
    }
}

fn to_usize(name: &str, v: i64) -> Result<usize, ParamError> {
    usize::try_from(v).map_err(|_| ParamError::Range(format!("`{name}` must be >= 0 (got {v})")))
}
fn to_u64(name: &str, v: i64) -> Result<u64, ParamError> {
    u64::try_from(v).map_err(|_| ParamError::Range(format!("`{name}` must be >= 0 (got {v})")))
}
fn to_u32(name: &str, v: i64) -> Result<u32, ParamError> {
    u32::try_from(v).map_err(|_| ParamError::Range(format!("`{name}` out of u32 range (got {v})")))
}
fn to_u16(name: &str, v: i64) -> Result<u16, ParamError> {
    u16::try_from(v).map_err(|_| ParamError::Range(format!("`{name}` out of u16 range (got {v})")))
}

/// Declare every parameter on `node` and build the resolved [`Config`].
///
/// All `options.*` parameters default to the documented Vertex defaults
/// ([`VertexOptions::default`]); leaving one unset preserves engine behavior.
pub fn build_config(node: &Node) -> Result<Config, ParamError> {
    // --- identity / network ---
    let bind_address: Arc<str> = node
        .declare_parameter("vertex.bind_address")
        .mandatory()
        .map_err(|_| ParamError::Missing("vertex.bind_address"))?
        .get();

    let secret_key_base58: Arc<str> = node
        .declare_parameter("vertex.secret_key_base58")
        .default(Arc::from(""))
        .mandatory()
        .map_err(|_| ParamError::Missing("vertex.secret_key_base58"))?
        .get();

    let secret_key_path: Arc<str> = node
        .declare_parameter("vertex.secret_key_path")
        .default(Arc::from(""))
        .mandatory()
        .map_err(|_| ParamError::Missing("vertex.secret_key_path"))?
        .get();

    // Prefer the file path (recommended production form — keeps the key out of
    // the parameter store; design §5.1). Fall back to the inline base58 value.
    let key = if !secret_key_path.is_empty() {
        let contents = std::fs::read_to_string(&*secret_key_path)
            .map_err(|e| ParamError::Io(format!("reading {secret_key_path}: {e}")))?;
        Config::secret_from_base58(contents.trim())?
    } else if !secret_key_base58.is_empty() {
        Config::secret_from_base58(&secret_key_base58)?
    } else {
        return Err(ParamError::Missing(
            "vertex.secret_key_path or vertex.secret_key_base58",
        ));
    };

    let peer_specs: Arc<[Arc<str>]> = node
        .declare_parameter("vertex.peers")
        .default(Arc::from([]))
        .mandatory()
        .map_err(|_| ParamError::Missing("vertex.peers"))?
        .get();
    let peers: Vec<PeerConfig> =
        Config::peers_from_specs(peer_specs.iter().map(|s| s.to_string()))?;

    let joining_running_session: bool = node
        .declare_parameter("vertex.joining_running_session")
        .default(false)
        .mandatory()
        .map_err(|_| ParamError::Missing("vertex.joining_running_session"))?
        .get();

    // --- bridge channel bounds (§4.6) ---
    let defaults = BridgeConfig::default();
    let tx_cap = node
        .declare_parameter("bridge.tx_channel_capacity")
        .default(defaults.tx_channel_capacity as i64)
        .mandatory()
        .map_err(|_| ParamError::Missing("bridge.tx_channel_capacity"))?
        .get();
    let event_cap = node
        .declare_parameter("bridge.event_channel_capacity")
        .default(defaults.event_channel_capacity as i64)
        .mandatory()
        .map_err(|_| ParamError::Missing("bridge.event_channel_capacity"))?
        .get();
    let bridge = BridgeConfig {
        tx_channel_capacity: to_usize("bridge.tx_channel_capacity", tx_cap)?,
        event_channel_capacity: to_usize("bridge.event_channel_capacity", event_cap)?,
    };

    // --- Vertex engine options (§6) ---
    let options = build_options(node)?;

    Ok(Config {
        bind_address: bind_address.to_string(),
        key,
        peers,
        joining_running_session,
        bridge,
        options,
    })
}

fn build_options(node: &Node) -> Result<VertexOptions, ParamError> {
    let d = VertexOptions::default();

    // base_min_event_interval_us is optional (Option<u64>); -1 means "unset".
    let base_min = node
        .declare_parameter("options.base_min_event_interval_us")
        .default(-1)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.base_min_event_interval_us"))?
        .get();
    let base_min_event_interval_us = if base_min < 0 {
        None
    } else {
        Some(to_u64("options.base_min_event_interval_us", base_min)?)
    };

    let report_gossip_events = node
        .declare_parameter("options.report_gossip_events")
        .default(d.report_gossip_events)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.report_gossip_events"))?
        .get();

    let fallen_behind_kick_s = node
        .declare_parameter("options.fallen_behind_kick_s")
        .default(d.fallen_behind_kick_s)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.fallen_behind_kick_s"))?
        .get();

    let heartbeat_us = to_u64(
        "options.heartbeat_us",
        node.declare_parameter("options.heartbeat_us")
            .default(d.heartbeat_us as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.heartbeat_us"))?
            .get(),
    )?;

    let target_ack_latency_ms = to_u32(
        "options.target_ack_latency_ms",
        node.declare_parameter("options.target_ack_latency_ms")
            .default(d.target_ack_latency_ms as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.target_ack_latency_ms"))?
            .get(),
    )?;
    let max_ack_latency_ms = to_u32(
        "options.max_ack_latency_ms",
        node.declare_parameter("options.max_ack_latency_ms")
            .default(d.max_ack_latency_ms as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.max_ack_latency_ms"))?
            .get(),
    )?;
    let throttle_ack_latency_ms = to_u32(
        "options.throttle_ack_latency_ms",
        node.declare_parameter("options.throttle_ack_latency_ms")
            .default(d.throttle_ack_latency_ms as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.throttle_ack_latency_ms"))?
            .get(),
    )?;
    let reset_ack_latency_ms = to_u32(
        "options.reset_ack_latency_ms",
        node.declare_parameter("options.reset_ack_latency_ms")
            .default(d.reset_ack_latency_ms as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.reset_ack_latency_ms"))?
            .get(),
    )?;

    let enable_dynamic_epoch_size = node
        .declare_parameter("options.enable_dynamic_epoch_size")
        .default(d.enable_dynamic_epoch_size)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.enable_dynamic_epoch_size"))?
        .get();

    let transaction_channel_size = to_usize(
        "options.transaction_channel_size",
        node.declare_parameter("options.transaction_channel_size")
            .default(d.transaction_channel_size as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.transaction_channel_size"))?
            .get(),
    )?;

    let max_unacknowledged_bytes = to_usize(
        "options.max_unacknowledged_bytes",
        node.declare_parameter("options.max_unacknowledged_bytes")
            .default(d.max_unacknowledged_bytes as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.max_unacknowledged_bytes"))?
            .get(),
    )?;

    // -1 means "unset" → engine default (number of CPU cores).
    let mbvt = node
        .declare_parameter("options.max_blocking_verify_threads")
        .default(-1)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.max_blocking_verify_threads"))?
        .get();
    let max_blocking_verify_threads = if mbvt < 0 {
        None
    } else {
        Some(to_usize("options.max_blocking_verify_threads", mbvt)?)
    };

    let enable_state_sharing = node
        .declare_parameter("options.enable_state_sharing")
        .default(d.enable_state_sharing)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.enable_state_sharing"))?
        .get();

    let epoch_states_to_cache = to_u16(
        "options.epoch_states_to_cache",
        node.declare_parameter("options.epoch_states_to_cache")
            .default(d.epoch_states_to_cache as i64)
            .mandatory()
            .map_err(|_| ParamError::Missing("options.epoch_states_to_cache"))?
            .get(),
    )?;

    let enable_hole_punching = node
        .declare_parameter("options.enable_hole_punching")
        .default(d.enable_hole_punching)
        .mandatory()
        .map_err(|_| ParamError::Missing("options.enable_hole_punching"))?
        .get();

    Ok(VertexOptions {
        base_min_event_interval_us,
        report_gossip_events,
        fallen_behind_kick_s,
        heartbeat_us,
        target_ack_latency_ms,
        max_ack_latency_ms,
        throttle_ack_latency_ms,
        reset_ack_latency_ms,
        enable_dynamic_epoch_size,
        transaction_channel_size,
        max_unacknowledged_bytes,
        max_blocking_verify_threads,
        enable_state_sharing,
        epoch_states_to_cache,
        enable_hole_punching,
    })
}

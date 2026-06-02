//! Typed configuration, parsed from ROS parameters (design §4.6, §6) but with no
//! dependency on rclrs so it is testable here. The `ros` module fills a
//! [`Config`] from declared parameters; everything below validates and maps it
//! onto `tashi-vertex` types.

use std::str::FromStr;

use tashi_vertex::{KeyPublic, KeySecret, Options, PeerCapabilities, Peers};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("invalid secret key: {0}")]
    SecretKey(String),
    #[error("invalid peer spec {spec:?}: {reason}")]
    Peer { spec: String, reason: String },
    #[error("failed to build Vertex peers: {0}")]
    Peers(String),
}

/// Plain mirror of every tunable `tashi_vertex::Options` setter (design §6).
/// Defaults match the documented Vertex defaults so an unset ROS parameter
/// leaves engine behavior unchanged.
#[derive(Debug, Clone)]
pub struct VertexOptions {
    pub base_min_event_interval_us: Option<u64>,
    pub report_gossip_events: bool,
    pub fallen_behind_kick_s: i64,
    pub heartbeat_us: u64,
    pub target_ack_latency_ms: u32,
    pub max_ack_latency_ms: u32,
    pub throttle_ack_latency_ms: u32,
    pub reset_ack_latency_ms: u32,
    pub enable_dynamic_epoch_size: bool,
    pub transaction_channel_size: usize,
    pub max_unacknowledged_bytes: usize,
    pub max_blocking_verify_threads: Option<usize>,
    pub enable_state_sharing: bool,
    pub epoch_states_to_cache: u16,
    pub enable_hole_punching: bool,
}

impl Default for VertexOptions {
    fn default() -> Self {
        // Mirrors tashi_vertex::Options defaults as documented in options.rs.
        VertexOptions {
            base_min_event_interval_us: None,
            report_gossip_events: false,
            fallen_behind_kick_s: 10,
            heartbeat_us: 500_000,
            target_ack_latency_ms: 400,
            max_ack_latency_ms: 600,
            throttle_ack_latency_ms: 900,
            reset_ack_latency_ms: 2000,
            enable_dynamic_epoch_size: true,
            transaction_channel_size: 32,
            max_unacknowledged_bytes: 500 * 1024 * 1024,
            max_blocking_verify_threads: None,
            enable_state_sharing: false,
            epoch_states_to_cache: 3,
            enable_hole_punching: true,
        }
    }
}

impl VertexOptions {
    /// Apply every field onto a fresh `tashi_vertex::Options`.
    pub fn build(&self) -> Options {
        let mut o = Options::default();
        if let Some(v) = self.base_min_event_interval_us {
            o.set_base_min_event_interval_us(v);
        }
        o.set_report_gossip_events(self.report_gossip_events);
        o.set_fallen_behind_kick_s(self.fallen_behind_kick_s);
        o.set_heartbeat_us(self.heartbeat_us);
        o.set_target_ack_latency_ms(self.target_ack_latency_ms);
        o.set_max_ack_latency_ms(self.max_ack_latency_ms);
        o.set_throttle_ack_latency_ms(self.throttle_ack_latency_ms);
        o.set_reset_ack_latency_ms(self.reset_ack_latency_ms);
        o.set_enable_dynamic_epoch_size(self.enable_dynamic_epoch_size);
        o.set_transaction_channel_size(self.transaction_channel_size);
        o.set_max_unacknowledged_bytes(self.max_unacknowledged_bytes);
        if let Some(v) = self.max_blocking_verify_threads {
            o.set_max_blocking_verify_threads(v);
        }
        o.set_enable_state_sharing(self.enable_state_sharing);
        o.set_epoch_states_to_cache(self.epoch_states_to_cache);
        o.set_enable_hole_punching(self.enable_hole_punching);
        o
    }
}

/// One remote peer: a literal `IP:port` (no DNS) and its public key.
///
/// v0.1 inserts every peer with default [`PeerCapabilities`] (design Appendix A);
/// per-peer capabilities are not exposed as ROS parameters yet (non-goal N1/N3).
#[derive(Debug, Clone)]
pub struct PeerConfig {
    pub address: String,
    pub public: KeyPublic,
}

impl PeerConfig {
    /// Parse a `"<base58_pubkey>@<ip:port>"` spec (the form used by the
    /// `tashi-vertex` `pingback` example and accepted on the `vertex.peers`
    /// ROS parameter).
    pub fn parse(spec: &str) -> Result<Self, ConfigError> {
        let (public, address) = spec.split_once('@').ok_or_else(|| ConfigError::Peer {
            spec: spec.to_string(),
            reason: "expected <public_key>@<ip:port>".into(),
        })?;
        let public = KeyPublic::from_str(public).map_err(|e| ConfigError::Peer {
            spec: spec.to_string(),
            reason: format!("bad public key: {e}"),
        })?;
        Ok(PeerConfig {
            address: address.to_string(),
            public,
        })
    }
}

/// Bounds on the two bridge channels (design §4.6).
#[derive(Debug, Clone, Copy)]
pub struct BridgeConfig {
    pub tx_channel_capacity: usize,
    pub event_channel_capacity: usize,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        BridgeConfig {
            tx_channel_capacity: 1024,
            event_channel_capacity: 4096,
        }
    }
}

/// The fully-resolved node configuration.
pub struct Config {
    pub bind_address: String,
    pub key: KeySecret,
    pub peers: Vec<PeerConfig>,
    /// Last arg to `Engine::start`; set true to rejoin an in-flight session.
    pub joining_running_session: bool,
    pub bridge: BridgeConfig,
    pub options: VertexOptions,
}

impl Config {
    /// Parse a secret key from either base58 (the `Display`/`FromStr` form) or,
    /// if that fails, raw DER bytes are accepted via [`Config::secret_from_der`].
    pub fn secret_from_base58(s: &str) -> Result<KeySecret, ConfigError> {
        KeySecret::from_str(s).map_err(|e| ConfigError::SecretKey(e.to_string()))
    }

    pub fn secret_from_der(der: &[u8]) -> Result<KeySecret, ConfigError> {
        KeySecret::from_der(der).map_err(|e| ConfigError::SecretKey(e.to_string()))
    }

    /// Parse the `vertex.peers` string list into [`PeerConfig`]s.
    pub fn peers_from_specs<I, S>(specs: I) -> Result<Vec<PeerConfig>, ConfigError>
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        specs
            .into_iter()
            .map(|s| PeerConfig::parse(s.as_ref()))
            .collect()
    }

    /// Total peer count including ourself (we always insert ourself).
    pub fn peer_count(&self) -> u32 {
        (self.peers.len() + 1) as u32
    }

    /// Build the `tashi_vertex::Peers` set: every configured remote peer plus
    /// ourself (design Appendix A).
    pub fn build_peers(&self) -> Result<Peers, ConfigError> {
        let mut peers = Peers::with_capacity(self.peers.len() + 1)
            .map_err(|e| ConfigError::Peers(e.to_string()))?;
        for p in &self.peers {
            peers
                .insert(&p.address, &p.public, PeerCapabilities::default())
                .map_err(|e| ConfigError::Peers(format!("{}: {e}", p.address)))?;
        }
        peers
            .insert(&self.bind_address, &self.key.public(), PeerCapabilities::default())
            .map_err(|e| ConfigError::Peers(format!("self {}: {e}", self.bind_address)))?;
        Ok(peers)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_pubkey_b58() -> String {
        // Derive a real public key so the parser exercises the actual decoder.
        KeySecret::generate().public().to_string()
    }

    #[test]
    fn peer_spec_parses() {
        let spec = format!("{}@127.0.0.1:9001", sample_pubkey_b58());
        let p = PeerConfig::parse(&spec).expect("parse");
        assert_eq!(p.address, "127.0.0.1:9001");
    }

    #[test]
    fn peer_spec_missing_at_is_rejected() {
        let err = PeerConfig::parse("no-at-sign").unwrap_err();
        assert!(matches!(err, ConfigError::Peer { .. }));
    }

    #[test]
    fn peer_spec_bad_key_is_rejected() {
        let err = PeerConfig::parse("not-a-key@127.0.0.1:9001").unwrap_err();
        assert!(matches!(err, ConfigError::Peer { .. }));
    }

    #[test]
    fn default_options_match_documented_vertex_defaults() {
        let o = VertexOptions::default();
        assert_eq!(o.heartbeat_us, 500_000);
        assert_eq!(o.target_ack_latency_ms, 400);
        assert_eq!(o.transaction_channel_size, 32);
        assert!(o.enable_dynamic_epoch_size);
        assert!(o.enable_hole_punching);
    }

    #[test]
    fn options_build_does_not_panic() {
        // Exercises every FFI setter against the real engine options object.
        let _ = VertexOptions::default().build();
    }

    #[test]
    fn round_trip_secret_base58() {
        let key = KeySecret::generate();
        let s = key.to_string();
        let parsed = Config::secret_from_base58(&s).expect("parse");
        assert_eq!(parsed.public().to_string(), key.public().to_string());
    }
}

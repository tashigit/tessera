//! The Rust twin of `nodes/airground_fsm.py`.
//!
//! This is the whole point of simulation 3. A tessera bot derives shared state
//! by folding `/vertex/event` through `AirGroundState` in Python; a drone
//! derives the same state by folding the same log through `AirGroundState`
//! here, having linked `tashi-vertex` directly with no ROS in the process.
//! If the two folds ever disagree the simulation proves nothing, and they
//! would disagree silently.
//!
//! So the two are pinned together by `fixtures/conformance.json`: a scripted
//! log plus the canonical snapshot after every record, replayed by
//! `test_airground_fsm.py` on one side and by the tests at the bottom of this
//! file on the other.
//!
//! Rules live in `../../README.md`. Keep every branch here pure and
//! order-dependent only on the log. Any change must be mirrored in the Python
//! fold and the fixture regenerated.

use std::collections::{BTreeMap, BTreeSet};

use serde_json::Value;

pub const SURVEYING: &str = "surveying";
pub const DONE: &str = "done";

/// Ground sector ids and centres, row-major from the south-west corner.
/// Mirrors `make_sectors` in the Python fold.
pub fn make_sectors(
    nx: usize,
    ny: usize,
    min_x: f64,
    min_y: f64,
    cell_w: f64,
    cell_h: f64,
) -> (Vec<String>, BTreeMap<String, (f64, f64)>) {
    let mut ids = Vec::new();
    let mut centers = BTreeMap::new();
    for iy in 0..ny {
        for ix in 0..nx {
            let sid = format!("S{:02}", iy * nx + ix);
            centers.insert(
                sid.clone(),
                (
                    min_x + (ix as f64 + 0.5) * cell_w,
                    min_y + (iy as f64 + 0.5) * cell_h,
                ),
            );
            ids.push(sid);
        }
    }
    (ids, centers)
}

/// Air survey blocks laid over the sector grid. Mirrors `make_blocks`.
pub fn make_blocks(
    nx: usize,
    ny: usize,
    block_w: usize,
    block_h: usize,
) -> (Vec<String>, BTreeMap<String, Vec<String>>) {
    let mut ids = Vec::new();
    let mut cells = BTreeMap::new();
    let bx = nx.div_ceil(block_w);
    let by = ny.div_ceil(block_h);
    for iy in 0..by {
        for ix in 0..bx {
            let bid = format!("B{:02}", iy * bx + ix);
            let mut members = Vec::new();
            for dy in 0..block_h {
                for dx in 0..block_w {
                    let (sx, sy) = (ix * block_w + dx, iy * block_h + dy);
                    if sx < nx && sy < ny {
                        members.push(format!("S{:02}", sy * nx + sx));
                    }
                }
            }
            cells.insert(bid.clone(), members);
            ids.push(bid);
        }
    }
    (ids, cells)
}

/// Deterministic fold of the consensus-ordered record stream.
///
/// Every collection is a `BTreeMap`/`BTreeSet` rather than a hash container:
/// iteration order is part of the cross-language contract, and a `HashMap`
/// would make the snapshot depend on Rust's random hash seed.
pub struct AirGroundState {
    sectors: BTreeSet<String>,
    blocks: BTreeSet<String>,
    block_cells: BTreeMap<String, Vec<String>>,

    pub epoch: i64,
    // air tier
    pub block_claims: BTreeMap<String, String>,
    pub surveyed_blocks: BTreeSet<String>,
    pub surveyed: BTreeSet<String>,
    pub grounded: BTreeSet<String>,
    // ground tier
    pub claimed: BTreeMap<String, String>,
    pub explored: BTreeSet<String>,
    pub explored_by: BTreeMap<String, String>,
    pub unreachable: BTreeSet<String>,
    // evidence
    pub hazard_reports: BTreeMap<String, BTreeSet<String>>,
    pub confirmed_hazards: BTreeSet<String>,
    // health, uniform across tiers
    pub unhealthy: BTreeSet<String>,
    pub health_seq: BTreeMap<String, i64>,
    pub phase: String,
}

/// Borrow a record field as a `&str`, or `None` if absent or not a string.
fn s<'a>(rec: &'a Value, key: &str) -> Option<&'a str> {
    rec.get(key).and_then(Value::as_str)
}

impl AirGroundState {
    pub fn new(
        sectors: Vec<String>,
        blocks: Vec<String>,
        block_cells: BTreeMap<String, Vec<String>>,
    ) -> Self {
        let mut st = Self {
            sectors: sectors.into_iter().collect(),
            blocks: blocks.into_iter().collect(),
            block_cells,
            epoch: 0,
            block_claims: BTreeMap::new(),
            surveyed_blocks: BTreeSet::new(),
            surveyed: BTreeSet::new(),
            grounded: BTreeSet::new(),
            claimed: BTreeMap::new(),
            explored: BTreeSet::new(),
            explored_by: BTreeMap::new(),
            unreachable: BTreeSet::new(),
            hazard_reports: BTreeMap::new(),
            confirmed_hazards: BTreeSet::new(),
            unhealthy: BTreeSet::new(),
            health_seq: BTreeMap::new(),
            phase: SURVEYING.to_string(),
        };
        st.wipe();
        st
    }

    fn wipe(&mut self) {
        self.block_claims.clear();
        self.surveyed_blocks.clear();
        self.surveyed.clear();
        self.grounded.clear();
        self.claimed.clear();
        self.explored.clear();
        self.explored_by.clear();
        self.unreachable.clear();
        self.hazard_reports.clear();
        self.confirmed_hazards.clear();
        self.unhealthy.clear();
        self.health_seq.clear();
        self.phase = SURVEYING.to_string();
    }

    /// The epoch gate and `reset`, mirroring `ReplicatedState.apply`.
    pub fn apply(&mut self, rec: &Value) {
        let Some(op) = s(rec, "op") else { return };
        if op == "reset" {
            let epoch = rec.get("epoch").and_then(Value::as_i64).unwrap_or(0);
            if epoch > self.epoch {
                self.wipe();
                self.epoch = epoch;
            }
            return;
        }
        if rec.get("epoch").and_then(Value::as_i64).unwrap_or(0) != self.epoch {
            return; // stale record from a previous epoch
        }
        self.apply_record(op, rec);
        self.recompute_phase();
    }

    fn apply_record(&mut self, op: &str, rec: &Value) {
        let agent = s(rec, "agent").map(str::to_string);
        match op {
            "survey_claim" => self.survey_claim(agent, s(rec, "block")),
            "surveyed" => self.surveyed_op(agent, s(rec, "block"), rec.get("cells")),
            // `hazard` and `corroborate` fold identically: what matters is how
            // many DISTINCT agents vouched, not which op carried the claim.
            "hazard" | "corroborate" => self.hazard(agent, s(rec, "cell")),
            "claim" => self.claim(agent, s(rec, "sector")),
            "explored" => self.explored_op(agent, s(rec, "sector")),
            "abandon" => {
                if let (Some(a), Some(sec)) = (agent, s(rec, "sector")) {
                    if self.claimed.get(sec) == Some(&a) {
                        self.claimed.remove(sec);
                    }
                }
            }
            "rtb" => self.rtb(agent),
            "ready" => {
                if let Some(a) = agent {
                    self.grounded.remove(&a);
                }
            }
            "health" => self.health(
                agent,
                rec.get("seq").and_then(Value::as_i64),
                rec.get("ok").and_then(Value::as_bool).unwrap_or(false),
            ),
            "suspect" => self.suspect(
                s(rec, "victim"),
                rec.get("seen_seq").and_then(Value::as_i64),
            ),
            _ => {}
        }
    }

    // ---- air tier ----
    fn survey_claim(&mut self, agent: Option<String>, block: Option<&str>) {
        let (Some(a), Some(b)) = (agent, block) else { return };
        let ok = self.phase == SURVEYING
            && !self.unhealthy.contains(&a)
            && !self.grounded.contains(&a)
            && self.blocks.contains(b)
            && !self.surveyed_blocks.contains(b)
            && !self.block_claims.contains_key(b)
            && !self.block_claims.values().any(|h| h == &a);
        if ok {
            self.block_claims.insert(b.to_string(), a);
        }
    }

    fn surveyed_op(&mut self, agent: Option<String>, block: Option<&str>, cells: Option<&Value>) {
        let (Some(a), Some(b)) = (agent, block) else { return };
        // Only the block's holder may report it surveyed.
        if self.block_claims.get(b) != Some(&a) {
            return;
        }
        self.surveyed_blocks.insert(b.to_string());
        self.block_claims.remove(b);
        // Clipped to the block actually held: a lying drone cannot clear the
        // whole map with one record.
        let allowed: BTreeSet<&String> = self
            .block_cells
            .get(b)
            .map(|v| v.iter().collect())
            .unwrap_or_default();
        match cells.and_then(Value::as_array) {
            Some(named) => {
                for c in named.iter().filter_map(Value::as_str) {
                    if allowed.contains(&c.to_string()) {
                        self.surveyed.insert(c.to_string());
                    }
                }
            }
            None => {
                for c in allowed {
                    self.surveyed.insert(c.clone());
                }
            }
        }
    }

    fn rtb(&mut self, agent: Option<String>) {
        let Some(a) = agent else { return };
        self.grounded.insert(a.clone());
        self.block_claims.retain(|_, h| h != &a);
    }

    // ---- evidence ----
    /// One witness is provisional, two distinct witnesses confirm.
    fn hazard(&mut self, agent: Option<String>, cell: Option<&str>) {
        let (Some(a), Some(c)) = (agent, cell) else { return };
        if self.unhealthy.contains(&a) || !self.sectors.contains(c) {
            return;
        }
        if self.explored.contains(c) {
            return; // already covered; nothing left to warn about
        }
        let witnesses = self.hazard_reports.entry(c.to_string()).or_default();
        witnesses.insert(a);
        if witnesses.len() >= 2 {
            self.confirmed_hazards.insert(c.to_string());
            self.unreachable.insert(c.to_string());
            self.claimed.remove(c);
        }
    }

    // ---- ground tier ----
    fn claim(&mut self, agent: Option<String>, sector: Option<&str>) {
        let (Some(a), Some(sec)) = (agent, sector) else { return };
        let ok = self.phase == SURVEYING
            && !self.unhealthy.contains(&a)
            && self.sectors.contains(sec)
            && self.surveyed.contains(sec) // THE cross-tier gate
            && !self.explored.contains(sec)
            && !self.unreachable.contains(sec)
            && !self.claimed.contains_key(sec)
            && !self.claimed.values().any(|h| h == &a);
        if ok {
            self.claimed.insert(sec.to_string(), a);
        }
    }

    fn explored_op(&mut self, agent: Option<String>, sector: Option<&str>) {
        let (Some(a), Some(sec)) = (agent, sector) else { return };
        if self.claimed.get(sec) != Some(&a) {
            return; // only the holder credits a sector
        }
        self.explored.insert(sec.to_string());
        self.explored_by.insert(sec.to_string(), a);
        self.claimed.remove(sec);
    }

    // ---- health, uniform across tiers ----
    fn health(&mut self, agent: Option<String>, seq: Option<i64>, ok: bool) {
        let (Some(a), Some(q)) = (agent, seq) else { return };
        if q <= *self.health_seq.get(&a).unwrap_or(&-1) {
            return; // stale or duplicate beacon
        }
        self.health_seq.insert(a.clone(), q);
        if ok {
            self.unhealthy.remove(&a);
        } else {
            self.mark_unhealthy(&a);
        }
    }

    fn suspect(&mut self, victim: Option<&str>, seen_seq: Option<i64>) {
        let Some(v) = victim else { return };
        // Acts only if the victim has not beaconed since the observation, so a
        // late beacon voids the suspicion and duplicates are no-ops.
        if self.health_seq.get(v).copied().unwrap_or(-1) == seen_seq.unwrap_or(-1) {
            self.mark_unhealthy(v);
        }
    }

    fn mark_unhealthy(&mut self, agent: &str) {
        self.unhealthy.insert(agent.to_string());
        self.claimed.retain(|_, h| h != agent);
        self.block_claims.retain(|_, h| h != agent);
    }

    fn recompute_phase(&mut self) {
        let blocks_left = self.blocks.difference(&self.surveyed_blocks).count();
        let ground_left = self
            .sectors
            .iter()
            .filter(|s| !self.explored.contains(*s) && !self.unreachable.contains(*s))
            .count();
        self.phase = if blocks_left == 0 && ground_left == 0 {
            DONE.to_string()
        } else {
            SURVEYING.to_string()
        };
    }

    // ---- derived state ----
    pub fn claimable_blocks(&self) -> Vec<String> {
        self.blocks
            .iter()
            .filter(|b| !self.surveyed_blocks.contains(*b) && !self.block_claims.contains_key(*b))
            .cloned()
            .collect()
    }

    pub fn my_block(&self, me: &str) -> Option<String> {
        self.block_claims
            .iter()
            .find(|(_, h)| h.as_str() == me)
            .map(|(b, _)| b.clone())
    }

    /// The language-neutral view compared against the Python fold. Must stay
    /// field-for-field identical to `canonical_snapshot` in airground_fsm.py.
    pub fn snapshot(&self) -> Value {
        let arr = |set: &BTreeSet<String>| Value::from(set.iter().cloned().collect::<Vec<_>>());
        let map = |m: &BTreeMap<String, String>| {
            Value::Object(
                m.iter()
                    .map(|(k, v)| (k.clone(), Value::from(v.clone())))
                    .collect(),
            )
        };
        serde_json::json!({
            "block_claims": map(&self.block_claims),
            "surveyed_blocks": arr(&self.surveyed_blocks),
            "surveyed": arr(&self.surveyed),
            "grounded": arr(&self.grounded),
            "claimed": map(&self.claimed),
            "explored": arr(&self.explored),
            "explored_by": map(&self.explored_by),
            "unreachable": arr(&self.unreachable),
            "hazard_reports": Value::Object(
                self.hazard_reports.iter()
                    .map(|(c, w)| (c.clone(), arr(w)))
                    .collect()),
            "confirmed_hazards": arr(&self.confirmed_hazards),
            "unhealthy": arr(&self.unhealthy),
            "health_seq": Value::Object(
                self.health_seq.iter()
                    .map(|(a, q)| (a.clone(), Value::from(*q)))
                    .collect()),
            "epoch": self.epoch,
            "phase": self.phase,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The cross-language contract. Replays `fixtures/conformance.json`, the
    /// same file `test_airground_fsm.py` replays, and checks the canonical
    /// snapshot after every single record. If this and the Python fold drift,
    /// one of the two goes red at the exact record that caused it.
    #[test]
    fn matches_the_python_fold_record_for_record() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../fixtures/conformance.json");
        let raw = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("cannot read {path}: {e} (run fixtures/gen_conformance.py)"));
        let doc: Value = serde_json::from_str(&raw).expect("conformance.json is not valid JSON");

        let strings = |v: &Value| -> Vec<String> {
            v.as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_str().unwrap().to_string())
                .collect()
        };
        let block_cells: BTreeMap<String, Vec<String>> = doc["block_cells"]
            .as_object()
            .unwrap()
            .iter()
            .map(|(k, v)| (k.clone(), strings(v)))
            .collect();

        let mut st = AirGroundState::new(
            strings(&doc["sectors"]),
            strings(&doc["blocks"]),
            block_cells,
        );

        let snapshots = doc["snapshots"].as_array().unwrap();
        assert_eq!(st.snapshot(), snapshots[0], "initial state differs");

        for (i, rec) in doc["log"].as_array().unwrap().iter().enumerate() {
            st.apply(rec);
            assert_eq!(
                st.snapshot(),
                snapshots[i + 1],
                "snapshot {} differs after applying {}",
                i + 1,
                rec
            );
        }
    }

    /// The geometry helpers are duplicated across the two languages too, so
    /// they get the same treatment.
    #[test]
    fn geometry_matches_the_fixture() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../fixtures/conformance.json");
        let doc: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let g = &doc["geometry"];
        let u = |k: &str| g[k].as_u64().unwrap() as usize;
        let f = |k: &str| g[k].as_f64().unwrap();

        let (sectors, _) = make_sectors(u("nx"), u("ny"), f("min_x"), f("min_y"), f("cell_w"), f("cell_h"));
        let (blocks, cells) = make_blocks(u("nx"), u("ny"), u("block_w"), u("block_h"));

        assert_eq!(Value::from(sectors), doc["sectors"]);
        assert_eq!(Value::from(blocks), doc["blocks"]);
        for (b, members) in cells {
            assert_eq!(Value::from(members), doc["block_cells"][&b], "block {b}");
        }
    }

    /// A ragged grid must not invent sectors that do not exist.
    #[test]
    fn ragged_blocks_do_not_invent_sectors() {
        let (ids, cells) = make_blocks(3, 1, 2, 1);
        assert_eq!(ids, vec!["B00", "B01"]);
        assert_eq!(cells["B01"], vec!["S02"]);
    }
}

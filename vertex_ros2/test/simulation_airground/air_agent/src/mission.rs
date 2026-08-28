//! What a drone should propose and where it should fly, given the folded
//! state and the latest telemetry.
//!
//! Split out from `main.rs` so the decisions are testable without an engine, a
//! socket, or Webots. Everything here is a pure function of (folded state,
//! telemetry, local plan) except the plan cursor itself.
//!
//! The division of labour matches the ground tier's: consensus owns who
//! surveys what, the airframe owns how to fly there, and this module is the
//! seam. Local sensing goes in, proposals come out.

use std::collections::BTreeMap;

use serde_json::{json, Value};

use crate::fold::AirGroundState;
use crate::link::Telemetry;

/// Altitude for a survey pass, metres. High enough to clear the arena's
/// BigSassafras canopy, low enough that the pit signal stays crisp.
pub const SURVEY_ALT: f64 = 12.0;

/// A pit reads as ground that is further away than it should be. The arena's
/// craters are around a metre deep, so a margin well under that separates a
/// real hole from ranging noise and from grass.
pub const PIT_MARGIN: f64 = 0.6;

/// Below this remaining battery the drone hands its block back and returns.
pub const RTB_BATTERY: f64 = 0.25;
/// And resumes once recharged past this.
pub const READY_BATTERY: f64 = 0.9;

/// Sensor staleness that makes a drone report itself unhealthy, seconds.
pub const STALE_AFTER: f64 = 3.0;

/// Horizontal distance at which a waypoint counts as reached, metres.
const WP_RADIUS: f64 = 2.5;

/// How the agent behaves. `Honest` is phase A; the other two are the phase-B
/// fault injection, and exist so the corroboration rule can be demonstrated
/// rather than merely asserted in a unit test.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Conduct {
    /// Report what the sensor saw.
    Honest,
    /// Report every block clear, including blocks with real pits in them.
    /// A bot will drive into one, stall, and become the second witness.
    FalseClear,
    /// Invent a hazard in every cell surveyed. Never corroborated by anyone,
    /// so it must never condemn a sector.
    PhantomHazards,
}

impl Conduct {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "honest" => Some(Conduct::Honest),
            "false-clear" => Some(Conduct::FalseClear),
            "phantom-hazards" => Some(Conduct::PhantomHazards),
            _ => None,
        }
    }
}

/// The order a block's sector centres get flown, as a boustrophedon pass.
///
/// Shared by `Plan::new` and `nearest_claimable_block`, so the block a drone
/// picks is scored on the point it will actually fly to first rather than on a
/// separately-derived guess. For a rectangular block those are the same cell
/// either way (see `entry_point_is_the_lowest_numbered_cell`), so this is about
/// having one definition of the pass order rather than two, not a bug fix.
///
/// Rows alternate direction so the drone never flies the width of the block
/// empty-handed.
pub fn survey_waypoints(
    cells: &[String],
    centers: &BTreeMap<String, (f64, f64)>,
) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = cells.iter().filter_map(|c| centers.get(c).copied()).collect();
    pts.sort_by(|a, b| {
        a.1.partial_cmp(&b.1)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
    });
    let mut serpentine = Vec::with_capacity(pts.len());
    let mut row: Vec<(f64, f64)> = Vec::new();
    let mut flip = false;
    for p in pts {
        if row.last().is_some_and(|l| (l.1 - p.1).abs() > 1e-6) {
            if flip {
                row.reverse();
            }
            serpentine.append(&mut row);
            flip = !flip;
        }
        row.push(p);
    }
    if flip {
        row.reverse();
    }
    serpentine.append(&mut row);
    serpentine
}

/// The lawnmower pass over one block, plus what the ranger saw along the way.
pub struct Plan {
    pub block: String,
    cells: Vec<String>,
    waypoints: Vec<(f64, f64)>,
    cursor: usize,
    /// Cells whose clearance came back long: candidate craters.
    pub sighted: Vec<String>,
}

impl Plan {
    /// Build a boustrophedon pass covering every sector centre in the block.
    ///
    /// Flying centre to centre rather than a fixed-pitch raster keeps the
    /// sampling aligned with the grid the ground tier claims in, so a
    /// clearance reading maps to exactly one sector with no interpolation.
    pub fn new(block: &str, cells: &[String], centers: &BTreeMap<String, (f64, f64)>) -> Self {
        let serpentine = survey_waypoints(cells, centers);

        Plan {
            block: block.to_string(),
            cells: cells.to_vec(),
            waypoints: serpentine,
            cursor: 0,
            sighted: Vec::new(),
        }
    }

    pub fn current(&self) -> Option<(f64, f64)> {
        self.waypoints.get(self.cursor).copied()
    }

    pub fn finished(&self) -> bool {
        self.cursor >= self.waypoints.len()
    }

    /// Sample the ranger. Call this on EVERY telemetry frame, not just on
    /// waypoint arrival.
    ///
    /// Sampling only at arrival misses craters: the arrival test has a
    /// tolerance, so the one reading is taken up to `WP_RADIUS` off the sector
    /// centre, where a bowl is far shallower than at its middle. With a 4 m
    /// crater that put the single sample on a 0.47 m lip against a 0.6 m
    /// threshold, and the drones reported a clear map over craters they had
    /// flown straight across. A real survey integrates along the whole pass,
    /// which is also cheaper to reason about: any frame over any cell of the
    /// block can raise the alarm.
    pub fn observe(&mut self, t: &Telemetry, cell: Option<&str>) {
        let Some(c) = cell else { return };
        // Only cells of the block we hold: a drone reports what it flew over.
        if !self.cells.iter().any(|x| x == c) {
            return;
        }
        // Expected clearance is the altitude itself over flat ground. A crater
        // shows up as materially more range for the same altitude.
        if t.clearance.is_finite() && t.clearance > t.z + PIT_MARGIN
            && !self.sighted.iter().any(|s| s == c)
        {
            self.sighted.push(c.to_string());
        }
    }

    /// Advance the pass if the drone has reached the current waypoint.
    pub fn advance(&mut self, t: &Telemetry) -> bool {
        let Some((wx, wy)) = self.current() else { return false };
        if (t.x - wx).hypot(t.y - wy) > WP_RADIUS {
            return false;
        }
        self.cursor += 1;
        true
    }
}

/// Which sector a position falls in, or `None` if outside the grid.
pub fn sector_at(
    x: f64,
    y: f64,
    nx: usize,
    ny: usize,
    min_x: f64,
    min_y: f64,
    cell_w: f64,
    cell_h: f64,
) -> Option<String> {
    let ix = ((x - min_x) / cell_w).floor();
    let iy = ((y - min_y) / cell_h).floor();
    if ix < 0.0 || iy < 0.0 || ix >= nx as f64 || iy >= ny as f64 {
        return None;
    }
    Some(format!("S{:02}", iy as usize * nx + ix as usize))
}

/// The nearest claimable block, measured to the point the drone would actually
/// fly to first. Ties break on block id, so two drones racing for work still
/// each produce a deterministic proposal.
pub fn nearest_claimable_block(
    st: &AirGroundState,
    t: &Telemetry,
    block_cells: &BTreeMap<String, Vec<String>>,
    centers: &BTreeMap<String, (f64, f64)>,
) -> Option<String> {
    st.claimable_blocks()
        .into_iter()
        .filter_map(|b| {
            let cells = block_cells.get(&b)?;
            let (cx, cy) = *survey_waypoints(cells, centers).first()?;
            Some((b, (t.x - cx).hypot(t.y - cy)))
        })
        .min_by(|a, c| {
            a.1.partial_cmp(&c.1)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.0.cmp(&c.0))
        })
        .map(|(b, _)| b)
}

/// The records a finished survey pass produces, in the order they should be
/// submitted. Hazards go first so a bot that claims the moment the survey
/// lands already sees the warning.
pub fn survey_records(
    me: &str,
    plan: &Plan,
    block_cells: &BTreeMap<String, Vec<String>>,
    conduct: Conduct,
    epoch: i64,
) -> Vec<Value> {
    let cells = block_cells.get(&plan.block).cloned().unwrap_or_default();
    let sighted: Vec<String> = match conduct {
        Conduct::Honest => plan.sighted.clone(),
        // Report nothing, whatever the ranger saw. The pit is still physically
        // there, so the bot that drives in supplies the second witness.
        Conduct::FalseClear => Vec::new(),
        // Cry wolf over the whole block. No second witness ever agrees, so the
        // fold must leave every one of these provisional.
        Conduct::PhantomHazards => cells.clone(),
    };

    let mut out: Vec<Value> = sighted
        .iter()
        .map(|c| json!({"op": "hazard", "agent": me, "cell": c, "kind": "pit", "epoch": epoch}))
        .collect();
    out.push(json!({
        "op": "surveyed", "agent": me, "block": plan.block,
        "cells": cells, "epoch": epoch,
    }));
    out
}

/// Self-assessed health, the air-tier twin of the ground tier's stream-age
/// check. A drone whose telemetry has gone stale says so itself, and the fold
/// releases its block; if it dies outright and stops beaconing entirely, a
/// peer's `suspect` covers it.
pub fn health_record(me: &str, seq: i64, t: Option<&Telemetry>, epoch: i64) -> Value {
    let ok = t.is_some_and(|t| t.age <= STALE_AFTER);
    json!({"op": "health", "agent": me, "seq": seq, "ok": ok, "epoch": epoch})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tele(x: f64, y: f64, z: f64, clearance: f64) -> Telemetry {
        Telemetry { x, y, z, clearance, battery: 1.0, age: 0.0 }
    }

    fn centers() -> BTreeMap<String, (f64, f64)> {
        let (_, c) = crate::fold::make_sectors(4, 3, -20.0, -15.0, 10.0, 10.0);
        c
    }

    #[test]
    fn sector_lookup_matches_the_grid() {
        let c = centers();
        for (sid, (x, y)) in &c {
            let got = sector_at(*x, *y, 4, 3, -20.0, -15.0, 10.0, 10.0);
            assert_eq!(got.as_deref(), Some(sid.as_str()));
        }
        // outside the grid on every side
        assert_eq!(sector_at(-21.0, 0.0, 4, 3, -20.0, -15.0, 10.0, 10.0), None);
        assert_eq!(sector_at(0.0, 16.0, 4, 3, -20.0, -15.0, 10.0, 10.0), None);
        assert_eq!(sector_at(20.1, 0.0, 4, 3, -20.0, -15.0, 10.0, 10.0), None);
    }

    #[test]
    fn plan_visits_every_cell_once() {
        let c = centers();
        let cells = vec!["S00".to_string(), "S01".to_string()];
        let plan = Plan::new("B00", &cells, &c);
        assert_eq!(plan.waypoints.len(), 2);
        assert!(!plan.finished());
    }

    #[test]
    fn plan_serpentines_across_rows() {
        // Two rows of two: row 0 left-to-right, row 1 right-to-left, so the
        // drone does not deadhead back across the block between rows.
        let (_, c) = crate::fold::make_sectors(2, 2, 0.0, 0.0, 10.0, 10.0);
        let cells: Vec<String> = vec!["S00", "S01", "S02", "S03"]
            .into_iter()
            .map(str::to_string)
            .collect();
        let plan = Plan::new("B00", &cells, &c);
        assert_eq!(plan.waypoints, vec![(5.0, 5.0), (15.0, 5.0), (15.0, 15.0), (5.0, 15.0)]);
    }

    #[test]
    fn long_clearance_at_a_waypoint_is_a_sighting() {
        let c = centers();
        let mut plan = Plan::new("B00", &["S00".to_string()], &c);
        let (wx, wy) = plan.current().unwrap();
        // Ground reads a metre further away than the altitude: a crater.
        plan.observe(&tele(wx, wy, SURVEY_ALT, SURVEY_ALT + 1.0), Some("S00"));
        assert!(plan.advance(&tele(wx, wy, SURVEY_ALT, SURVEY_ALT + 1.0)));
        assert_eq!(plan.sighted, vec!["S00"]);
        assert!(plan.finished());
    }

    #[test]
    fn flat_ground_is_not_a_sighting() {
        let c = centers();
        let mut plan = Plan::new("B00", &["S00".to_string()], &c);
        let (wx, wy) = plan.current().unwrap();
        // A little ranging noise, well under the margin.
        plan.observe(&tele(wx, wy, SURVEY_ALT, SURVEY_ALT + 0.1), Some("S00"));
        assert!(plan.sighted.is_empty());
    }

    #[test]
    fn distant_drone_does_not_advance() {
        let c = centers();
        let mut plan = Plan::new("B00", &["S00".to_string()], &c);
        assert!(!plan.advance(&tele(100.0, 100.0, SURVEY_ALT, SURVEY_ALT)));
        assert!(!plan.finished());
    }

    #[test]
    fn a_liar_reports_no_hazards_it_saw() {
        let c = centers();
        let (_, block_cells) = crate::fold::make_blocks(4, 3, 2, 1);
        let mut plan = Plan::new("B00", &block_cells["B00"], &c);
        plan.sighted.push("S00".to_string());

        let honest = survey_records("drone_0", &plan, &block_cells, Conduct::Honest, 0);
        assert_eq!(honest.len(), 2, "one hazard plus the survey");
        assert_eq!(honest[0]["op"], "hazard");

        let lying = survey_records("drone_1", &plan, &block_cells, Conduct::FalseClear, 0);
        assert_eq!(lying.len(), 1, "false-clear must emit only the survey");
        assert_eq!(lying[0]["op"], "surveyed");
    }

    #[test]
    fn phantom_conduct_flags_the_whole_block() {
        let c = centers();
        let (_, block_cells) = crate::fold::make_blocks(4, 3, 2, 1);
        let plan = Plan::new("B00", &block_cells["B00"], &c);
        let recs = survey_records("drone_1", &plan, &block_cells, Conduct::PhantomHazards, 0);
        assert_eq!(recs.len(), block_cells["B00"].len() + 1);
        assert!(recs[..recs.len() - 1].iter().all(|r| r["op"] == "hazard"));
    }

    #[test]
    fn health_follows_stream_age() {
        let fresh = tele(0.0, 0.0, 12.0, 12.0);
        assert_eq!(health_record("drone_0", 1, Some(&fresh), 0)["ok"], true);
        let mut stale = fresh;
        stale.age = STALE_AFTER + 1.0;
        assert_eq!(health_record("drone_0", 2, Some(&stale), 0)["ok"], false);
        // No telemetry at all is not healthy either: nothing is connected yet.
        assert_eq!(health_record("drone_0", 3, None, 0)["ok"], false);
    }

    #[test]
    fn entry_point_is_the_lowest_numbered_cell() {
        // Not a coincidence worth relying on silently. The pass starts at the
        // lowest row travelling in +x, and sectors are numbered row-major from
        // the south-west, so a rectangular block's first waypoint is always its
        // lowest-numbered cell. Pinning it here means `survey_waypoints` cannot
        // be reordered without something going red, since the selection metric
        // reads its first element.
        for (nx, ny, bw, bh) in [(4, 3, 2, 1), (4, 4, 2, 2), (6, 4, 3, 2), (3, 1, 2, 1)] {
            let (_, c) = crate::fold::make_sectors(nx, ny, -20.0, -20.0, 10.0, 10.0);
            let (blocks, cells) = crate::fold::make_blocks(nx, ny, bw, bh);
            for b in &blocks {
                let m = &cells[b];
                assert_eq!(
                    *survey_waypoints(m, &c).first().unwrap(),
                    c[m.first().unwrap()],
                    "{b} ({nx}x{ny} grid, {bw}x{bh} blocks): the pass must start \
                     at the lowest-numbered cell"
                );
            }
        }
    }

    #[test]
    fn closest_block_wins_from_any_corner() {
        let c = centers();
        let (blocks, block_cells) = crate::fold::make_blocks(4, 3, 2, 1);
        let st = AirGroundState::new(
            crate::fold::make_sectors(4, 3, -20.0, -15.0, 10.0, 10.0).0,
            blocks, block_cells.clone());
        // From each corner of the arena the nearest block by entry point must
        // be the one selected. Computed independently of the function under
        // test, so this is a real check rather than a restatement.
        for (px, py) in [(-24.0, -24.0), (24.0, 24.0), (-24.0, 24.0), (24.0, -24.0)] {
            let want = block_cells
                .iter()
                .map(|(b, cells)| {
                    let (ex, ey) = *survey_waypoints(cells, &c).first().unwrap();
                    (b.clone(), (px - ex).hypot(py - ey))
                })
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap().then(a.0.cmp(&b.0)))
                .map(|(b, _)| b)
                .unwrap();
            let got = nearest_claimable_block(
                &st, &tele(px, py, 12.0, 12.0), &block_cells, &c);
            assert_eq!(got.as_deref(), Some(want.as_str()),
                       "from ({px}, {py}) the closest block is {want}");
        }
    }

    #[test]
    fn nearest_block_is_deterministic() {
        let c = centers();
        let (blocks, block_cells) = crate::fold::make_blocks(4, 3, 2, 1);
        let st = AirGroundState::new(
            crate::fold::make_sectors(4, 3, -20.0, -15.0, 10.0, 10.0).0,
            blocks,
            block_cells.clone(),
        );
        // Sitting on the south-west corner, B00 is nearest.
        let t = tele(-20.0, -15.0, 12.0, 12.0);
        assert_eq!(
            nearest_claimable_block(&st, &t, &block_cells, &c).as_deref(),
            Some("B00")
        );
    }
}

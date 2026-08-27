"use strict";
// The air/ground fold, in JavaScript — the viewer's copy.
//
// This is the THIRD implementation of the same fold: Python in
// nodes/airground_fsm.py (what the tessera bots run), Rust in
// air_agent/src/fold.rs (what the drones run), and this one, so the viewer can
// decode /vertex/event itself instead of trusting a summary.
//
// Three copies of a rule is exactly how a protocol drifts, so all three are
// pinned to one file: fixtures/conformance.json, a scripted record log plus
// the canonical state snapshot after every record. airground_fold.test.js
// replays it here under Node, test_airground_fsm.py replays it in Python, and
// a cargo test replays it in Rust. Change one and the other two go red at the
// exact record that diverged.
//
// Keep every branch pure and dependent only on the log.

class AirGroundFold {
  constructor(sectors, blocks, blockCells) {
    this.sectors = new Set(sectors);
    this.blocks = new Set(blocks);
    this.blockCells = blockCells || {};
    this.epoch = 0;
    this.wipe();
  }

  wipe() {
    // Provenance: WHO did what to each sector and block. Deliberately kept
    // out of snapshot(), which is the cross-language contract pinned by
    // fixtures/conformance.json and must stay byte-comparable with the Python
    // and Rust folds. This is viewer-only narration derived from the same
    // records, not shared state.
    this.story = {};              // sector -> {surveyedBy, claims[], abandons{}, hazards[], exploredBy}
    this.blockStory = {};         // block  -> {claims[], surveyedBy, sightings[]}
    this.contrib = {};            // agent  -> counters
    this.blockClaims = {};        // block -> drone
    this.surveyedBlocks = new Set();
    this.surveyed = new Set();    // sectors cleared for the ground tier
    this.grounded = new Set();    // drones that submitted rtb
    this.claimed = {};            // sector -> bot
    this.explored = new Set();
    this.exploredBy = {};
    this.unreachable = new Set();
    this.hazardReports = {};      // cell -> Set of distinct witnesses
    this.confirmedHazards = new Set();
    this.unhealthy = new Set();
    this.healthSeq = {};
    this.phase = "surveying";
  }

  _sec(id) {
    return (this.story[id] ||= { surveyedBy: null, claims: [], abandons: {},
                                 hazards: [], exploredBy: null });
  }

  _blk(id) {
    return (this.blockStory[id] ||= { claims: [], surveyedBy: null, sightings: [] });
  }

  _tally(agent, key) {
    if (!agent) return;
    const c = (this.contrib[agent] ||= { surveyed: 0, sighted: 0, explored: 0,
                                         abandoned: 0, corroborated: 0, rtb: 0,
                                         claimsWon: 0, claimsLost: 0 });
    c[key] = (c[key] || 0) + 1;
  }

  /** Epoch gate and reset, mirroring vertex_fleet.ReplicatedState.apply. */
  apply(rec) {
    if (!rec || typeof rec.op !== "string") return { accepted: false, reason: "malformed" };
    if (rec.op === "reset") {
      const e = typeof rec.epoch === "number" ? rec.epoch : 0;
      if (e > this.epoch) { this.wipe(); this.epoch = e; return { accepted: true }; }
      return { accepted: false, reason: "stale reset" };
    }
    if ((typeof rec.epoch === "number" ? rec.epoch : 0) !== this.epoch) {
      return { accepted: false, reason: "stale epoch" };
    }
    const before = this.fingerprint();
    this.applyRecord(rec);
    this.recomputePhase();
    return { accepted: this.fingerprint() !== before };
  }

  applyRecord(rec) {
    const a = rec.agent;
    switch (rec.op) {
      case "survey_claim": return this.surveyClaim(a, rec.block);
      case "surveyed":     return this.surveyedOp(a, rec.block, rec.cells);
      // hazard and corroborate fold identically: what matters is how many
      // DISTINCT agents vouched, not which op carried the claim.
      case "hazard":
      case "corroborate":  return this.hazard(a, rec.cell);
      case "claim":        return this.claim(a, rec.sector);
      case "explored":     return this.exploredOp(a, rec.sector);
      case "abandon": {
        if (a != null && this.claimed[rec.sector] === a) delete this.claimed[rec.sector];
        if (a != null && rec.sector != null) {
          const ab = this._sec(rec.sector).abandons;
          ab[a] = (ab[a] || 0) + 1;
          this._tally(a, "abandoned");
        }
        return;
      }
      case "rtb":          this._tally(a, "rtb"); return this.rtb(a);
      case "ready":        if (a != null) this.grounded.delete(a); return;
      case "health":       return this.health(a, rec.seq, !!rec.ok);
      case "suspect":      return this.suspect(rec.victim, rec.seen_seq);
      default: return;
    }
  }

  // ---- air tier ----
  surveyClaim(agent, block) {
    if (agent == null || block == null) return;
    const ok = this.phase === "surveying"
      && !this.unhealthy.has(agent)
      && !this.grounded.has(agent)
      && this.blocks.has(block)
      && !this.surveyedBlocks.has(block)
      && !(block in this.blockClaims)
      && !Object.values(this.blockClaims).includes(agent);
    this._blk(block).claims.push({ agent, won: ok });
    this._tally(agent, ok ? "claimsWon" : "claimsLost");
    if (ok) this.blockClaims[block] = agent;
  }

  surveyedOp(agent, block, cells) {
    if (agent == null || block == null) return;
    if (this.blockClaims[block] !== agent) return;   // only the holder reports
    this.surveyedBlocks.add(block);
    delete this.blockClaims[block];
    // Clipped to the block actually held: a lying drone cannot clear the whole
    // map with one record.
    const allowed = new Set(this.blockCells[block] || []);
    const named = Array.isArray(cells) ? cells : [...allowed];
    for (const c of named) if (allowed.has(c)) {
      this.surveyed.add(c);
      this._sec(c).surveyedBy = agent;
    }
    this._blk(block).surveyedBy = agent;
    this._tally(agent, "surveyed");
  }

  rtb(agent) {
    if (agent == null) return;
    this.grounded.add(agent);
    for (const [b, h] of Object.entries(this.blockClaims)) {
      if (h === agent) delete this.blockClaims[b];
    }
  }

  // ---- evidence: one witness is provisional, two distinct witnesses confirm ----
  hazard(agent, cell) {
    if (agent == null || cell == null) return;
    if (this.unhealthy.has(agent) || !this.sectors.has(cell)) return;
    if (this.explored.has(cell)) return;   // already covered, nothing to warn
    const w = this.hazardReports[cell] || (this.hazardReports[cell] = new Set());
    const fresh = !w.has(agent);
    w.add(agent);
    if (fresh) {
      this._sec(cell).hazards.push(agent);
      this._tally(agent, w.size === 1 ? "sighted" : "corroborated");
    }
    if (w.size >= 2) {
      this.confirmedHazards.add(cell);
      this.unreachable.add(cell);
      delete this.claimed[cell];
    }
  }

  // ---- ground tier ----
  claim(agent, sector) {
    if (agent == null || sector == null) return;
    const ok = this.phase === "surveying"
      && !this.unhealthy.has(agent)
      && this.sectors.has(sector)
      && this.surveyed.has(sector)            // THE cross-tier gate
      && !this.explored.has(sector)
      && !this.unreachable.has(sector)
      && !(sector in this.claimed)
      && !Object.values(this.claimed).includes(agent);
    this._sec(sector).claims.push({ agent, won: ok });
    this._tally(agent, ok ? "claimsWon" : "claimsLost");
    if (ok) this.claimed[sector] = agent;
  }

  exploredOp(agent, sector) {
    if (agent == null || sector == null) return;
    if (this.claimed[sector] !== agent) return;   // only the holder credits it
    this.explored.add(sector);
    this.exploredBy[sector] = agent;
    this._sec(sector).exploredBy = agent;
    this._tally(agent, "explored");
    delete this.claimed[sector];
  }

  // ---- health, uniform across tiers ----
  health(agent, seq, ok) {
    if (agent == null || typeof seq !== "number") return;
    if (seq <= (this.healthSeq[agent] ?? -1)) return;   // stale or duplicate
    this.healthSeq[agent] = seq;
    if (ok) this.unhealthy.delete(agent);
    else this.markUnhealthy(agent);
  }

  suspect(victim, seenSeq) {
    if (victim == null) return;
    // Acts only if the victim has not beaconed since the observation, so a
    // late beacon voids the suspicion and duplicates are no-ops.
    if ((this.healthSeq[victim] ?? -1) === (seenSeq ?? -1)) this.markUnhealthy(victim);
  }

  markUnhealthy(agent) {
    this.unhealthy.add(agent);
    for (const [s, h] of Object.entries(this.claimed)) if (h === agent) delete this.claimed[s];
    for (const [b, h] of Object.entries(this.blockClaims)) if (h === agent) delete this.blockClaims[b];
  }

  recomputePhase() {
    let blocksLeft = 0;
    for (const b of this.blocks) if (!this.surveyedBlocks.has(b)) blocksLeft++;
    let groundLeft = 0;
    for (const s of this.sectors) {
      if (!this.explored.has(s) && !this.unreachable.has(s)) groundLeft++;
    }
    this.phase = (blocksLeft === 0 && groundLeft === 0) ? "done" : "surveying";
  }

  // ---- derived ----
  provisionalHazards() {
    return Object.entries(this.hazardReports)
      .filter(([c, w]) => w.size === 1 && !this.confirmedHazards.has(c))
      .map(([c]) => c).sort();
  }

  myBlock(agent) {
    for (const [b, h] of Object.entries(this.blockClaims)) if (h === agent) return b;
    return null;
  }

  fingerprint() { return JSON.stringify(this.snapshot()); }

  /** The canonical snapshot — must stay field-for-field identical to
   *  canonical_snapshot() in Python and snapshot() in Rust. */
  snapshot() {
    const sorted = (set) => [...set].sort();
    const sortedObj = (o) => Object.fromEntries(Object.entries(o).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0)));
    return {
      block_claims: sortedObj(this.blockClaims),
      surveyed_blocks: sorted(this.surveyedBlocks),
      surveyed: sorted(this.surveyed),
      grounded: sorted(this.grounded),
      claimed: sortedObj(this.claimed),
      explored: sorted(this.explored),
      explored_by: sortedObj(this.exploredBy),
      unreachable: sorted(this.unreachable),
      hazard_reports: Object.fromEntries(
        Object.entries(this.hazardReports).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0))
          .map(([c, w]) => [c, sorted(w)])),
      confirmed_hazards: sorted(this.confirmedHazards),
      unhealthy: sorted(this.unhealthy),
      health_seq: sortedObj(this.healthSeq),
      epoch: this.epoch,
      phase: this.phase,
    };
  }
}

// ---- geometry, mirroring make_sectors / make_blocks ----
function makeSectors(nx, ny, minX, minY, cellW, cellH) {
  const ids = [], centers = {};
  for (let iy = 0; iy < ny; iy++) {
    for (let ix = 0; ix < nx; ix++) {
      const sid = "S" + String(iy * nx + ix).padStart(2, "0");
      ids.push(sid);
      centers[sid] = [minX + (ix + 0.5) * cellW, minY + (iy + 0.5) * cellH];
    }
  }
  return { ids, centers };
}

function makeBlocks(nx, ny, blockW, blockH) {
  const ids = [], cells = {};
  const bx = Math.ceil(nx / blockW), by = Math.ceil(ny / blockH);
  for (let iy = 0; iy < by; iy++) {
    for (let ix = 0; ix < bx; ix++) {
      const bid = "B" + String(iy * bx + ix).padStart(2, "0");
      ids.push(bid);
      const members = [];
      for (let dy = 0; dy < blockH; dy++) {
        for (let dx = 0; dx < blockW; dx++) {
          const sx = ix * blockW + dx, sy = iy * blockH + dy;
          if (sx < nx && sy < ny) members.push("S" + String(sy * nx + sx).padStart(2, "0"));
        }
      }
      cells[bid] = members;
    }
  }
  return { ids, cells };
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { AirGroundFold, makeSectors, makeBlocks };
}

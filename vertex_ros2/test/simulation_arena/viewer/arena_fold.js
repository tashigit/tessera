"use strict";

// ---- deterministic fold, mirrors vertex_fleet.ReplicatedState + arena_fsm.ArenaState ----
// (plus exploredBy, which Python's fold discards but the raw "explored" record
// already carries — the whole point of decoding the real consensus stream
// client-side instead of relying on a derived-state topic).
//
// Split out of arena_viewer.html so arena_fold.test.js can exercise the exact
// code the browser runs (no build step either way: a plain <script src> tag
// picks this up in the browser, `require("./arena_fold.js")` picks it up in
// Node — the module.exports guard below only fires where `module` exists).
class ArenaFold {
  constructor(sectorIds) { this.sectorIds = sectorIds; this.epoch = 0; this.wipe(); }
  wipe() {
    this.claimed = {};            // sector -> bot
    this.explored = new Set();
    this.exploredBy = {};         // sector -> bot
    this.unreachable = new Set();
    this.unhealthy = new Set();
    this.healthSeq = {};          // bot -> seq
    this.detections = [];
    this.phase = "exploring";
  }
  applyRaw(rec) {
    if (!rec || !rec.op) return null;
    if (rec.op === "reset") {
      const e = rec.epoch || 0;
      if (typeof e === "number" && e > this.epoch) { this.wipe(); this.epoch = e; }
      return { accepted: true };
    }
    if ((rec.epoch || 0) !== this.epoch) return { accepted: false, reason: "stale epoch" };
    return this.applyRecord(rec);
  }
  applyRecord(rec) {
    const { op, bot, sector } = rec;
    let accepted = false;
    switch (op) {
      case "claim": {
        accepted = this.phase === "exploring" && bot != null && !this.unhealthy.has(bot)
          && this.sectorIds.includes(sector) && !this.explored.has(sector)
          && !this.unreachable.has(sector) && !(sector in this.claimed)
          && !Object.values(this.claimed).includes(bot);
        if (accepted) this.claimed[sector] = bot;
        break;
      }
      case "explored":
        accepted = this.claimed[sector] === bot;
        if (accepted) { this.explored.add(sector); this.exploredBy[sector] = bot; delete this.claimed[sector]; }
        break;
      case "abandon":
        accepted = this.claimed[sector] === bot;
        if (accepted) delete this.claimed[sector];
        break;
      case "unreachable":
        accepted = this.claimed[sector] === bot;
        if (accepted) { this.unreachable.add(sector); delete this.claimed[sector]; }
        break;
      case "health": {
        const seq = rec.seq;
        if (bot == null || typeof seq !== "number") break;
        accepted = seq > (this.healthSeq[bot] ?? -1);
        if (accepted) {
          this.healthSeq[bot] = seq;
          if (rec.ok) this.unhealthy.delete(bot); else this._markUnhealthy(bot);
        }
        break;
      }
      case "suspect": {
        const victim = rec.victim, seen = rec.seen_seq;
        if (victim == null) break;
        accepted = (this.healthSeq[victim] ?? -1) === seen;
        if (accepted) this._markUnhealthy(victim);
        break;
      }
      case "detection": {
        const seq = rec.seq;
        if (bot == null || this.unhealthy.has(bot)) break;
        accepted = !this.detections.some((d) => d.bot === bot && d.seq === seq);
        if (accepted) this.detections.push({ bot, seq, label: rec.label, x: rec.x, y: rec.y });
        break;
      }
      default: return { accepted: false, reason: "unknown op" };
    }
    this._recomputePhase();
    return { accepted };
  }
  _markUnhealthy(bot) {
    this.unhealthy.add(bot);
    for (const s of Object.keys(this.claimed)) if (this.claimed[s] === bot) delete this.claimed[s];
  }
  _recomputePhase() {
    const remaining = this.sectorIds.filter((s) => !this.explored.has(s) && !this.unreachable.has(s));
    this.phase = remaining.length === 0 ? "done" : "exploring";
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { ArenaFold };
}

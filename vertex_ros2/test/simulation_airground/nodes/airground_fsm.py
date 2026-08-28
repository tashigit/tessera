"""Pure replicated state machine for consensus-coordinated AIR/GROUND
SURVEY-AND-SWEEP — the third simulation: two Pioneer 3-AT ground bots and two
DJI Mavic 2 Pro drones cooperatively clear an arena, with every inter-agent
decision derived from the Vertex ordered log.

What makes this one different from the first two (../../simulation/ and
../../simulation_arena/, both homogeneous Python fleets on vertex_fleet):

  * The fleet is HETEROGENEOUS. The bots join consensus through tessera
    (vertex_node + this Python fold); the drones join through the raw
    tashi-vertex crate from Rust, with no ROS anywhere. Both tiers fold the
    same records with the same rules. air_agent/src/fold.rs is the Rust
    twin of this file, and fixtures/conformance.json pins them together.
  * The tiers have ASYMMETRIC SENSORS, and consensus is what reconciles them.
    A Pioneer's horizontal lidar cannot see a pit; a drone looking down can.
    So ground work depends on air work, ordered by the log.

Rules (see ../README.md):
  * Two claim namespaces, one exclusivity rule. A drone claims an air `block`,
    a bot claims a ground `sector`. First claim in consensus order wins; an
    agent holds at most one thing at a time.
  * Ground work is GATED ON AIR WORK. A sector is claimable only once some
    drone has reported `surveyed` for it and no confirmed hazard sits in it.
    Before that the sector is unknown and no bot may claim it. This is the
    cross-tier dependency, and it is ordered by consensus, not by wall clock.
  * Hazards need CORROBORATION. A `hazard` from one agent is provisional: it
    defers the sector but does not condemn it. A second DISTINCT agent
    reporting the same cell (another drone, or a bot that physically stalled
    there) promotes it to confirmed, which condemns the sector for good.
    This is the phase-B point: consensus makes everyone agree on what was
    SAID; corroboration is what turns that into what is TRUE. A lying drone
    can neither condemn a good sector on its own nor hide a real pit, because
    the bot that stalls on it is the second witness.
  * Battery lease. A drone low on flight time submits `rtb`; the fold releases
    its block and grounds it (no new survey claims) until a `ready` record.
    Same lease shape as the arena scenario's silence lease, different physics.
  * Health and the silence lease apply UNIFORMLY ACROSS TIERS: a bot may
    suspect a silent drone and a drone may suspect a silent bot.
  * DONE when every block is surveyed and every sector is explored or
    condemned unreachable.

Agent ids are STRINGS here ("bot_0", "drone_1"), not the integers the arena
scenario used: the fleet is mixed, so the id has to say which tier it is on.

Transaction payloads are opaque JSON records (all carry `epoch` for reset):
    {"op":"survey_claim","agent":"drone_0","block":"B03","epoch":0}
    {"op":"surveyed",    "agent":"drone_0","block":"B03",
                         "cells":["S06","S07"],"epoch":0}
    {"op":"hazard",      "agent":"drone_0","seq":7,"cell":"S09",
                         "kind":"pit","x":7.5,"y":11.6,"epoch":0}
    {"op":"corroborate", "agent":"bot_1","cell":"S09","epoch":0}
    {"op":"claim",       "agent":"bot_0","sector":"S06","epoch":0}
    {"op":"explored",    "agent":"bot_0","sector":"S06","epoch":0}
    {"op":"abandon",     "agent":"bot_0","sector":"S06","epoch":0}
    {"op":"rtb",         "agent":"drone_0","epoch":0}
    {"op":"ready",       "agent":"drone_0","epoch":0}
    {"op":"health",      "agent":"drone_1","seq":41,"ok":true,"epoch":0}
    {"op":"suspect",     "agent":"bot_0","victim":"drone_1","seen_seq":41,
                         "epoch":0}
    {"op":"reset",       "epoch":1}
"""

from __future__ import annotations

import os
import sys

# Built on the vertex_fleet library (the consumer API this simulation
# exercises end to end). When run straight from the source tree on a host
# without the workspace installed, resolve the package by path.
try:
    from vertex_fleet.state import ReplicatedState, decode, encode
except ImportError:                                    # pragma: no cover
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "vertex_fleet"))
    from vertex_fleet.state import ReplicatedState, decode, encode

__all__ = ["AirGroundState", "make_sectors", "make_blocks",
           "canonical_snapshot", "encode", "decode", "SURVEYING", "DONE"]

SURVEYING, DONE = "surveying", "done"


def canonical_snapshot(st) -> dict:
    """The language-neutral view of a folded state.

    This is the cross-implementation contract. The Python fold here and the
    Rust fold in air_agent/src/fold.rs must produce an EQUAL snapshot for
    every prefix of the same log, which is what fixtures/conformance.json
    tests. Everything is sorted so the comparison never depends on hash order
    in either language.
    """
    return {
        "block_claims": dict(sorted(st.block_claims.items())),
        "surveyed_blocks": sorted(st.surveyed_blocks),
        "surveyed": sorted(st.surveyed),
        "grounded": sorted(st.grounded),
        "claimed": dict(sorted(st.claimed.items())),
        "explored": sorted(st.explored),
        "explored_by": dict(sorted(st.explored_by.items())),
        "unreachable": sorted(st.unreachable),
        "hazard_reports": {c: sorted(w)
                           for c, w in sorted(st.hazard_reports.items())},
        "confirmed_hazards": sorted(st.confirmed_hazards),
        "unhealthy": sorted(st.unhealthy),
        "health_seq": dict(sorted(st.health_seq.items())),
        "epoch": st.epoch,
        "phase": st.phase,
    }


def make_sectors(nx: int, ny: int, min_x: float, min_y: float,
                 cell_w: float, cell_h: float):
    """Ground sector ids and centres for an nx-by-ny grid.

    Pure geometry shared by the coordinators, the robots (mock and Webots),
    the Rust air agents, and the tests, so every process keys sectors
    identically. Ids run row-major from the south-west corner.
    """
    ids, centers = [], {}
    for iy in range(ny):
        for ix in range(nx):
            sid = f"S{iy * nx + ix:02d}"
            ids.append(sid)
            centers[sid] = (min_x + (ix + 0.5) * cell_w,
                            min_y + (iy + 0.5) * cell_h)
    return ids, centers


def make_blocks(nx: int, ny: int, block_w: int, block_h: int):
    """Air survey blocks laid over the sector grid.

    Each block covers a block_w-by-block_h patch of sectors. Returns
    (ids, cells): ids row-major ("B00", "B01", ...), cells as
    {block_id: [sector_id, ...]} so a `surveyed` record can name exactly the
    sectors it clears. Blocks deliberately outnumber the drones, so survey
    claims actually contend.
    """
    ids, cells = [], {}
    bx = (nx + block_w - 1) // block_w
    by = (ny + block_h - 1) // block_h
    for iy in range(by):
        for ix in range(bx):
            bid = f"B{iy * bx + ix:02d}"
            ids.append(bid)
            members = []
            for dy in range(block_h):
                for dx in range(block_w):
                    sx, sy = ix * block_w + dx, iy * block_h + dy
                    if sx < nx and sy < ny:
                        members.append(f"S{sy * nx + sx:02d}")
            cells[bid] = members
    return ids, cells


class AirGroundState(ReplicatedState):
    """Deterministic fold of the consensus-ordered record stream, identical on
    every bot and every drone. Epoch gating and `reset` come from
    vertex_fleet.ReplicatedState; this class folds the air/ground ops.

    Keep every branch pure and order-dependent only on the log: the Rust twin
    in air_agent/src/fold.rs must produce byte-identical state for the same
    input, and fixtures/conformance.json is the test that proves it.
    """

    def __init__(self, sectors, blocks, block_cells, agents):
        self.sectors = list(sectors)
        self.blocks = list(blocks)
        self.block_cells = {b: list(c) for b, c in block_cells.items()}
        self.agents = list(agents)
        super().__init__()        # sets epoch = 0 and calls wipe()

    def wipe(self) -> None:
        # --- air tier ---
        self.block_claims: dict[str, str] = {}   # block -> drone, exclusive
        self.surveyed_blocks: set[str] = set()   # blocks fully swept from air
        self.surveyed: set[str] = set()          # sectors cleared for ground
        self.grounded: set[str] = set()          # drones that submitted rtb
        # --- ground tier ---
        self.claimed: dict[str, str] = {}        # sector -> bot, exclusive
        self.explored: set[str] = set()
        self.explored_by: dict[str, str] = {}
        self.unreachable: set[str] = set()       # confirmed-hazard condemned
        # --- evidence ---
        # cell -> set of agents that reported a hazard there. One witness is
        # provisional, two distinct witnesses confirm.
        self.hazard_reports: dict[str, set[str]] = {}
        self.confirmed_hazards: set[str] = set()
        # --- health, uniform across tiers ---
        self.unhealthy: set[str] = set()
        self.health_seq: dict[str, int] = {}
        self.phase = SURVEYING

    # ---- fold one same-epoch record (consensus order) ----
    def apply_record(self, rec: dict) -> None:
        op = rec.get("op")
        agent = rec.get("agent")

        if op == "survey_claim":
            self._survey_claim(agent, rec.get("block"))
        elif op == "surveyed":
            self._surveyed(agent, rec.get("block"), rec.get("cells"))
        elif op == "hazard":
            self._hazard(agent, rec.get("cell"))
        elif op == "corroborate":
            self._hazard(agent, rec.get("cell"))
        elif op == "claim":
            self._claim(agent, rec.get("sector"))
        elif op == "explored":
            self._explored(agent, rec.get("sector"))
        elif op == "abandon":
            if self.claimed.get(rec.get("sector")) == agent:
                del self.claimed[rec.get("sector")]
        elif op == "rtb":
            self._rtb(agent)
        elif op == "ready":
            if agent in self.grounded:
                self.grounded.discard(agent)
        elif op == "health":
            self._health(agent, rec.get("seq"), rec.get("ok"))
        elif op == "suspect":
            self._suspect(rec.get("victim"), rec.get("seen_seq"))

        self._recompute_phase()

    # ---- air tier ----
    def _survey_claim(self, agent, block) -> None:
        # Exclusive, first in consensus order wins. A grounded (rtb) or
        # unhealthy drone may not take new work.
        ok = (self.phase == SURVEYING
              and agent is not None
              and agent not in self.unhealthy
              and agent not in self.grounded
              and block in self.blocks
              and block not in self.surveyed_blocks
              and block not in self.block_claims
              and agent not in self.block_claims.values())
        if ok:
            self.block_claims[block] = agent

    def _surveyed(self, agent, block, cells) -> None:
        # Only the block's holder may report it surveyed, so a drone cannot
        # clear ground it never flew over.
        if self.block_claims.get(block) != agent:
            return
        self.surveyed_blocks.add(block)
        del self.block_claims[block]
        # A survey clears exactly the cells the reporter names, intersected
        # with the block it actually held: a lying drone cannot clear the
        # whole map with one record.
        allowed = set(self.block_cells.get(block, ()))
        named = set(cells) if isinstance(cells, list) else allowed
        self.surveyed |= (named & allowed)

    def _rtb(self, agent) -> None:
        # Battery lease: release the block so the other drone can take it,
        # and stop the returning drone claiming more until it reports `ready`.
        if agent is None:
            return
        self.grounded.add(agent)
        self.block_claims = {b: a for b, a in self.block_claims.items()
                             if a != agent}

    # ---- evidence ----
    def _hazard(self, agent, cell) -> None:
        """One witness is provisional, two distinct witnesses confirm.

        `hazard` (a drone sighting from the air) and `corroborate` (a second
        sighting, or a bot that physically stalled there) fold identically:
        what matters is how many DISTINCT agents have vouched, not which op
        carried the claim. An unhealthy agent is not a witness at all.
        """
        if agent is None or cell is None or agent in self.unhealthy:
            return
        if cell not in self.sectors:
            return
        if cell in self.explored:
            return          # already physically covered; nothing left to warn
        witnesses = self.hazard_reports.setdefault(cell, set())
        witnesses.add(agent)      # a set, so one agent shouting twice is one
        if len(witnesses) >= 2:
            self.confirmed_hazards.add(cell)
            self.unreachable.add(cell)
            # Condemned ground is not held by anyone.
            if cell in self.claimed:
                del self.claimed[cell]

    # ---- ground tier ----
    def _claim(self, agent, sector) -> None:
        ok = (self.phase == SURVEYING
              and agent is not None
              and agent not in self.unhealthy
              and sector in self.sectors
              and sector in self.surveyed          # THE cross-tier gate
              and sector not in self.explored
              and sector not in self.unreachable
              and sector not in self.claimed
              and agent not in self.claimed.values())
        if ok:
            self.claimed[sector] = agent

    def _explored(self, agent, sector) -> None:
        # Only the claim holder can credit a sector, so coverage is never
        # credited off a stale assignment.
        if self.claimed.get(sector) != agent:
            return
        self.explored.add(sector)
        self.explored_by[sector] = agent
        del self.claimed[sector]

    # ---- health, uniform across tiers ----
    def _health(self, agent, seq, ok) -> None:
        if agent is None or not isinstance(seq, int):
            return
        if seq <= self.health_seq.get(agent, -1):
            return                      # stale or duplicate beacon
        self.health_seq[agent] = seq
        if ok:
            self.unhealthy.discard(agent)
        else:
            self._mark_unhealthy(agent)

    def _suspect(self, victim, seen_seq) -> None:
        # Silence lease: acts only if the victim has not beaconed since the
        # proposer's observation, so a late beacon voids every in-flight
        # suspicion and duplicates are no-ops.
        if victim is None:
            return
        if self.health_seq.get(victim, -1) == seen_seq:
            self._mark_unhealthy(victim)

    def _mark_unhealthy(self, agent) -> None:
        self.unhealthy.add(agent)
        # An untrusted agent holds no work, in either tier.
        self.claimed = {s: a for s, a in self.claimed.items() if a != agent}
        self.block_claims = {b: a for b, a in self.block_claims.items()
                             if a != agent}

    def _recompute_phase(self) -> None:
        blocks_left = set(self.blocks) - self.surveyed_blocks
        ground_left = (set(self.sectors) - self.explored) - self.unreachable
        self.phase = DONE if not (blocks_left or ground_left) else SURVEYING

    # ---- derived state (identical on every agent) ----
    def claimable_blocks(self):
        """Blocks a healthy, airborne drone may claim right now."""
        return [b for b in self.blocks
                if b not in self.surveyed_blocks and b not in self.block_claims]

    def claimable_sectors(self):
        """Sectors a healthy, idle bot may claim right now: surveyed, not yet
        covered, not condemned, not already held."""
        return [s for s in self.sectors
                if s in self.surveyed and s not in self.explored
                and s not in self.unreachable and s not in self.claimed]

    def provisional_hazards(self):
        """Cells with exactly one witness: deferred, not condemned. A bot
        should sweep these last, which is what gives the second witness a
        chance to arrive before anyone drives in."""
        return sorted(c for c, w in self.hazard_reports.items()
                      if len(w) == 1 and c not in self.confirmed_hazards)

    def my_block(self, me):
        for b, a in self.block_claims.items():
            if a == me:
                return b
        return None

    def my_claim(self, me):
        for s, a in self.claimed.items():
            if a == me:
                return s
        return None

    def role(self, me):
        """What agent `me` should physically do now:
        ('survey', block) | ('sweep', sector) | ('done', None) | ('idle', None)
        """
        if self.phase == DONE:
            return ("done", None)
        block = self.my_block(me)
        if block is not None:
            return ("survey", block)
        sector = self.my_claim(me)
        if sector is not None:
            return ("sweep", sector)
        return ("idle", None)

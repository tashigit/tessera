"""Pure replicated state machine for consensus-coordinated ARENA EXPLORATION —
the second simulation: five Pioneer 3-AT robots sweep an outdoor arena divided
into a fixed grid of sectors, with every inter-robot decision derived from the
Vertex ordered log. (The first simulation, route exploration, lives in
../../simulation/; this one exercises a different protocol shape on the same
vertex_fleet consumer API.)

No ROS / Webots dependencies, so it is unit-testable on any host (see
test_arena_fsm.py). Given the same ordered sequence of decoded transaction
records (the Vertex consensus order on /vertex/event), every robot derives
identical global state.

Rules (see ../README.md):
  * Sector claims are exclusive: the first `claim` for a free sector in
    consensus order wins it; a bot holds at most one sector at a time and all
    claimed bots sweep concurrently.
  * The claim holder that physically covers its sector reports `explored`
    (permanent). A holder that cannot reach its sector releases it with
    `abandon` (someone else may try) or, after repeated failures, condemns it
    with `unreachable` (excluded from the remaining work).
  * Health is a fold, not a vote. Each bot periodically submits a `health`
    beacon (monotonic `seq`, self-assessed `ok` from its local sensor-stream
    freshness). An unhealthy bot loses its claim at that consensus point and
    new claims from it are ignored; a fresh ok beacon readmits it. Because
    every bot folds the same beacons in the same order, the fleet's health
    verdicts agree everywhere with no vote/tally protocol on top — the MQTT
    predecessor needed one because its transport had no agreement primitive.
  * Silence lease: a crashed bot stops beaconing. Any live bot may propose
    `suspect` carrying the victim's last folded beacon seq; it acts only if no
    newer beacon has landed, so a late beacon (the victim was alive after all)
    makes every in-flight suspicion a no-op. First in consensus order wins,
    duplicates are no-ops.
  * `detection` records (a deer, a rock worth flagging) are accepted only if
    the reporter is healthy at that point in the log — the pioneer stack's
    "consensus on robot health before accepting detections", now a pure rule.
  * DONE when every sector is explored or condemned unreachable.

Transaction payloads are opaque JSON records (all carry `epoch` for reset):
    {"op":"claim",       "bot":2, "sector":"S07", "epoch":0}
    {"op":"explored",    "bot":2, "sector":"S07", "epoch":0}
    {"op":"abandon",     "bot":2, "sector":"S07", "epoch":0}
    {"op":"unreachable", "bot":2, "sector":"S07", "epoch":0}
    {"op":"health",      "bot":2, "seq":41, "ok":true,  "epoch":0}
    {"op":"suspect",     "bot":3, "victim":2, "seen_seq":41, "epoch":0}
    {"op":"detection",   "bot":1, "seq":0, "label":"deer",
                         "x":-14.6, "y":9.6, "epoch":0}
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

__all__ = ["ArenaState", "make_grid", "encode", "decode", "EXPLORING", "DONE"]

EXPLORING, DONE = "exploring", "done"


def make_grid(nx: int, ny: int, min_x: float, min_y: float,
              cell_w: float, cell_h: float):
    """Sector ids and centres for an nx-by-ny grid laid over the arena.
    Pure geometry shared by the coordinator, the robots (mock and Webots),
    and the tests, so every process keys sectors identically.

    Returns (ids, centers): ids row-major from the south-west corner
    ("S00", "S01", ...), centers as {id: (cx, cy)}.
    """
    ids, centers = [], {}
    for iy in range(ny):
        for ix in range(nx):
            sid = f"S{iy * nx + ix:02d}"
            ids.append(sid)
            centers[sid] = (min_x + (ix + 0.5) * cell_w,
                            min_y + (iy + 0.5) * cell_h)
    return ids, centers


class ArenaState(ReplicatedState):
    """Deterministic fold of the consensus-ordered record stream. Epoch gating
    and `reset` handling come from vertex_fleet.ReplicatedState; this class
    folds the arena-exploration ops."""

    def __init__(self, sectors, num_bots=5):
        self.sectors = list(sectors)
        self.num_bots = num_bots
        super().__init__()        # sets epoch = 0 and calls wipe()

    def wipe(self) -> None:
        self.claimed: dict[str, int] = {}   # sector -> bot, exclusive
        self.explored: set[str] = set()     # permanently covered
        self.unreachable: set[str] = set()  # condemned, excluded from the work
        self.unhealthy: set[int] = set()    # current fleet health verdicts
        self.health_seq: dict[int, int] = {}  # bot -> latest folded beacon seq
        self.detections: list[dict] = []    # accepted reports, consensus order
        self.phase = EXPLORING

    # ---- fold one same-epoch record (consensus order) ----
    def apply_record(self, rec: dict) -> None:
        op = rec["op"]
        bot, sector = rec.get("bot"), rec.get("sector")
        if op == "claim":
            # Exclusive assignment, first claim in consensus order wins.
            ok = (self.phase == EXPLORING and bot is not None
                  and bot not in self.unhealthy
                  and sector in self.sectors
                  and sector not in self.explored
                  and sector not in self.unreachable
                  and sector not in self.claimed
                  and bot not in self.claimed.values())
            if ok:
                self.claimed[sector] = bot
        elif op == "explored":
            # Only the claim holder can credit a sector: a bot whose claim was
            # released (health blip, suspicion) must re-claim before reporting,
            # so coverage is never credited off a stale assignment.
            if self.claimed.get(sector) == bot:
                self.explored.add(sector)
                del self.claimed[sector]
        elif op == "abandon":
            if self.claimed.get(sector) == bot:
                del self.claimed[sector]
        elif op == "unreachable":
            # Condemnation also requires holding the claim: the reporter is at
            # the sector and has physically tried.
            if self.claimed.get(sector) == bot:
                self.unreachable.add(sector)
                del self.claimed[sector]
        elif op == "health":
            seq = rec.get("seq")
            if bot is None or not isinstance(seq, int):
                return
            if seq <= self.health_seq.get(bot, -1):
                return                      # stale or duplicate beacon
            self.health_seq[bot] = seq
            if rec.get("ok"):
                self.unhealthy.discard(bot)
            else:
                self._mark_unhealthy(bot)
        elif op == "suspect":
            # Silence lease: acts only if the victim has not beaconed since
            # the proposer's observation, so late beacons void the suspicion
            # and duplicate suspicions are no-ops.
            victim, seen = rec.get("victim"), rec.get("seen_seq")
            if victim is None:
                return
            if self.health_seq.get(victim, -1) == seen:
                self._mark_unhealthy(victim)
        elif op == "detection":
            seq = rec.get("seq")
            if bot is None or bot in self.unhealthy:
                return                      # untrusted reporter: rejected
            if any(d["bot"] == bot and d["seq"] == seq for d in self.detections):
                return                      # duplicate proposal
            self.detections.append({
                "bot": bot, "seq": seq, "label": rec.get("label"),
                "x": rec.get("x"), "y": rec.get("y"),
            })

        self._recompute_phase()

    def _mark_unhealthy(self, bot: int) -> None:
        self.unhealthy.add(bot)
        # An untrusted bot cannot hold work: release its claim so a healthy
        # bot picks the sector up.
        self.claimed = {s: b for s, b in self.claimed.items() if b != bot}

    def _recompute_phase(self) -> None:
        remaining = (set(self.sectors) - self.explored) - self.unreachable
        self.phase = DONE if not remaining else EXPLORING

    # ---- derived state (identical on every robot) ----
    def claimable_sectors(self):
        """Sectors a healthy, idle bot may claim right now."""
        return [s for s in self.sectors
                if s not in self.explored and s not in self.unreachable
                and s not in self.claimed]

    def my_claim(self, my_id: int):
        """The sector `my_id` currently holds, or None."""
        for s, b in self.claimed.items():
            if b == my_id:
                return s
        return None

    def role(self, my_id: int):
        """What robot `my_id` should physically do now:
        ('explore', sector) | ('done', None) | ('idle', None)."""
        if self.phase == DONE:
            return ("done", None)
        mine = self.my_claim(my_id)
        if mine is not None:
            return ("explore", mine)
        return ("idle", None)

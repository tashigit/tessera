#!/usr/bin/env python3
"""ground_coordinator — one per Pioneer in the air/ground simulation.

The tessera half of a mixed fleet. This node is an ordinary
`vertex_fleet.VertexAgent`: it talks to its `vertex_node` over the
`/vertex/*` contract topics and folds `/vertex/event` through
`AirGroundState`. Its peers on the mesh include two drones that do none of
that, having linked `tashi-vertex` straight into a Rust binary
(`air_agent/`), and the two tiers still derive the same shared state. That is
what the simulation is for.

What this node adds on top of the base agent:

  * the claim loop, which unlike the arena scenario's must WAIT for the air
    tier. A sector is not claimable until a drone has surveyed it, so an idle
    bot with nothing surveyed nearby simply idles, and that is correct
    behaviour rather than a stall.
  * physical outcome reports: `explored` on covering a sector centre,
    `abandon` when progress stops.
  * corroboration. A bot that stalls on a sector already flagged by a drone
    is the second, independent witness that turns a provisional hazard into a
    confirmed one. This is the ground tier's half of the evidence rule, and
    the reason a lying drone cannot hide a real pit: the bot that drives into
    it says so.
  * the health beacon and the silence lease, identical in shape to the arena
    scenario's but now able to name a drone as victim.

  subscribes
    pose        geometry_msgs/PointStamped   from the robot (GPS / mock)
    telemetry   std_msgs/String (JSON)       per-stream sensor ages
    /reset      std_msgs/Int32               supervisor reset control
  publishes
    goto        std_msgs/String              sector id | HOLD | STOP
    mission_state  std_msgs/String (JSON)    for the launch_test
"""

import json
import math
import os
import sys

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, String

from airground_fsm import (DONE, AirGroundState, decode, make_blocks,
                           make_sectors)

try:
    from vertex_fleet import VertexAgent, spin_agent
except ImportError:                                    # pragma: no cover
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "vertex_fleet"))
    from vertex_fleet import VertexAgent, spin_agent


class GroundCoordinator(VertexAgent):
    def __init__(self):
        super().__init__("ground_coordinator", state=None, tick_period_sec=0.2)
        self.declare_parameter("agent_id", "bot_0")
        self.declare_parameter("agents", ["bot_0", "bot_1", "drone_0", "drone_1"])
        # grid, mirroring airground.launch.py, the Rust agent and the fixture
        self.declare_parameter("grid_nx", 4)
        self.declare_parameter("grid_ny", 3)
        self.declare_parameter("grid_min_x", -20.0)
        self.declare_parameter("grid_min_y", -15.0)
        self.declare_parameter("cell_w", 10.0)
        self.declare_parameter("cell_h", 10.0)
        self.declare_parameter("block_w", 2)
        self.declare_parameter("block_h", 1)

        self.declare_parameter("claim_interval_sec", 1.0)
        self.declare_parameter("cover_radius", 2.0)
        # after losing a claim race, avoid re-picking that sector briefly so
        # two bots that collided diverge instead of re-colliding every cycle
        self.declare_parameter("lost_avoid_sec", 3.0)
        self.declare_parameter("health_interval_sec", 1.0)
        self.declare_parameter("stream_timeout_sec", 3.0)
        self.declare_parameter("suspect_after_sec", 15.0)
        # no forward progress toward the target for this long -> abandon, and
        # corroborate if the air tier had already flagged the sector
        self.declare_parameter("stall_sec", 20.0)
        # Physically immobilized: commanded to drive but the body has not
        # displaced for this long. A crater steep enough to be worth warning
        # about is steep enough to swallow a Pioneer, and it happens: a bot
        # drove into S05 and could not climb out. A trapped bot must stop
        # claiming, or it starves sectors it can never reach and the fleet
        # never finishes. Local gate only; health stays a sensor-stream
        # verdict, exactly as in the arena scenario.
        self.declare_parameter("immobilized_sec", 45.0)
        # After this many of my OWN failed attempts on one sector, report it as
        # impassable. Not every blocker is a crater: a tight cluster of trees
        # stops a Pioneer just as well, and the air tier sees straight over it.
        # Without this a bot claims such a sector, stalls, abandons, re-claims
        # and loops forever, which is exactly what happened on S10. Repeated
        # first-hand failure is evidence, so it goes in as `corroborate` and
        # obeys the same two-distinct-witness rule as everything else: one
        # unlucky bot defers the sector, two independent bots condemn it.
        self.declare_parameter("max_attempts", 2)

        p = lambda n: self.get_parameter(n).value
        self.me = str(p("agent_id"))
        self.agents = [str(a) for a in p("agents")]
        nx, ny = int(p("grid_nx")), int(p("grid_ny"))
        self.sectors, self.centers = make_sectors(
            nx, ny, float(p("grid_min_x")), float(p("grid_min_y")),
            float(p("cell_w")), float(p("cell_h")))
        self.blocks, self.block_cells = make_blocks(
            nx, ny, int(p("block_w")), int(p("block_h")))

        self.claim_interval = float(p("claim_interval_sec"))
        self.cover_radius = float(p("cover_radius"))
        self.lost_avoid_sec = float(p("lost_avoid_sec"))
        self.health_interval = float(p("health_interval_sec"))
        self.stream_timeout = float(p("stream_timeout_sec"))
        self.suspect_after = float(p("suspect_after_sec"))
        self.stall_sec = float(p("stall_sec"))
        self.immobilized_sec = float(p("immobilized_sec"))
        self.max_attempts = int(p("max_attempts"))

        self.state = AirGroundState(self.sectors, self.blocks,
                                    self.block_cells, self.agents)

        self.pose_x = self.pose_y = None
        self._pose_stamp = None
        self._stream_ages = None
        self._telemetry_stamp = None

        now = self._now_sec()
        self._last_claim = now
        self._health_seq = -1
        self._last_beacon = None
        self._beacon_advanced = {a: now for a in self.agents}
        self._seq_seen = {}
        self._suspect_sent = set()
        self._reset_seen = 0
        self._last_claim_sector = None
        self._recently_lost = {}
        # pursuit bookkeeping for the sector currently being driven
        self._pursuit = None
        self._pursuit_best = None
        self._pursuit_progress_at = None
        self._outcome_sent = None
        self._corroborated = set()
        self._attempts = {}            # (epoch, sector) -> my failed attempts
        self._imm_anchor = None        # (t, x, y) while driving
        self._immobilized_at = None    # (x, y) where the body got stuck
        self._last_decision = None

        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "logs")
        os.makedirs(log_dir, exist_ok=True)
        self._logf = open(os.path.join(log_dir, f"{self.me}_airground.log"),
                          "a", buffering=1)
        self._flog(f"=== ground_coordinator start (agent={self.me}, "
                   f"{len(self.sectors)} sectors, {len(self.blocks)} blocks) ===")

        self.goto_pub = self.create_publisher(String, "goto", 10)
        self.state_pub = self.create_publisher(String, "mission_state", 10)
        self.create_subscription(PointStamped, "pose", self._on_pose, 10)
        self.create_subscription(String, "telemetry", self._on_telemetry, 10)
        self.create_subscription(Int32, "/reset", self._on_reset, 10)

        self.get_logger().info(
            f"ground_coordinator up: agent={self.me} "
            f"sectors={len(self.sectors)} blocks={len(self.blocks)}")

    # ---- plumbing ----
    def _flog(self, line):
        self._logf.write(f"[{self._now_sec():.3f}] {line}\n")

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_pose(self, msg):
        self.pose_x, self.pose_y = msg.point.x, msg.point.y
        self._pose_stamp = self._now_sec()

    def _on_telemetry(self, msg):
        try:
            self._stream_ages = json.loads(msg.data)
            self._telemetry_stamp = self._now_sec()
        except ValueError:
            pass

    def _on_reset(self, msg):
        epoch = int(msg.data)
        if epoch > self._reset_seen:
            self._reset_seen = epoch
            self.propose_reset(epoch)

    # ---- consensus hooks ----
    def on_event(self, msg):
        h = bytes(msg.hash).hex()[:12]
        recs = [decode(tx.payload) for tx in msg.transactions]
        self._flog(f"EVENT #{self.events_folded} hash={h} "
                   f"records={[r for r in recs if r]}")
        s = self.state
        self._flog(
            f"STATE  surveyed_blocks={sorted(s.surveyed_blocks)} "
            f"surveyed={sorted(s.surveyed)} "
            f"claimed={dict(sorted(s.claimed.items()))} "
            f"explored={sorted(s.explored)} "
            f"unreachable={sorted(s.unreachable)} "
            f"hazards={ {c: sorted(w) for c, w in sorted(s.hazard_reports.items())} } "
            f"unhealthy={sorted(s.unhealthy)} phase={s.phase}")

        # Track when each agent's folded beacon last advanced, for the lease.
        now = self._now_sec()
        for a in self.agents:
            seq = s.health_seq.get(a)
            key = (s.epoch, a, seq)
            if seq is not None and self._seq_seen.get(a) != key:
                self._seq_seen[a] = key
                self._beacon_advanced[a] = now

    def on_state_changed(self):
        s = self.state
        self.state_pub.publish(String(data=json.dumps({
            "agent": self.me,
            "epoch": s.epoch,
            "phase": s.phase,
            "surveyed_blocks": sorted(s.surveyed_blocks),
            "surveyed": sorted(s.surveyed),
            "block_claims": dict(sorted(s.block_claims.items())),
            "claimed": dict(sorted(s.claimed.items())),
            "explored": sorted(s.explored),
            "explored_by": dict(sorted(s.explored_by.items())),
            "unreachable": sorted(s.unreachable),
            "hazard_reports": {c: sorted(w)
                               for c, w in sorted(s.hazard_reports.items())},
            "confirmed_hazards": sorted(s.confirmed_hazards),
            "grounded": sorted(s.grounded),
            "unhealthy": sorted(s.unhealthy),
            "immobilized": self._immobilized_at is not None,
            "events": self.events_folded,
        }, sort_keys=True)))

    # ---- the periodic decision loop ----
    def tick(self):
        now = self._now_sec()
        self._beacon(now)
        self._silence_lease(now)
        self._check_mobility(now)

        s = self.state
        if s.phase == DONE:
            self._drive("STOP", "mission complete")
            return

        mine = s.my_claim(self.me)

        # Resolve the outcome of my last claim attempt.
        if self._last_claim_sector is not None:
            epoch, sector = self._last_claim_sector
            if epoch == s.epoch and s.claimed.get(sector) is not None:
                if s.claimed[sector] != self.me:
                    self._recently_lost[(epoch, sector)] = now
                self._last_claim_sector = None

        if mine is None:
            self._pursuit = None
            self._drive("HOLD", None)
            if (now - self._last_claim >= self.claim_interval
                    and self._healthy() and self._immobilized_at is None):
                self._last_claim = now
                target = self._pick_sector()
                if target is not None:
                    self._last_claim_sector = (s.epoch, target)
                    self.propose({"op": "claim", "agent": self.me,
                                  "sector": target, "epoch": s.epoch})
                    self._flog(f"TX     claim {target}")
                else:
                    # Nothing claimable is the normal state early on: the air
                    # tier has not surveyed anything yet. Say so once rather
                    # than looking like a stall.
                    self._decide_once(
                        "waiting on the air tier: no surveyed sector free")
            return

        # Holding a sector: drive to it and report the physical outcome.
        self._pursue(mine, now)

    def _pursue(self, sector, now):
        s = self.state
        if self._pursuit != (s.epoch, sector):
            self._pursuit = (s.epoch, sector)
            self._pursuit_best = None
            self._pursuit_progress_at = now
            self._outcome_sent = None
            self._flog(f"DECIDE pursuing {sector}")

        self._drive(sector, None)
        if self.pose_x is None:
            return

        cx, cy = self.centers[sector]
        dist = math.hypot(self.pose_x - cx, self.pose_y - cy)

        if dist <= self.cover_radius:
            if self._outcome_sent != "explored":
                self._outcome_sent = "explored"
                self.propose({"op": "explored", "agent": self.me,
                              "sector": sector, "epoch": s.epoch})
                self._flog(f"TX     explored {sector}")
            return

        if self._pursuit_best is None or dist < self._pursuit_best - 0.25:
            self._pursuit_best = dist
            self._pursuit_progress_at = now
            return

        if now - self._pursuit_progress_at < self.stall_sec:
            return

        # Stalled. If the air tier already flagged this cell, my stall is the
        # independent second witness that confirms it: the drone saw a hole
        # from above, I could not get through it from the ground. If nothing
        # was flagged, just hand the sector back.
        if self._outcome_sent is not None:
            return
        flagged = sector in s.hazard_reports and self.me not in s.hazard_reports[sector]
        self._outcome_sent = "stalled"
        key = (s.epoch, sector)
        self._attempts[key] = self._attempts.get(key, 0) + 1
        self.propose({"op": "abandon", "agent": self.me,
                      "sector": sector, "epoch": s.epoch})
        self._flog(f"TX     abandon {sector} (no progress for {self.stall_sec}s, "
                   f"attempt {self._attempts[key]})")
        # Two ways to become a witness that this ground is impassable: the air
        # tier already flagged it and I have just failed to cross it, or I have
        # failed on it enough times by myself to be sure.
        if key in self._corroborated:
            return
        reason = None
        if flagged:
            reason = "air flagged it, I cannot pass"
        elif self._attempts[key] >= self.max_attempts:
            reason = f"{self._attempts[key]} failed attempts of my own"
        if reason:
            self._corroborated.add(key)
            self.propose({"op": "corroborate", "agent": self.me,
                          "cell": sector, "epoch": s.epoch})
            self._flog(f"TX     corroborate {sector} ({reason})")

    def _check_mobility(self, now):
        """Notice when the body has stopped moving despite being driven.

        Separate from the stall check: a stall is about progress toward a
        target and produces `abandon`, this is about the robot being stuck at
        all and produces a local refusal to take more work.
        """
        if self.pose_x is None:
            return
        if self._immobilized_at is not None:
            ix, iy = self._immobilized_at
            if math.hypot(self.pose_x - ix, self.pose_y - iy) > 1.0:
                self._immobilized_at = None      # climbed out: resume claiming
                self._imm_anchor = None
                self._decide_once("mobile again, resuming claims")
            return
        if self._pursuit is None:                # not being driven anywhere
            self._imm_anchor = None
            return
        if self._imm_anchor is None:
            self._imm_anchor = (now, self.pose_x, self.pose_y)
            return
        t0, ax, ay = self._imm_anchor
        if math.hypot(self.pose_x - ax, self.pose_y - ay) > 0.5:
            self._imm_anchor = (now, self.pose_x, self.pose_y)
        elif now - t0 > self.immobilized_sec:
            self._immobilized_at = (self.pose_x, self.pose_y)
            self._flog(f"DECIDE IMMOBILIZED at ({self.pose_x:.1f}, "
                       f"{self.pose_y:.1f}) — no longer claiming")
            self._witness_where_i_am()

    def _witness_where_i_am(self):
        """Being stuck is evidence about WHERE I AM, not where I was going.

        The stall path in _pursue only reports the sector it was pursuing, so a
        bot that drives into a crater while merely crossing it says nothing —
        which happened: bot_0 fell into S05 en route to S10, went immobilized,
        and the drone's warning about S05 stayed on one witness forever. A body
        that cannot move is a first-hand observation about the ground beneath
        it, so report that sector, whether or not anyone flagged it first. The
        fold folds `corroborate` and `hazard` identically and counts distinct
        agents, so this is a second witness when a drone already saw it and a
        first witness when nobody did.
        """
        sector = self._sector_at(self.pose_x, self.pose_y)
        if sector is None:
            return
        key = (self.state.epoch, sector)
        if key in self._corroborated:
            return
        self._corroborated.add(key)
        self.propose({"op": "corroborate", "agent": self.me,
                      "cell": sector, "epoch": self.state.epoch})
        self._flog(f"TX     corroborate {sector} (immobilized in it)")

    def _sector_at(self, x, y):
        """Which sector a world position falls in, or None if outside."""
        if x is None or y is None:
            return None
        p = lambda n: self.get_parameter(n).value
        nx, ny = int(p("grid_nx")), int(p("grid_ny"))
        ix = int(math.floor((x - float(p("grid_min_x"))) / float(p("cell_w"))))
        iy = int(math.floor((y - float(p("grid_min_y"))) / float(p("cell_h"))))
        if ix < 0 or iy < 0 or ix >= nx or iy >= ny:
            return None
        return f"S{iy * nx + ix:02d}"

    def _pick_sector(self):
        """Nearest claimable sector, deferring cells the air tier flagged.

        A provisional hazard is not a veto, it is a preference: sweep the
        clear ground first and give a second witness time to arrive before
        anyone drives into a suspected crater. If only flagged sectors remain,
        one gets attempted, and the stall that follows supplies the
        corroboration.
        """
        s = self.state
        now = self._now_sec()
        avoid = {sec for (ep, sec), t in self._recently_lost.items()
                 if ep == s.epoch and now - t < self.lost_avoid_sec}
        options = [c for c in s.claimable_sectors() if c not in avoid]
        if not options:
            options = s.claimable_sectors()
        if not options:
            return None
        flagged = set(s.provisional_hazards())

        def rank(sec):
            cx, cy = self.centers[sec]
            d = (math.hypot(self.pose_x - cx, self.pose_y - cy)
                 if self.pose_x is not None else 0.0)
            return (sec in flagged, d, sec)

        return min(options, key=rank)

    # ---- health ----
    def _healthy(self):
        return self.me not in self.state.unhealthy and self._self_ok()

    def _self_ok(self):
        """Self-assessed from sensor-stream freshness, the same rule the air
        tier applies to its own telemetry age."""
        now = self._now_sec()
        if self._telemetry_stamp is None or self._pose_stamp is None:
            return False
        if now - self._telemetry_stamp > self.stream_timeout:
            return False
        if now - self._pose_stamp > self.stream_timeout:
            return False
        ages = self._stream_ages or {}
        return all(float(v) <= self.stream_timeout for v in ages.values())

    def _beacon(self, now):
        if self._last_beacon is not None and now - self._last_beacon < self.health_interval:
            return
        self._last_beacon = now
        self._health_seq += 1
        self.propose({"op": "health", "agent": self.me, "seq": self._health_seq,
                      "ok": self._self_ok(), "epoch": self.state.epoch})

    def _silence_lease(self, now):
        """Propose `suspect` for any peer whose beacons stopped advancing.

        Uniform across tiers: the victim may just as well be a drone whose
        Rust process died as a bot whose node crashed. The fold does not
        distinguish them and neither does this.
        """
        s = self.state
        for a in self.agents:
            if a == self.me or a in s.unhealthy:
                continue
            if now - self._beacon_advanced.get(a, now) < self.suspect_after:
                continue
            seen = s.health_seq.get(a, -1)
            key = (s.epoch, a, seen)
            if key in self._suspect_sent:
                continue
            self._suspect_sent.add(key)
            self.propose({"op": "suspect", "agent": self.me, "victim": a,
                          "seen_seq": seen, "epoch": s.epoch})
            self._flog(f"TX     suspect {a} (silent since seq {seen})")

    # ---- output ----
    def _drive(self, target, reason):
        self.goto_pub.publish(String(data=target))
        if reason:
            self._decide_once(reason)

    def _decide_once(self, reason):
        if reason != self._last_decision:
            self._last_decision = reason
            self._flog(f"DECIDE {reason}")


def main():
    spin_agent(GroundCoordinator)


if __name__ == "__main__":
    main()

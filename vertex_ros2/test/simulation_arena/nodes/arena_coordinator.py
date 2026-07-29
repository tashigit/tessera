#!/usr/bin/env python3
"""arena_coordinator — one per robot; the arena-exploration logic on top of
the vertex_fleet consumer API (see arena_fsm.py and ../README.md).

The second simulation: five Pioneer 3-AT robots sweep an arena divided into a
fixed sector grid. Everything the MQTT predecessor routed through a broker
(sector coordination, robot-health voting, detection acceptance) is here a
deterministic fold of the Vertex ordered log, built on vertex_fleet.VertexAgent
(engine lifecycle bring-up, single-mutation-path fold, epoch-stamped
proposals). This node adds what is scenario-specific:

  subscribes
    pose        geometry_msgs/PointStamped   from the robot (GPS / mock)
    telemetry   std_msgs/String (JSON)       per-stream sensor ages
    detection   std_msgs/String (JSON)       sighted objects, robot-side id
    /reset      std_msgs/Int32               supervisor reset control
  publishes
    goto        std_msgs/String              sector id | HOLD | STOP
    mission_state  std_msgs/String (JSON)    for the launch_test

The claim loop (nearest free sector), the physical outcome reports
(explored / abandon / unreachable), the health beacon (self-assessed from
sensor-stream freshness), the silence lease (suspect a bot whose beacons
stopped), and the per-node consensus log that verify_consensus_logs.py diffs
across bots. Physical obstacle avoidance is the robot's job; consensus owns
sector exclusivity and the fleet's health verdicts.
"""

import json
import os
import sys

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, String

from arena_fsm import DONE, EXPLORING, ArenaState, decode, make_grid

try:
    from vertex_fleet import VertexAgent, spin_agent
except ImportError:                                    # pragma: no cover
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "vertex_fleet"))
    from vertex_fleet import VertexAgent, spin_agent


class ArenaCoordinator(VertexAgent):
    def __init__(self):
        # State construction needs ROS parameters, so it is attached right
        # after the node exists and before spinning (see VertexAgent docs).
        super().__init__("arena_coordinator", state=None, tick_period_sec=0.2)
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("num_bots", 5)
        # sector grid over the arena (mirrors worlds/pioneer_arena.wbt)
        self.declare_parameter("grid_nx", 5)
        self.declare_parameter("grid_ny", 4)
        self.declare_parameter("grid_min_x", -20.0)
        self.declare_parameter("grid_min_y", -15.0)
        self.declare_parameter("cell_w", 8.0)
        self.declare_parameter("cell_h", 7.5)
        self.declare_parameter("claim_interval_sec", 1.0)
        self.declare_parameter("cover_radius", 1.5)
        # after losing a claim race on a sector, avoid re-picking it for this
        # long, so two bots that collide on the same nearest sector diverge
        # within a cycle or two instead of re-colliding every interval
        self.declare_parameter("lost_avoid_sec", 3.0)
        # health: beacon period, and how stale a sensor stream (or the
        # telemetry feed itself) may be before this bot self-reports not-ok
        self.declare_parameter("health_interval_sec", 1.0)
        self.declare_parameter("stream_timeout_sec", 3.0)
        # silence lease: propose `suspect` for a bot whose beacons stopped
        # advancing for this long (crashed robot; see arena_fsm.py)
        self.declare_parameter("suspect_after_sec", 15.0)
        # no forward progress toward the target for this long -> abandon;
        # after max_attempts own abandons of one sector -> unreachable
        self.declare_parameter("stall_sec", 25.0)
        self.declare_parameter("max_attempts", 2)
        # physically immobilized (commanded to drive but the body has not
        # displaced for this long, e.g. trapped in a pit): stop claiming, so a
        # trapped robot cannot starve sectors it can never reach. Local gate
        # only — health stays a sensor-stream verdict, as in the predecessor.
        self.declare_parameter("immobilized_sec", 90.0)

        p = lambda n: self.get_parameter(n).value
        self.my_id = int(p("robot_id"))
        self.num_bots = int(p("num_bots"))
        self.sectors, self.centers = make_grid(
            int(p("grid_nx")), int(p("grid_ny")),
            float(p("grid_min_x")), float(p("grid_min_y")),
            float(p("cell_w")), float(p("cell_h")))
        self.claim_interval = float(p("claim_interval_sec"))
        self.cover_radius = float(p("cover_radius"))
        self.lost_avoid_sec = float(p("lost_avoid_sec"))
        self.health_interval = float(p("health_interval_sec"))
        self.stream_timeout = float(p("stream_timeout_sec"))
        self.suspect_after = float(p("suspect_after_sec"))
        self.stall_sec = float(p("stall_sec"))
        self.max_attempts = int(p("max_attempts"))
        self.immobilized_sec = float(p("immobilized_sec"))

        self.state = ArenaState(self.sectors, num_bots=self.num_bots)
        self.pose_x = None
        self.pose_y = None
        self._pose_stamp = None           # pose freshness (odom stream parity)
        self._stream_ages = None          # latest telemetry payload
        self._telemetry_stamp = None      # when it arrived (local clock)
        self._pending_detections = []     # robot detections awaiting proposal
        self._inflight_detections = {}    # (epoch, id) -> det dict, sent, unconfirmed
        self._confirmed_detections = set()  # (epoch, id) landed in state.detections

        now = self.get_clock().now()
        self._last_claim = now
        self._health_seq = -1             # my beacon counter (monotonic)
        self._last_beacon = None
        self._last_ok = None              # my latest self-assessment
        # silence lease bookkeeping: when each bot's folded beacon seq last
        # advanced (startup counts as an advance: a grace period, not a verdict)
        self._beacon_advanced = {b: now for b in range(self.num_bots)}
        self._seq_seen = {}               # bot -> (epoch, bot, seq) last noted
        self._suspect_sent = set()        # (epoch, victim, seen_seq) proposed
        self._reset_seen = 0
        # claim-loop bookkeeping: which sector my most recent claim attempt
        # targeted (pending resolution), and sectors I recently lost a claim
        # race on (temporarily avoided by _pick_sector, see lost_avoid_sec)
        self._last_claim_sector = None    # (epoch, sector), or None if resolved
        self._recently_lost = {}          # (epoch, sector) -> sim time I lost it
        # physical-outcome bookkeeping for the sector I am pursuing
        self._pursuit = None              # (epoch, sector) currently driven
        self._pursuit_best = None         # closest distance achieved so far
        self._pursuit_progress_at = None  # when that distance last improved
        self._outcome_sent = None         # pursuit already reported
        self._attempts = {}               # (epoch, sector) -> my abandons
        self._imm_anchor = None           # (Time, x, y) while exploring
        self._immobilized_at = None       # (x, y) where the body got trapped
        self._last_decision = None

        # Per-node consensus log, host-visible via the bind mount: every
        # delivered event (with hash), every submitted tx, every decision —
        # so the five files can be diffed to SHOW the nodes agree.
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "logs")
        os.makedirs(log_dir, exist_ok=True)
        self._logf = open(os.path.join(log_dir, f"robot_{self.my_id}_arena.log"),
                          "a", buffering=1)
        self._flog(f"=== arena_coordinator start (robot_id={self.my_id}, "
                   f"{len(self.sectors)} sectors) ===")

        self.goto_pub = self.create_publisher(String, "goto", 10)
        self.state_pub = self.create_publisher(String, "mission_state", 10)

        self.create_subscription(PointStamped, "pose", self._on_pose, 10)
        self.create_subscription(String, "telemetry", self._on_telemetry, 10)
        self.create_subscription(String, "detection", self._on_detection, 10)
        self.create_subscription(Int32, "/reset", self._on_reset, 10)

        self.get_logger().info(
            f"arena_coordinator up: robot_id={self.my_id} "
            f"sectors={len(self.sectors)}")

    def _flog(self, line: str):
        t = self.get_clock().now().nanoseconds / 1e9
        self._logf.write(f"[{t:.3f}] {line}\n")

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---- consensus hooks (folding is done by VertexAgent) ----
    def on_event(self, msg):
        h = bytes(msg.hash).hex()[:12]
        recs = [decode(tx.payload) for tx in msg.transactions]
        self._flog(f"EVENT #{self.events_folded} hash={h} "
                   f"records={[r for r in recs if r]}")
        self._flog(f"STATE  claimed={dict(sorted(self.state.claimed.items()))} "
                   f"explored={sorted(self.state.explored)} "
                   f"unreachable={sorted(self.state.unreachable)} "
                   f"unhealthy={sorted(self.state.unhealthy)} "
                   f"detections={len(self.state.detections)} "
                   f"phase={self.state.phase}")
        self._check_my_detections(recs)

    def _check_my_detections(self, recs):
        """Resolve any of my own detection proposals just processed by the
        fold: confirm if they landed in state.detections, or requeue them for
        retry if the fold rejected them (e.g. I was unhealthy at that exact
        consensus point). Without this, a real sighting can be silently and
        permanently lost purely because of ordering luck between a health
        flip and the detection tx (see README, "consensus on robot health
        before accepting detections")."""
        for r in recs:
            if not r or r.get("op") != "detection" or r.get("bot") != self.my_id:
                continue
            seq = r.get("seq")
            key = (self.state.epoch, seq)
            if key in self._confirmed_detections:
                continue
            det = self._inflight_detections.pop(key, None)
            if det is None:
                continue           # not one of mine currently in flight
            landed = any(d["bot"] == self.my_id and d["seq"] == seq
                         for d in self.state.detections)
            if landed:
                self._confirmed_detections.add(key)
            else:
                self._flog(f"DETECTION-REJECTED seq={seq} "
                           f"(unhealthy at fold time) -> requeued")
                self._pending_detections.append(det)

    def on_state_changed(self):
        # silence-lease bookkeeping: note every folded beacon advance
        now = self.get_clock().now()
        for b, seq in self.state.health_seq.items():
            key = (self.state.epoch, b, seq)
            if self._seq_seen.get(b) != key:
                self._seq_seen[b] = key
                self._beacon_advanced[b] = now
        self._check_claim_outcome()
        self._react()

    def _check_claim_outcome(self):
        """Resolve my most recent claim attempt, if still pending: if someone
        else ended up holding (or already resolved) the sector I targeted, I
        lost that race — remember it briefly so _pick_sector diverges to a
        different sector instead of re-colliding with the same peer every
        claim_interval_sec."""
        pending = self._last_claim_sector
        if pending is None:
            return
        epoch, sector = pending
        if epoch != self.state.epoch:
            self._last_claim_sector = None   # epoch moved on (reset)
            return
        holder = self.state.claimed.get(sector)
        if holder == self.my_id:
            self._last_claim_sector = None
            return
        if holder is not None or sector in self.state.explored \
                or sector in self.state.unreachable:
            self._recently_lost[(epoch, sector)] = self._now_sec()
            self._flog(f"CLAIM-LOST sector={sector} to bot={holder}")
            self._last_claim_sector = None

    # ---- inputs ----
    def _on_pose(self, msg: PointStamped):
        self.pose_x = msg.point.x
        self.pose_y = msg.point.y
        self._pose_stamp = self.get_clock().now()

    def _on_telemetry(self, msg: String):
        try:
            ages = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(ages, dict):
            self._stream_ages = ages
            self._telemetry_stamp = self.get_clock().now()

    def _on_detection(self, msg: String):
        try:
            det = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(det, dict) and "id" in det:
            self._pending_detections.append(det)

    def _on_reset(self, msg: Int32):
        e = int(msg.data)
        if e > self._reset_seen:
            self._reset_seen = e
            self.propose_reset(e)
            self._flog(f"TX     reset epoch {e}")

    # ---- emit a transaction (epoch-stamped by VertexAgent.propose) ----
    def _emit(self, record: dict):
        stamped = {**record, "epoch": self.state.epoch}
        self.propose(record)
        self.get_logger().info(f"tx -> {stamped}")
        self._flog(f"TX     {stamped}")

    # ---- my own health, from local sensor-stream freshness ----
    def _self_ok(self) -> bool:
        """Fresh pose (the odom stream), fresh telemetry feed, and every
        reported sensor age under the timeout — the same per-stream check the
        MQTT predecessor voted on (odom / camera / scan; its cmd_vel stream
        has no robot-side equivalent here, goto flows the other way)."""
        if self._stream_ages is None or self._telemetry_stamp is None \
                or self._pose_stamp is None:
            return False                   # not provably ok yet
        now = self.get_clock().now()
        if (now - self._pose_stamp).nanoseconds / 1e9 > self.stream_timeout:
            return False                   # pose stream went silent
        if (now - self._telemetry_stamp).nanoseconds / 1e9 > self.stream_timeout:
            return False                   # the feed itself went silent
        return all(isinstance(a, (int, float)) and a < self.stream_timeout
                   for a in self._stream_ages.values())

    # ---- periodic (engine is Active) ----
    def tick(self):
        now = self.get_clock().now()

        # 1. Health beacon: monotonically sequenced self-assessment. This is
        #    the whole "telemetry consensus" of the MQTT predecessor — every
        #    peer folds the same beacons in the same order, so the verdicts
        #    agree everywhere with no vote/tally round-trips.
        if self._last_beacon is None or \
                (now - self._last_beacon).nanoseconds / 1e9 >= self.health_interval:
            self._last_beacon = now
            self._health_seq += 1
            self._last_ok = self._self_ok()
            self._emit({"op": "health", "bot": self.my_id,
                        "seq": self._health_seq, "ok": self._last_ok})

        # 2. Silence lease: a crashed bot stops beaconing entirely (its own
        #    not-ok path never runs). Propose `suspect` with the last folded
        #    seq; the fold ignores it if a newer beacon lands first.
        for b in range(self.num_bots):
            if b == self.my_id or b in self.state.unhealthy:
                continue
            since = self._beacon_advanced.get(b, now)
            if (now - since).nanoseconds / 1e9 < self.suspect_after:
                continue
            seen = self.state.health_seq.get(b, -1)
            key = (self.state.epoch, b, seen)
            if key in self._suspect_sent:
                continue
            self._suspect_sent.add(key)
            self._emit({"op": "suspect", "bot": self.my_id,
                        "victim": b, "seen_seq": seen})

        # 3. Relay the robot's detections into consensus (the fold accepts
        #    them only while this bot is healthy at fold time — if a health
        #    flip races a detection, _check_my_detections requeues it here
        #    instead of losing it silently).
        while self._pending_detections:
            det = self._pending_detections.pop(0)
            key = (self.state.epoch, int(det["id"]))
            if key in self._confirmed_detections or key in self._inflight_detections:
                continue
            self._inflight_detections[key] = det
            self._emit({"op": "detection", "bot": self.my_id,
                        "seq": int(det["id"]), "label": det.get("label"),
                        "x": det.get("x"), "y": det.get("y")})

        # 3b. Immobilized gate: commanded to drive but the body is not
        #     displacing (trapped in a pit, high-centred on a rock). Without
        #     this, a trapped robot claims its nearest sector, stalls it for
        #     stall_sec, abandons, and immediately re-claims it — starving
        #     that sector forever. Trapped robots stop claiming; the flag
        #     clears by itself if the body ever moves again.
        role, target = self.state.role(self.my_id)
        if self.pose_x is not None:
            if self._immobilized_at is not None:
                ix, iy = self._immobilized_at
                if ((self.pose_x - ix) ** 2 + (self.pose_y - iy) ** 2) ** 0.5 > 0.8:
                    self._immobilized_at = None       # freed: resume claiming
                    self._flog("IMMOBILIZED cleared: body moved again")
            elif role == "explore":
                if self._imm_anchor is None or \
                        ((self.pose_x - self._imm_anchor[1]) ** 2
                         + (self.pose_y - self._imm_anchor[2]) ** 2) ** 0.5 > 0.5:
                    self._imm_anchor = (now, self.pose_x, self.pose_y)
                elif (now - self._imm_anchor[0]).nanoseconds / 1e9 \
                        > self.immobilized_sec:
                    self._immobilized_at = (self.pose_x, self.pose_y)
                    self._imm_anchor = None
                    self._flog(f"IMMOBILIZED at ({self.pose_x:.1f}, "
                               f"{self.pose_y:.1f}): claiming suspended")
            else:
                self._imm_anchor = None   # idle robots hold still by design

        # 4. Physical outcome for the sector I hold: explored when covered,
        #    abandon on a progress stall, unreachable after repeated failures.
        key_now = (self.state.epoch, target)
        if role == "explore" and self._pursuit != key_now:
            self._pursuit = key_now
            self._pursuit_best = None
            self._pursuit_progress_at = now
            self._outcome_sent = None
        if role == "explore" and self.pose_x is not None:
            cx, cy = self.centers[target]
            dist = ((self.pose_x - cx) ** 2 + (self.pose_y - cy) ** 2) ** 0.5
            if self._pursuit_best is None or dist < self._pursuit_best - 0.2:
                self._pursuit_best = dist
                self._pursuit_progress_at = now
            if self._outcome_sent != key_now:
                stalled = (now - self._pursuit_progress_at).nanoseconds / 1e9 \
                    > self.stall_sec
                if dist < self.cover_radius:
                    self._emit({"op": "explored", "bot": self.my_id,
                                "sector": target})
                    self._outcome_sent = key_now
                elif stalled:
                    n = self._attempts.get(key_now, 0) + 1
                    self._attempts[key_now] = n
                    op = "unreachable" if n >= self.max_attempts else "abandon"
                    self._emit({"op": op, "bot": self.my_id, "sector": target})
                    self._outcome_sent = key_now

        # 5. Claim loop: healthy and idle -> claim the nearest free sector.
        #    Everyone claims concurrently; consensus order arbitrates, and a
        #    loser's next pick avoids the sector it just lost (see
        #    _pick_sector / _check_claim_outcome) so two colliding bots
        #    diverge instead of re-colliding every interval.
        if role == "idle" and self._last_ok \
                and self._immobilized_at is None \
                and self.my_id not in self.state.unhealthy:
            if (now - self._last_claim).nanoseconds / 1e9 >= self.claim_interval:
                sector = self._pick_sector()
                if sector is not None:
                    self._last_claim = now
                    self._last_claim_sector = (self.state.epoch, sector)
                    self._emit({"op": "claim", "bot": self.my_id,
                                "sector": sector})

        self._react()

    def _pick_sector(self):
        """Nearest claimable sector centre, avoiding sectors I recently lost a
        claim race on for lost_avoid_sec (so two bots that collide on the
        same nearest sector diverge within a cycle or two instead of
        re-colliding every claim_interval_sec); deterministic tie-break on
        id. Falls back to the full candidate set if avoiding recent losses
        would leave nothing to claim."""
        avail = self.state.claimable_sectors()
        if not avail:
            return None
        now = self._now_sec()
        epoch = self.state.epoch
        self._recently_lost = {k: t for k, t in self._recently_lost.items()
                               if now - t < self.lost_avoid_sec}
        candidates = [s for s in avail if (epoch, s) not in self._recently_lost]
        if not candidates:
            candidates = avail
        if self.pose_x is None:
            return candidates[0]
        return min(candidates, key=lambda s: (
            (self.pose_x - self.centers[s][0]) ** 2
            + (self.pose_y - self.centers[s][1]) ** 2, s))

    # ---- translate the FSM role into a drive command + publish state ----
    def _react(self):
        role, target = self.state.role(self.my_id)
        if role == "explore":
            goto = target
        elif role == "done":
            goto = "STOP"
        else:
            goto = "HOLD"

        decision = (role, target, goto)
        if decision != self._last_decision:
            self._last_decision = decision
            self._flog(f"DECIDE role={role} target={target} goto={goto}")

        # publish every call (heartbeat) — a rosbridge robot can subscribe
        # late and miss a one-shot publish over the WebSocket bridge.
        self.goto_pub.publish(String(data=goto))

        self.state_pub.publish(String(data=json.dumps({
            "robot_id": self.my_id,
            "epoch": self.state.epoch,
            "claimed": dict(sorted(self.state.claimed.items())),
            "explored": sorted(self.state.explored),
            "unreachable": sorted(self.state.unreachable),
            "unhealthy": sorted(self.state.unhealthy),
            "detections": [[d["bot"], d["seq"], d["label"]]
                           for d in self.state.detections],
            "phase": self.state.phase,
            "role": role,
            "target": target,
            "self_ok": bool(self._last_ok),
            "immobilized": self._immobilized_at is not None,
        }, sort_keys=True)))


def main(args=None):
    spin_agent(ArenaCoordinator)


if __name__ == "__main__":
    main()

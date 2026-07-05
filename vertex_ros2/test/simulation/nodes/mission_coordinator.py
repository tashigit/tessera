#!/usr/bin/env python3
"""mission_coordinator — one per robot; the mission logic on top of the
vertex_fleet consumer API (PARALLEL model: all bots move concurrently,
consensus assigns routes exclusively; see mission_fsm.py and worlds/README.md).

Built on vertex_fleet.VertexAgent, which provides the engine lifecycle
bring-up, the single-mutation-path fold of /vertex/event into MissionState,
and epoch-stamped proposals. This node adds what is mission-specific:

  subscribes
    pose               geometry_msgs/PointStamped     from waypoint_follower (GPS)
    barrier            std_msgs/Bool                  in-lane progress stalled?
    /reset             std_msgs/Int32                 Supervisor reset control
    /world_changed     std_msgs/Int32                 Supervisor barrier changes
  publishes
    drive              std_msgs/String                route id | STAGING | STOP
    mission_state      std_msgs/String (JSON)         for the launch_test

Physical triggers (arrival on the claimed route's row / barrier stall), the
claim loop (home lane first, retry when everything is blocked), the lease
timeout for dead explorers, and the per-node consensus log that
verify_consensus_logs.py diffs across bots. Physical collision safety is the
follower's job; consensus only guarantees route exclusivity.
"""

import json
import os
import random
import sys

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Int32, String

from mission_fsm import EXPLORING, MissionState, decode

try:
    from vertex_fleet import VertexAgent, spin_agent
except ImportError:                                    # pragma: no cover
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", "vertex_fleet"))
    from vertex_fleet import VertexAgent, spin_agent


class MissionCoordinator(VertexAgent):
    def __init__(self):
        # State construction needs ROS parameters, so it is attached right
        # after the node exists and before spinning (see VertexAgent docs).
        super().__init__("mission_coordinator", state=None, tick_period_sec=0.2)
        self.declare_parameter("robot_id", 0)
        self.declare_parameter("routes", ["R1", "R2", "R3", "R4"])
        # lane row per route, same order as `routes` (mirrors config/routes.yaml);
        # arrival requires being ON the claimed route's row, not just past goal_x
        self.declare_parameter("route_lane_y", [2.25, 0.75, -0.75, -2.25])
        self.declare_parameter("goal_x", 3.6)
        self.declare_parameter("num_bots", 4)
        self.declare_parameter("claim_interval_sec", 1.5)
        # after this long with nothing claimable, re-claim a blocked route
        # (retry=true) so the fleet recovers when the user reopens a route
        self.declare_parameter("retry_after_sec", 6.0)
        # rule: the elected bot picks its route; random tie-break by default,
        # deterministic (home lane, then lowest) for the launch_test
        self.declare_parameter("random_routes", True)
        # lease (fault tolerance): if another bot's assignment produces no
        # blocked/arrived outcome for this long, propose a consensus `timeout`
        # that releases its route (first timeout in consensus order wins,
        # duplicates are no-ops). Must exceed the worst-case probe time.
        self.declare_parameter("lease_sec", 45.0)

        self.my_id = int(self.get_parameter("robot_id").value)
        self.routes = list(self.get_parameter("routes").value)
        self.lane_y = dict(zip(self.routes,
                               list(self.get_parameter("route_lane_y").value)))
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.num_bots = int(self.get_parameter("num_bots").value)
        self.claim_interval = float(self.get_parameter("claim_interval_sec").value)
        self.retry_after = float(self.get_parameter("retry_after_sec").value)
        self.random_routes = bool(self.get_parameter("random_routes").value)
        self.lease_sec = float(self.get_parameter("lease_sec").value)
        # home lane: bot i <-> routes[i] (straight-out departure, no lane change)
        self.home_route = self.routes[self.my_id] if self.my_id < len(self.routes) else None

        self.state = MissionState(self.routes, num_bots=self.num_bots)
        self.pose_x = -4.0
        self.pose_y = 0.0
        self.barrier_ahead = False

        now = self.get_clock().now()
        self._last_claim = now
        self._unclaimable_since = None   # when claimable_routes() went empty
        self._reported_for = None        # (epoch, route) already reported
        self._reset_seen = 0
        self._world_seen = 0             # last /world_changed seq relayed
        self._winner_seen = None         # when this node learned the winner
        self._last_decision = None       # last logged (role, target) pair
        self._assign_seen = {}           # (bot, route) -> Time it entered the log
        self._timeout_sent = set()       # (epoch, bot, route) timeouts I proposed

        # Per-node consensus log: one file per bot, visible on the host via the
        # bind mount. Records every delivered consensus event (with its hash),
        # every transaction submitted, and every decision taken — so the four
        # files can be diffed to SHOW that all vertex nodes agree.
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "logs")
        os.makedirs(log_dir, exist_ok=True)
        self._logf = open(os.path.join(log_dir, f"robot_{self.my_id}_consensus.log"),
                          "a", buffering=1)
        self._flog(f"=== mission_coordinator start (robot_id={self.my_id}) ===")

        self.drive_pub = self.create_publisher(String, "drive", 10)
        self.state_pub = self.create_publisher(String, "mission_state", 10)

        self.create_subscription(PointStamped, "pose", self._on_pose, 10)
        self.create_subscription(Bool, "barrier", self._on_barrier, 10)
        self.create_subscription(Int32, "/reset", self._on_reset, 10)
        # Supervisor broadcast: the user changed the barriers -> all recorded
        # blocks are stale. Relayed into Vertex as `unblock_all` so the whole
        # fleet re-opens exploration at the same consensus point.
        self.create_subscription(Int32, "/world_changed", self._on_world_changed, 10)

        self.get_logger().info(
            f"mission_coordinator up: robot_id={self.my_id} routes={self.routes} "
            f"home={self.home_route}")

    def _flog(self, line: str):
        """Append to this node's consensus log file (host-visible)."""
        t = self.get_clock().now().nanoseconds / 1e9
        self._logf.write(f"[{t:.3f}] {line}\n")

    # ---- consensus hooks (folding is done by VertexAgent) ----
    def on_event(self, msg):
        h = bytes(msg.hash).hex()[:12]
        recs = [decode(tx.payload) for tx in msg.transactions]
        self._flog(f"EVENT #{self.events_folded} hash={h} "
                   f"records={[r for r in recs if r]}")
        self._flog(f"STATE  blocked={sorted(self.state.blocked)} "
                   f"assigned={dict(sorted(self.state.assigned.items()))} "
                   f"arrived={sorted(self.state.arrived)} "
                   f"winner={self.state.winner_route} phase={self.state.phase}")

    def on_state_changed(self):
        if self.state.phase == "converging" and self._winner_seen is None:
            self._winner_seen = self.get_clock().now()
        elif self.state.phase != "converging":
            self._winner_seen = None
        # lease bookkeeping: start a clock when an assignment enters the log,
        # drop it when the assignment resolves (blocked/arrived/timeout/reset)
        now = self.get_clock().now()
        cur = set(self.state.assigned.items())
        for pair in cur - set(self._assign_seen):
            self._assign_seen[pair] = now
        for pair in set(self._assign_seen) - cur:
            del self._assign_seen[pair]
        self._react()

    # ---- inputs ----
    def _on_pose(self, msg: PointStamped):
        self.pose_x = msg.point.x
        self.pose_y = msg.point.y

    def _on_barrier(self, msg: Bool):
        self.barrier_ahead = msg.data

    # ---- emit a transaction (epoch-stamped by VertexAgent.propose) ----
    def _emit(self, record: dict):
        stamped = {**record, "epoch": self.state.epoch}
        self.propose(record)
        self.get_logger().info(f"tx -> {stamped}")
        self._flog(f"TX     {stamped}")

    # ---- reset control: relay the Supervisor's reset into the ordered log ----
    def _on_reset(self, msg: Int32):
        e = int(msg.data)
        if e > self._reset_seen:
            self._reset_seen = e
            self.propose_reset(e)
            self.get_logger().info(f"tx -> reset epoch {e}")
            self._flog(f"TX     reset epoch {e}")

    # ---- world-change control: user moved barriers -> stale blocks invalid ----
    def _on_world_changed(self, msg: Int32):
        n = int(msg.data)
        if n > self._world_seen:
            self._world_seen = n
            self._emit({"op": "unblock_all", "bot": self.my_id, "seq": n})

    # ---- periodic: claims + physical outcome reports (engine is Active) ----
    def tick(self):
        role, target = self.state.role(self.my_id)
        now = self.get_clock().now()

        # a fresh assignment must be reportable even if the same route was
        # reported in a previous attempt (retry after the user reopened it)
        if role == "wait":
            self._reported_for = None

        # 1. Physical outcome for my current target (explore or converge), once.
        #    An arrival counts only ON the target route's row: a bot can end up
        #    past goal_x on a DIFFERENT lane (e.g. shoved down a freshly opened
        #    route by a barrier flip mid-push) and must not credit its assigned
        #    route with a phantom `arrived` — that poisons the shared state.
        if role in ("explore", "converge") and target is not None:
            key = (self.state.epoch, target)
            if self._reported_for != key:
                row = self.lane_y.get(target)
                on_row = row is None or abs(self.pose_y - row) < 0.5
                if self.pose_x > self.goal_x and on_row:
                    self._emit({"op": "arrived", "bot": self.my_id, "route": target})
                    self._reported_for = key
                elif self.barrier_ahead:
                    self._emit({"op": "blocked", "bot": self.my_id, "route": target})
                    self._reported_for = key

        # 1b. Lease: another bot's assignment with no outcome for lease_sec
        #     means its robot is presumed dead. Propose a consensus timeout to
        #     release the route; every live bot proposes, the first one in
        #     consensus order acts and the rest are no-ops.
        for (bot, route), since in list(self._assign_seen.items()):
            if bot == self.my_id:
                continue
            if (now - since).nanoseconds / 1e9 < self.lease_sec:
                continue
            key = (self.state.epoch, bot, route)
            if key in self._timeout_sent:
                continue
            self._timeout_sent.add(key)
            self._emit({"op": "timeout", "bot": self.my_id,
                        "victim": bot, "route": route})

        # 2. Claim loop: while unassigned and exploring, claim a free route
        #    (home lane first). All bots do this concurrently; consensus order
        #    arbitrates — losers just claim again. When nothing is claimable for
        #    retry_after seconds, re-claim a blocked route (recovery).
        if self.state.phase == EXPLORING and self.my_id not in self.state.arrived \
                and self.my_id not in self.state.assigned:
            if (now - self._last_claim).nanoseconds / 1e9 >= self.claim_interval:
                route, retry = self._pick_route(now)
                if route is not None:
                    rec = {"op": "claim", "bot": self.my_id, "route": route}
                    if retry:
                        rec["retry"] = True
                    self._emit(rec)
                    self._last_claim = now
        else:
            self._unclaimable_since = None

        self._react()

    def _pick_route(self, now):
        """(route, retry). Prefer my home lane, then the others. Retry a blocked
        route ONLY when the whole fleet is idle (nothing claimable AND nobody is
        still exploring) — while any bot is still out there, a winner may land
        and everyone converges; re-probing known-blocked barriers meanwhile just
        drives bots back into walls."""
        avail = self.state.claimable_routes()
        if avail:
            self._unclaimable_since = None
            if self.home_route in avail:
                return self.home_route, False
            return (random.choice(avail) if self.random_routes else avail[0]), False
        if self.state.assigned:             # someone is still exploring: wait
            self._unclaimable_since = None
            return None, False
        if self._unclaimable_since is None:
            self._unclaimable_since = now
            return None, False
        if (now - self._unclaimable_since).nanoseconds / 1e9 < self.retry_after:
            return None, False
        pool = self.state.retryable_routes()
        if not pool:
            return None, False
        return (random.choice(pool) if self.random_routes else pool[0]), True

    # ---- translate the FSM role into a drive command + publish state ----
    def _react(self):
        role, target = self.state.role(self.my_id)

        # Consensus-coordinated collision avoidance at the funnel: converging
        # bots enter the winner lane STAGGERED by their deterministic rank
        # (identical on every node — derived from the shared arrived-set), so
        # no two cars corner into the same lane mouth at once.
        hold_converge = False
        if role == "converge":
            rank = self.state.converge_rank(self.my_id)
            if self._winner_seen is None:
                self._winner_seen = self.get_clock().now()
            waited = (self.get_clock().now() - self._winner_seen).nanoseconds / 1e9
            hold_converge = waited < rank * 5.0

        if role in ("explore", "converge") and not hold_converge:
            drive = target        # route id: drive it to the end
        elif role == "done":
            drive = "STOP"        # reached the end: park there
        else:                     # wait / staggered / returning: at the start
            drive = "STAGING"

        decision = (role, target, drive)
        if decision != self._last_decision:
            self._last_decision = decision
            extra = ""
            if role == "converge":
                extra = (f" (rank={self.state.converge_rank(self.my_id)}"
                         f"{', staggered-hold' if hold_converge else ', go'})")
            self._flog(f"DECIDE role={role} target={target} drive={drive}{extra}")

        # publish every call (heartbeat) — the rosbridge follower can subscribe
        # late and miss a one-shot publish over the lossy WebSocket bridge.
        self.drive_pub.publish(String(data=drive))

        self.state_pub.publish(String(data=json.dumps({
            "robot_id": self.my_id,
            "epoch": self.state.epoch,
            "assigned": {str(b): r for b, r in sorted(self.state.assigned.items())},
            "arrived": sorted(self.state.arrived),
            "blocked": sorted(self.state.blocked),
            "winner_route": self.state.winner_route,
            "phase": self.state.phase,
            "role": role,
        }, sort_keys=True)))


def main(args=None):
    spin_agent(MissionCoordinator)


if __name__ == "__main__":
    main()

"""Pure replicated state machine for consensus-coordinated route exploration —
PARALLEL model: all bots move concurrently, consensus assigns routes exclusively.

No ROS / Webots dependencies, so it is unit-testable on any host (see
test_mission_fsm.py). Given the same ordered sequence of decoded transaction
records (the Vertex consensus order on /vertex/event), every robot derives
identical global state — that determinism is the property under test.

Rules (see worlds/README.md):
  * Every bot claims a route via consensus; claims are processed in consensus
    order and NO TWO BOTS are ever assigned the same route (exclusive
    assignment). All assigned bots explore concurrently.
  * A blocked bot reports `blocked` and returns to the start; the moment that
    event is in consensus the route is out, its assignment is released, and the
    bot may immediately claim another free route — nobody waits on anybody.
  * The first `arrived` fixes the winner route: all assignments clear and every
    remaining bot converges onto the winner (convoy; physical spacing is the
    follower's proximity guard, not consensus).
  * If the winner route later gets blocked, exploration re-opens.
  * DONE when every bot has arrived.
  * Recovery: when no claimable route exists (all blocked), a bot may re-claim a
    blocked route with `retry: true` — if the user re-opened it, the arrival
    clears its blocked mark.
  * Lease (fault tolerance, n=4 tolerates f=1): a crashed explorer can neither
    report nor free its route. Any bot may propose a `timeout` for an
    assignment that has produced no outcome for the lease window; the first
    timeout in consensus order releases the assignment (the route becomes
    claimable again, NOT blocked — its state is unknown). Duplicates and
    stale timeouts (victim re-assigned or already released) are no-ops, so
    every bot proposing concurrently is safe.

Transaction payloads are opaque JSON records (all carry `epoch` for reset):
    {"op":"claim",   "bot":2, "route":"R1", "epoch":0}
    {"op":"claim",   "bot":2, "route":"R1", "epoch":0, "retry":true}
    {"op":"blocked", "bot":2, "route":"R1", "epoch":0}
    {"op":"arrived", "bot":2, "route":"R1", "epoch":0}
    {"op":"timeout", "bot":3, "victim":2, "route":"R1", "epoch":0}  # lease expiry
    {"op":"unblock_all", "bot":2,           "epoch":0}   # user changed barriers:
                                                         # stale blocks invalid
    {"op":"reset",                          "epoch":1}
"""

from __future__ import annotations

import json

# mission phase
EXPLORING, CONVERGING, DONE = "exploring", "converging", "done"


def encode(record: dict) -> bytes:
    """Serialize a record to the opaque bytes carried in VertexTransaction.payload."""
    return json.dumps(record, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode(payload) -> dict | None:
    """Decode transaction payload bytes back to a record, or None if malformed."""
    try:
        return json.loads(bytes(payload).decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return None


class MissionState:
    """Deterministic fold of the consensus-ordered record stream."""

    def __init__(self, routes, num_bots=4):
        self.routes = list(routes)
        self.num_bots = num_bots
        self.epoch = 0            # bumped by `reset`; stale-epoch records are ignored
        self._reset_state()

    def _reset_state(self) -> None:
        self.arrived: set[int] = set()     # bots that reached the end (stay there)
        self.blocked: set[str] = set()     # routes reported blocked
        self.winner_route = None           # first proven-open route
        self.assigned: dict[int, str] = {} # bot -> route, exclusive (consensus order)
        self.phase = EXPLORING

    # ---- fold one record (consensus order) ----
    def apply(self, rec: dict | None) -> None:
        if not rec or "op" not in rec:
            return
        op = rec["op"]

        if op == "reset":                  # restart at a fresh epoch (first one wins)
            e = rec.get("epoch", 0)
            if e > self.epoch:
                self._reset_state()
                self.epoch = e
            return
        if rec.get("epoch", 0) != self.epoch:
            return                         # stale record from a previous epoch

        bot, route = rec.get("bot"), rec.get("route")
        if op == "claim":
            # Exclusive assignment, first claim in consensus order wins the route.
            # `retry` claims may target a blocked route (all-blocked recovery).
            ok = (self.phase == EXPLORING and bot not in self.arrived
                  and bot not in self.assigned and route in self.routes
                  and route not in self.assigned.values()
                  and (route not in self.blocked or rec.get("retry")))
            if ok:
                self.assigned[bot] = route
        elif op == "blocked":
            if route is not None:
                self.blocked.add(route)
                if route == self.winner_route:
                    self.winner_route = None   # proven path lost -> re-explore
            # release the reporter (and anyone on that route) immediately —
            # others keep moving; the returner just drives home unassigned
            self.assigned = {b: r for b, r in self.assigned.items()
                             if b != bot and r != route}
        elif op == "arrived":
            if bot is not None:
                self.arrived.add(bot)
            if route is not None:
                self.blocked.discard(route)    # proven open (clears a retried route)
                if self.winner_route is None:
                    self.winner_route = route  # first success: everyone converges
                    self.assigned.clear()      # abandon in-flight explorations
            self.assigned.pop(bot, None)
        elif op == "timeout":
            # Lease expiry for a (presumed dead) explorer: release the victim's
            # assignment so the route becomes claimable again. Only acts if the
            # victim still holds exactly the named route, which makes stale and
            # duplicate timeouts no-ops (first one in consensus order wins).
            victim = rec.get("victim")
            if victim is not None and self.assigned.get(victim) == route \
                    and route is not None:
                del self.assigned[victim]
        elif op == "unblock_all":
            # The user changed the barriers: every recorded BLOCK is stale, so
            # clear only the blocked set. Assignments and the winner stay —
            # in-flight bots keep driving and reality re-teaches us: a bot on a
            # now-blocked route stalls and reports; a kept winner that is now
            # blocked gets discovered by the first converger. (Clearing
            # assignments here yanked every moving bot back to staging on each
            # of the four duplicate relays — fleet-wide whiplash.)
            self.blocked.clear()

        self._recompute_phase()

    def _recompute_phase(self) -> None:
        if len(self.arrived) >= self.num_bots:
            self.phase = DONE
        elif self.winner_route is not None:
            self.phase = CONVERGING
        else:
            self.phase = EXPLORING

    # ---- derived state (identical on every robot) ----
    def claimable_routes(self):
        """Routes a bot may claim right now (unblocked and unassigned)."""
        taken = set(self.assigned.values())
        return [r for r in self.routes if r not in self.blocked and r not in taken]

    def retryable_routes(self):
        """Blocked-but-unassigned routes (all-blocked recovery targets)."""
        taken = set(self.assigned.values())
        return [r for r in self.routes if r not in taken]

    def converge_rank(self, my_id: int) -> int:
        """Deterministic converge order (identical on every node): my index among
        the unarrived bots. Used to stagger entries into the winner lane so the
        funnel never has two cars cornering at once — consensus-coordinated
        spacing with zero extra messages."""
        pending = sorted(b for b in range(self.num_bots) if b not in self.arrived)
        return pending.index(my_id) if my_id in pending else 0

    def role(self, my_id: int):
        """What robot `my_id` should physically do now:
        ('explore', route) | ('converge', route) | ('done', None) | ('wait', None).
        """
        if my_id in self.arrived or self.phase == DONE:
            return ("done", None)
        if my_id in self.assigned:
            return ("explore", self.assigned[my_id])
        if self.phase == CONVERGING:
            return ("converge", self.winner_route)
        return ("wait", None)

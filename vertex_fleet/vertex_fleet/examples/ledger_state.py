"""The ledger example's replicated state: pure Python, no ROS imports, so it
is unit-testable on any host (the split every application should copy: pure
state in one module, the ROS node in another)."""

from vertex_fleet.state import ReplicatedState


class LedgerState(ReplicatedState):
    """The shared state: an ordered list of [agent, seq] entries."""

    def wipe(self):
        self.entries = []

    def apply_record(self, rec):
        if rec.get("op") != "append":
            return
        entry = [rec.get("agent"), rec.get("seq")]
        if None not in entry and entry not in self.entries:   # idempotent
            self.entries.append(entry)

#!/usr/bin/env python3
"""Minimal vertex_fleet consumer: a replicated append-only ledger.

Each agent contributes ``count`` entries by proposing ``append`` records into
consensus. Every agent folds the same finalized stream, so every agent
derives the identical ordered ledger, including entries from agents it has
never talked to directly. This file is the template to copy for a new
application: one ReplicatedState subclass, one VertexAgent subclass, one
``main``.

Records:
    {"op": "append", "agent": 0, "seq": 3, "epoch": 0}

Rules demonstrated:
  * idempotency: an (agent, seq) pair is folded once, so re-proposing while
    waiting for finalization is harmless
  * propose then wait: the agent keeps proposing its next entry until it
    OBSERVES it in the folded ledger, never assuming submission succeeded

Publishes its derived state as JSON on ``ledger`` (std_msgs/String) so tests
and humans can compare agents:
    {"agent": 0, "entries": [[a, s], ...], "epoch": 0, "done": true}

Run it under a per-agent namespace next to a vertex_node whose contract
topics are remapped into the same namespace (see test/ledger_demo.launch_test.py).
"""

import json

from std_msgs.msg import String

from vertex_fleet import VertexAgent, spin_agent
from vertex_fleet.examples.ledger_state import LedgerState


class LedgerAgent(VertexAgent):
    def __init__(self):
        super().__init__("ledger_agent", LedgerState())
        self.declare_parameter("agent_id", 0)
        self.declare_parameter("count", 5)
        self.me = int(self.get_parameter("agent_id").value)
        self.count = int(self.get_parameter("count").value)
        self._state_pub = self.create_publisher(String, "ledger", 10)

    def tick(self):
        # Propose my next entry until I SEE it in the folded ledger. Folding
        # is idempotent, so re-proposing while a record is in flight is safe.
        mine = {seq for agent, seq in self.state.entries if agent == self.me}
        if len(mine) < self.count:
            self.propose({"op": "append", "agent": self.me, "seq": len(mine)})
        self._publish_state()

    def on_state_changed(self):
        self._publish_state()

    def _publish_state(self):
        mine = sum(1 for agent, _ in self.state.entries if agent == self.me)
        self._state_pub.publish(String(data=json.dumps({
            "agent": self.me,
            "entries": self.state.entries,
            "epoch": self.state.epoch,
            "done": mine >= self.count,
        }, sort_keys=True)))


def main():
    spin_agent(LedgerAgent)


if __name__ == "__main__":
    main()

"""ReplicatedState: the deterministic-fold base class.

A consensus-coordinated fleet derives all shared state by folding the
totally-ordered record stream from /vertex/event through a pure function.
Because every agent folds the same log with the same function, every agent
derives the same state. This module gives applications that fold with the
restart semantics already handled:

  * every record carries an ``epoch``; records from older epochs are ignored
  * a ``{"op": "reset", "epoch": N}`` record wipes the state and moves to the
    new epoch (first reset in consensus order wins, duplicates are no-ops)

Subclass and implement two methods. Keep both PURE: no I/O, no clocks, no
randomness, no dependence on anything but the record and current state, or
agents will silently diverge.

    class LedgerState(ReplicatedState):
        def wipe(self):
            self.entries = []

        def apply_record(self, rec):
            if rec.get("op") == "append":
                entry = [rec.get("agent"), rec.get("seq")]
                if entry not in self.entries:      # idempotent
                    self.entries.append(entry)

Records are opaque JSON objects on the wire; ``encode``/``decode`` here define
the canonical byte form (sorted keys, compact separators) so identical records
have identical bytes on every agent.
"""

from __future__ import annotations

import json


def encode(record: dict) -> bytes:
    """Serialize a record to the canonical opaque bytes carried in
    VertexTransaction.payload."""
    return json.dumps(record, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode(payload) -> dict | None:
    """Decode transaction payload bytes back to a record, or None if the bytes
    are not a valid record (foreign traffic on the same mesh is ignored)."""
    try:
        rec = json.loads(bytes(payload).decode("utf-8"))
    except (ValueError, TypeError, UnicodeDecodeError, RecursionError):
        # A record is untrusted input off the mesh: malformed bytes, non-uint8
        # values, invalid UTF-8, and deeply-nested JSON (which raises
        # RecursionError, not ValueError) must all be dropped, never crash the
        # fold. This is the trust boundary for /vertex/event payloads.
        return None
    return rec if isinstance(rec, dict) else None


class ReplicatedState:
    """Deterministic fold of the consensus-ordered record stream.

    Feed it every decoded record from /vertex/event, in delivery order, and
    only from there (VertexAgent enforces this). ``epoch`` and ``reset``
    handling are provided; subclasses implement ``wipe`` (clear all derived
    state) and ``apply_record`` (fold one same-epoch record).
    """

    def __init__(self):
        self.epoch = 0
        self.wipe()

    # ---- subclass API ----
    def wipe(self) -> None:
        """Reset every piece of derived state. Called on construction and when
        a ``reset`` record is applied."""
        raise NotImplementedError

    def apply_record(self, record: dict) -> None:
        """Fold one record that already passed the epoch gate. Must be pure
        and idempotent (the same record may be proposed by several agents)."""
        raise NotImplementedError

    # ---- the fold ----
    def apply(self, record: dict | None) -> None:
        if not record or "op" not in record:
            return
        if record["op"] == "reset":
            epoch = record.get("epoch", 0)
            if isinstance(epoch, int) and epoch > self.epoch:
                self.wipe()
                self.epoch = epoch
            return
        if record.get("epoch", 0) != self.epoch:
            return          # stale record from a previous epoch
        self.apply_record(record)

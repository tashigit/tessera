"""Unit tests for vertex_fleet.state: the determinism guarantees consumers
rely on. Plain python3, no ROS:  python3 test/test_state.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vertex_fleet.state import ReplicatedState, decode, encode  # noqa: E402
from vertex_fleet.examples.ledger_state import LedgerState      # noqa: E402


def R(op, **kw):
    return {"op": op, "epoch": kw.pop("epoch", 0), **kw}


def test_codec_canonical_roundtrip():
    rec = {"op": "append", "agent": 2, "seq": 1, "epoch": 0}
    assert decode(encode(rec)) == rec
    # canonical form: key order in the source dict must not change the bytes
    reordered = {"seq": 1, "epoch": 0, "op": "append", "agent": 2}
    assert encode(rec) == encode(reordered)
    assert decode(b"not json") is None
    assert decode(encode({"op": "x"})) == {"op": "x"}
    assert decode(b"[1,2,3]") is None          # non-object payloads rejected
    print("ok  codec_canonical_roundtrip")


def test_same_log_same_state_across_instances():
    log = [R("append", agent=0, seq=0), R("append", agent=1, seq=0),
           R("append", agent=0, seq=1)]
    fleet = [LedgerState() for _ in range(4)]
    for s in fleet:
        for rec in log:
            s.apply(rec)
    assert all(s.entries == fleet[0].entries for s in fleet)
    assert fleet[0].entries == [[0, 0], [1, 0], [0, 1]]
    print("ok  same_log_same_state_across_instances")


def test_duplicates_are_idempotent():
    s = LedgerState()
    for _ in range(3):
        s.apply(R("append", agent=0, seq=0))
    assert s.entries == [[0, 0]]
    print("ok  duplicates_are_idempotent")


def test_epoch_gates_stale_records():
    s = LedgerState()
    s.apply(R("append", agent=0, seq=0))
    s.apply({"op": "reset", "epoch": 1})
    assert s.entries == [] and s.epoch == 1
    s.apply(R("append", agent=0, seq=1, epoch=0))     # stale: ignored
    assert s.entries == []
    s.apply(R("append", agent=0, seq=1, epoch=1))
    assert s.entries == [[0, 1]]
    print("ok  epoch_gates_stale_records")


def test_reset_first_wins_duplicates_noop():
    s = LedgerState()
    s.apply({"op": "reset", "epoch": 2})
    s.apply(R("append", agent=1, seq=0, epoch=2))
    s.apply({"op": "reset", "epoch": 2})              # duplicate: no wipe
    assert s.entries == [[1, 0]] and s.epoch == 2
    s.apply({"op": "reset", "epoch": 1})              # older: ignored
    assert s.epoch == 2
    print("ok  reset_first_wins_duplicates_noop")


def test_malformed_records_ignored():
    s = LedgerState()
    for junk in (None, {}, {"noop": 1}, {"op": "append"},
                 {"op": "reset", "epoch": "high"}):
        s.apply(junk)
    assert s.entries == [] and s.epoch == 0
    print("ok  malformed_records_ignored")


def test_base_class_requires_implementation():
    try:
        ReplicatedState()
    except NotImplementedError:
        print("ok  base_class_requires_implementation")
    else:
        raise AssertionError("bare ReplicatedState must not construct")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nall {len(tests)} vertex_fleet state tests passed")


if __name__ == "__main__":
    main()

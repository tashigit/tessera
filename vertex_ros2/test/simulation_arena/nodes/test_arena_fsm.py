"""Unit tests for arena_fsm — consensus-coordinated arena exploration.

Runs on any host with plain python3:  python3 test_arena_fsm.py

Core property: feeding the SAME consensus-ordered record log to N independent
ArenaState instances yields identical derived state on all of them —
mirroring Vertex's byte-identical /vertex/event guarantee.
"""

import sys

from arena_fsm import DONE, EXPLORING, ArenaState, decode, encode, make_grid

SECTORS, CENTERS = make_grid(3, 2, -20.0, -15.0, 40.0 / 3, 15.0)


def fresh_fleet(n=5):
    return [ArenaState(SECTORS, num_bots=5) for _ in range(n)]


def feed(fleet, log):
    for fsm in fleet:
        for rec in log:
            fsm.apply(rec)


def snap(fsm):
    return (tuple(sorted(fsm.claimed.items())), tuple(sorted(fsm.explored)),
            tuple(sorted(fsm.unreachable)), tuple(sorted(fsm.unhealthy)),
            tuple((d["bot"], d["seq"], d["label"]) for d in fsm.detections),
            fsm.phase)


def assert_agreement(fleet, label):
    snaps = [snap(f) for f in fleet]
    assert all(s == snaps[0] for s in snaps), f"{label}: fleet diverged: {snaps}"


def R(op, bot=None, sector=None, epoch=0, **kw):
    d = {"op": op, "epoch": epoch, **kw}
    if bot is not None:
        d["bot"] = bot
    if sector is not None:
        d["sector"] = sector
    return d


def test_grid_geometry():
    ids, centers = make_grid(2, 2, 0.0, 0.0, 10.0, 10.0)
    assert ids == ["S00", "S01", "S02", "S03"]
    assert centers["S00"] == (5.0, 5.0)
    assert centers["S03"] == (15.0, 15.0)
    print("ok  grid_geometry")


def test_codec_roundtrip():
    rec = {"op": "claim", "bot": 2, "sector": "S01", "epoch": 0}
    assert decode(encode(rec)) == rec
    assert decode(b"nope") is None
    print("ok  codec_roundtrip")


def test_exclusive_claims():
    # two bots race for one sector; first in consensus order wins, and a bot
    # holds at most one sector at a time
    fleet = fresh_fleet()
    feed(fleet, [
        R("claim", 0, "S00"), R("claim", 1, "S00"),   # bot1 loses S00
        R("claim", 1, "S01"), R("claim", 0, "S02"),   # bot0 already holds S00
    ])
    assert_agreement(fleet, "exclusive_claims")
    assert fleet[0].claimed == {"S00": 0, "S01": 1}
    print("ok  exclusive_claims")


def test_explored_requires_holder():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("explored", 1, "S00"))                # not the holder -> no-op
    assert fsm.explored == set()
    fsm.apply(R("claim", 1, "S00"))
    fsm.apply(R("explored", 2, "S00"))                # someone else -> no-op
    assert fsm.explored == set()
    fsm.apply(R("explored", 1, "S00"))                # the holder
    assert fsm.explored == {"S00"} and fsm.claimed == {}
    fsm.apply(R("claim", 2, "S00"))                   # explored: never re-claimed
    assert fsm.claimed == {}
    print("ok  explored_requires_holder")


def test_abandon_releases_for_others():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("claim", 0, "S01"))
    fsm.apply(R("abandon", 0, "S01"))
    assert fsm.claimed == {}
    fsm.apply(R("claim", 3, "S01"))                   # someone else may try
    assert fsm.claimed == {"S01": 3}
    print("ok  abandon_releases_for_others")


def test_unreachable_requires_holder_and_excludes():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("unreachable", 0, "S01"))             # not holding -> no-op
    assert fsm.unreachable == set()
    fsm.apply(R("claim", 0, "S01"))
    fsm.apply(R("unreachable", 0, "S01"))
    assert fsm.unreachable == {"S01"} and fsm.claimed == {}
    fsm.apply(R("claim", 2, "S01"))                   # condemned: not claimable
    assert fsm.claimed == {}
    assert "S01" not in fsm.claimable_sectors()
    print("ok  unreachable_requires_holder_and_excludes")


def test_health_beacon_marks_and_readmits():
    fleet = fresh_fleet()
    feed(fleet, [
        R("claim", 4, "S02"),
        R("health", 4, seq=0, ok=True),
        R("health", 4, seq=1, ok=False),              # unhealthy: claim released
    ])
    assert_agreement(fleet, "health_mark")
    assert fleet[0].unhealthy == {4} and fleet[0].claimed == {}
    feed(fleet, [R("claim", 4, "S02")])               # ignored while unhealthy
    assert fleet[0].claimed == {}
    feed(fleet, [R("health", 4, seq=2, ok=True),      # readmitted
                 R("claim", 4, "S02")])
    assert_agreement(fleet, "health_readmit")
    assert fleet[0].unhealthy == set() and fleet[0].claimed == {"S02": 4}
    print("ok  health_beacon_marks_and_readmits")


def test_health_stale_seq_ignored():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("health", 2, seq=5, ok=True))
    fsm.apply(R("health", 2, seq=4, ok=False))        # stale -> no-op
    assert fsm.unhealthy == set() and fsm.health_seq[2] == 5
    fsm.apply(R("health", 2, seq=5, ok=False))        # duplicate seq -> no-op
    assert fsm.unhealthy == set()
    print("ok  health_stale_seq_ignored")


def test_suspect_silence_lease():
    fleet = fresh_fleet()
    feed(fleet, [
        R("health", 2, seq=7, ok=True),
        R("claim", 2, "S03"),
        R("suspect", 0, victim=2, seen_seq=7),        # silent since seq 7 -> out
    ])
    assert_agreement(fleet, "suspect")
    assert fleet[0].unhealthy == {2} and fleet[0].claimed == {}
    feed(fleet, [R("suspect", 1, victim=2, seen_seq=7)])   # duplicate -> no-op
    assert fleet[0].unhealthy == {2}
    print("ok  suspect_silence_lease")


def test_late_beacon_voids_suspicion():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("health", 2, seq=7, ok=True))
    fsm.apply(R("health", 2, seq=8, ok=True))         # beacon lands first
    fsm.apply(R("suspect", 0, victim=2, seen_seq=7))  # stale observation -> no-op
    assert fsm.unhealthy == set()
    print("ok  late_beacon_voids_suspicion")


def test_detection_gated_on_health():
    fleet = fresh_fleet()
    feed(fleet, [
        R("health", 1, seq=0, ok=True),
        R("detection", 1, seq=0, label="deer", x=-14.6, y=9.6),   # accepted
        R("health", 4, seq=0, ok=False),
        R("detection", 4, seq=0, label="deer", x=11.8, y=5.0),    # rejected
        R("detection", 1, seq=0, label="deer", x=-14.6, y=9.6),   # dup -> no-op
    ])
    assert_agreement(fleet, "detections")
    assert [(d["bot"], d["label"]) for d in fleet[0].detections] == [(1, "deer")]
    print("ok  detection_gated_on_health")


def test_mission_completes():
    fleet = fresh_fleet()
    log = []
    for i, s in enumerate(SECTORS[:-1]):
        bot = i % 5
        log += [R("claim", bot, s), R("explored", bot, s)]
    log += [R("claim", 0, SECTORS[-1]), R("unreachable", 0, SECTORS[-1])]
    feed(fleet, log)
    assert_agreement(fleet, "completion")
    assert fleet[0].phase == DONE
    assert fleet[0].role(3) == ("done", None)
    assert fleet[0].claimable_sectors() == []
    feed(fleet, [R("claim", 2, "S00")])               # done: claims ignored...
    assert fleet[0].claimed == {}
    print("ok  mission_completes")


def test_roles():
    fsm = ArenaState(SECTORS)
    assert fsm.role(0) == ("idle", None)
    fsm.apply(R("claim", 0, "S04"))
    assert fsm.role(0) == ("explore", "S04")
    assert fsm.role(1) == ("idle", None)
    assert fsm.my_claim(0) == "S04" and fsm.my_claim(1) is None
    print("ok  roles")


def test_reset_epoch():
    fsm = ArenaState(SECTORS)
    fsm.apply(R("claim", 0, "S00"))
    fsm.apply(R("explored", 0, "S00"))
    fsm.apply(R("health", 4, seq=3, ok=False))
    fsm.apply({"op": "reset", "epoch": 1})
    assert fsm.epoch == 1 and fsm.explored == set() and fsm.unhealthy == set()
    fsm.apply(R("claim", 0, "S00", epoch=0))          # stale epoch -> ignored
    assert fsm.claimed == {}
    fsm.apply(R("claim", 0, "S00", epoch=1))
    assert fsm.claimed == {"S00": 0}
    print("ok  reset_epoch")


def test_same_log_same_state_shuffled_sources():
    # the log interleaves records from every bot; all replicas agree at every
    # prefix, not just at the end
    log = [
        R("health", 0, seq=0, ok=True), R("claim", 0, "S00"),
        R("health", 1, seq=0, ok=True), R("claim", 1, "S00"),
        R("claim", 1, "S01"), R("explored", 0, "S00"),
        R("health", 1, seq=1, ok=False), R("claim", 2, "S01"),
        R("claim", 2, "S02"),   # ignored: bot2 already holds S01
        R("detection", 2, seq=0, label="rock", x=9.1, y=7.1),
        R("suspect", 0, victim=3, seen_seq=-1),
        R("health", 1, seq=2, ok=True), R("claim", 1, "S01"),  # S01 taken by 2
    ]
    fleet = fresh_fleet()
    for k in range(len(log)):
        feed(fleet, [log[k]])
        assert_agreement(fleet, f"prefix {k}")
    assert fleet[0].claimed == {"S01": 2}
    assert fleet[0].unhealthy == {3}                  # never beaconed: suspect held
    print("ok  same_log_same_state_shuffled_sources")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nall {len(tests)} arena_fsm tests passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        print(f"FAIL: {e}")
        sys.exit(1)

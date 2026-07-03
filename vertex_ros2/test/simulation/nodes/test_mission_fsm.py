"""Unit tests for mission_fsm — parallel consensus route assignment.

Runs on any host with plain python3:  python3 test_mission_fsm.py

Core property: feeding the SAME consensus-ordered record log to N independent
MissionState instances yields identical derived state on all of them —
mirroring Vertex's byte-identical /vertex/event guarantee.
"""

import sys

from mission_fsm import CONVERGING, DONE, EXPLORING, MissionState, decode, encode

ROUTES = ["R1", "R2", "R3", "R4"]


def fresh_fleet(n=4):
    return [MissionState(ROUTES, num_bots=4) for _ in range(n)]


def feed(fleet, log):
    for fsm in fleet:
        for rec in log:
            fsm.apply(rec)


def snap(fsm):
    return (tuple(sorted(fsm.arrived)), tuple(sorted(fsm.blocked)),
            fsm.winner_route, tuple(sorted(fsm.assigned.items())), fsm.phase)


def assert_agreement(fleet, label):
    snaps = [snap(f) for f in fleet]
    assert all(s == snaps[0] for s in snaps), f"{label}: fleet diverged: {snaps}"


def R(op, bot=None, route=None, epoch=0, **kw):
    d = {"op": op, "epoch": epoch, **kw}
    if bot is not None:
        d["bot"] = bot
    if route is not None:
        d["route"] = route
    return d


def test_codec_roundtrip():
    rec = {"op": "claim", "bot": 2, "route": "R1", "epoch": 0}
    assert decode(encode(rec)) == rec
    assert decode(b"nope") is None
    print("ok  codec_roundtrip")


def test_exclusive_parallel_assignment():
    # all four claim concurrently; consensus order resolves conflicts — no two
    # bots ever share a route, and everyone with a distinct claim explores at once
    fleet = fresh_fleet()
    feed(fleet, [
        R("claim", 0, "R1"), R("claim", 1, "R1"),   # bot1 loses R1 to bot0
        R("claim", 1, "R2"), R("claim", 2, "R3"), R("claim", 3, "R4"),
    ])
    assert_agreement(fleet, "assign")
    f = fleet[0]
    assert f.assigned == {0: "R1", 1: "R2", 2: "R3", 3: "R4"}
    assert len(set(f.assigned.values())) == 4          # exclusivity
    for i in range(4):
        assert fleet[i].role(i)[0] == "explore"        # all move concurrently
    print("ok  exclusive_parallel_assignment")


def test_blocked_frees_immediately_no_waiting():
    # a block releases the reporter instantly; others' assignments are untouched
    # (nobody waits on the returner), and the freed bot can claim a new route.
    fleet = fresh_fleet()
    feed(fleet, [R("claim", 0, "R1"), R("claim", 1, "R2"),
                 R("blocked", 0, "R1")])
    assert_agreement(fleet, "block")
    f = fleet[0]
    assert 0 not in f.assigned and f.assigned == {1: "R2"}   # bot1 unaffected
    assert "R1" in f.blocked
    assert f.role(1)[0] == "explore"                         # still moving
    feed(fleet, [R("claim", 0, "R3")])
    assert fleet[0].assigned[0] == "R3"                      # instant re-claim
    print("ok  blocked_frees_immediately_no_waiting")


def test_claim_on_blocked_route_needs_retry_flag():
    fsm = MissionState(ROUTES)
    fsm.apply(R("blocked", 2, "R1"))
    fsm.apply(R("claim", 0, "R1"))
    assert 0 not in fsm.assigned                    # plain claim refused
    fsm.apply(R("claim", 0, "R1", retry=True))
    assert fsm.assigned[0] == "R1"                  # retry allowed (recovery)
    fsm.apply(R("arrived", 0, "R1"))
    assert "R1" not in fsm.blocked                  # arrival proves it open again
    print("ok  claim_on_blocked_route_needs_retry_flag")


def test_first_arrival_converges_everyone():
    # bots explore in parallel; first arrival sets the winner, all in-flight
    # assignments clear, and every unarrived bot converges on the winner.
    fleet = fresh_fleet()
    feed(fleet, [R("claim", 0, "R1"), R("claim", 1, "R2"),
                 R("claim", 2, "R3"), R("claim", 3, "R4"),
                 R("arrived", 1, "R2")])
    assert_agreement(fleet, "winner")
    f = fleet[0]
    assert f.winner_route == "R2" and f.phase == CONVERGING
    assert f.assigned == {}                          # explorations abandoned
    assert f.role(0) == ("converge", "R2")
    assert f.role(1) == ("done", None)
    # converging bots arrive one by one -> DONE
    feed(fleet, [R("arrived", 0, "R2"), R("arrived", 2, "R2"),
                 R("arrived", 3, "R2")])
    assert_agreement(fleet, "done")
    assert fleet[0].phase == DONE
    print("ok  first_arrival_converges_everyone")


def test_winner_blocked_reopens_exploration():
    fsm = MissionState(ROUTES)
    fsm.apply(R("claim", 0, "R2")); fsm.apply(R("arrived", 0, "R2"))
    assert fsm.phase == CONVERGING and fsm.winner_route == "R2"
    fsm.apply(R("blocked", 1, "R2"))
    assert fsm.winner_route is None and "R2" in fsm.blocked
    assert fsm.phase == EXPLORING
    assert fsm.role(1) == ("wait", None)             # back to claiming
    print("ok  winner_blocked_reopens_exploration")


def test_single_open_route_scenario():
    # the user's test mode: only R3 open. Everyone races to claim it; one wins,
    # the rest wait; the winner's arrival converges everyone onto R3.
    fleet = fresh_fleet()
    feed(fleet, [R("blocked", 0, "R1"), R("blocked", 1, "R2"),
                 R("blocked", 3, "R4")])
    f = fleet[0]
    assert f.claimable_routes() == ["R3"]
    feed(fleet, [R("claim", 2, "R3"), R("claim", 0, "R3"), R("claim", 1, "R3")])
    assert_agreement(fleet, "race")
    assert fleet[0].assigned == {2: "R3"}            # single winner of the race
    assert fleet[0].role(0) == ("wait", None)
    feed(fleet, [R("arrived", 2, "R3")])
    assert fleet[0].phase == CONVERGING and fleet[0].winner_route == "R3"
    print("ok  single_open_route_scenario")


def test_unblock_all_reopens_stale_world():
    # the user's "3rd turn": blocks accumulated over earlier turns; the user
    # re-opens a route physically -> the supervisor's world-change broadcast
    # (relayed as unblock_all) must clear every stale mark so bots re-explore
    # immediately instead of wandering through the retry loop.
    fleet = fresh_fleet()
    feed(fleet, [R("blocked", 0, "R1"), R("blocked", 1, "R2"),
                 R("blocked", 2, "R3"), R("blocked", 3, "R4")])
    assert fleet[0].claimable_routes() == []          # fully stale-blocked
    feed(fleet, [R("unblock_all", 0)])
    assert_agreement(fleet, "unblock")
    f = fleet[0]
    assert f.blocked == set() and f.claimable_routes() == ROUTES
    assert f.phase == EXPLORING
    # arrivals, assignments and the winner all survive an unblock — only the
    # stale block knowledge is wiped (in-flight bots keep driving; reality
    # re-teaches the rest)
    feed(fleet, [R("claim", 1, "R1"), R("arrived", 1, "R1"),
                 R("claim", 0, "R2"), R("unblock_all", 2)])
    assert fleet[0].arrived == {1}
    assert fleet[0].winner_route == "R1"
    assert fleet[0].assigned == {}                    # cleared when winner was set
    print("ok  unblock_all_reopens_stale_world")


def test_converge_rank_is_deterministic():
    fleet = fresh_fleet()
    feed(fleet, [R("claim", 1, "R2"), R("arrived", 1, "R2")])
    # unarrived = [0, 2, 3] -> ranks 0, 1, 2 identical on every node
    for f in fleet:
        assert f.converge_rank(0) == 0
        assert f.converge_rank(2) == 1
        assert f.converge_rank(3) == 2
    print("ok  converge_rank_is_deterministic")


def test_reset_wipes_state_via_epoch():
    fsm = MissionState(ROUTES)
    fsm.apply(R("claim", 0, "R1")); fsm.apply(R("arrived", 0, "R1"))
    assert fsm.arrived == {0}
    fsm.apply({"op": "reset", "epoch": 1})
    assert (fsm.epoch == 1 and fsm.arrived == set() and fsm.assigned == {}
            and fsm.winner_route is None and fsm.phase == EXPLORING)
    fsm.apply(R("arrived", 0, "R1", epoch=0))        # stale epoch -> ignored
    assert fsm.arrived == set()
    fsm.apply(R("claim", 0, "R1", epoch=1)); fsm.apply(R("arrived", 0, "R1", epoch=1))
    assert fsm.arrived == {0}
    print("ok  reset_wipes_state_via_epoch")


def test_timeout_releases_dead_explorers_route():
    # lease expiry: a timeout frees the victim's route so others can claim it;
    # the route is NOT marked blocked (its state is unknown)
    fleet = fresh_fleet()
    feed(fleet, [
        R("claim", 2, "R3"),
        R("timeout", 0, "R3", victim=2),
        R("claim", 1, "R3"),                 # released route is claimable again
    ])
    assert_agreement(fleet, "timeout")
    f = fleet[0]
    assert f.assigned == {1: "R3"} and "R3" not in f.blocked
    assert f.role(2) == ("wait", None)       # victim (if alive) is just unassigned
    print("ok  timeout_releases_dead_explorers_route")


def test_timeout_stale_and_duplicate_are_noops():
    fsm = MissionState(ROUTES)
    fsm.apply(R("timeout", 0, "R3", victim=2))            # victim not assigned
    assert fsm.assigned == {}
    fsm.apply(R("claim", 2, "R3"))
    fsm.apply(R("timeout", 0, "R3", victim=2))            # releases
    fsm.apply(R("timeout", 1, "R3", victim=2))            # duplicate -> no-op
    assert fsm.assigned == {}
    fsm.apply(R("claim", 2, "R1"))                        # victim re-claimed
    fsm.apply(R("timeout", 3, "R3", victim=2))            # stale route -> no-op
    assert fsm.assigned == {2: "R1"}
    fsm.apply(R("timeout", 3, None, victim=2))            # no route -> no-op
    assert fsm.assigned == {2: "R1"}
    print("ok  timeout_stale_and_duplicate_are_noops")


def test_timeout_does_not_touch_arrived_bots():
    fsm = MissionState(ROUTES)
    fsm.apply(R("claim", 2, "R3")); fsm.apply(R("arrived", 2, "R3"))
    fsm.apply(R("timeout", 0, "R3", victim=2))            # already released
    assert 2 in fsm.arrived and fsm.winner_route == "R3"
    print("ok  timeout_does_not_touch_arrived_bots")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nall {len(tests)} mission_fsm tests passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        print(f"FAIL: {e}")
        sys.exit(1)

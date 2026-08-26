"""Unit tests for airground_fsm — consensus-coordinated air/ground
survey-and-sweep.

Runs on any host with plain python3:  python3 test_airground_fsm.py

Core property, same as the first two simulations: feeding the SAME
consensus-ordered record log to N independent AirGroundState instances yields
identical derived state on all of them, mirroring Vertex's byte-identical
/vertex/event guarantee.

The property this simulation adds: those N instances stand for a MIXED fleet.
Two of them are what a tessera bot derives and two are what a Rust drone
derives, so agreement here is agreement across the tiers. The Rust side folds
the same log through air_agent/src/fold.rs, and fixtures/conformance.json is
the cross-language test that pins the two implementations together.
"""

import sys

from airground_fsm import (DONE, SURVEYING, AirGroundState, canonical_snapshot,
                           decode, encode, make_blocks, make_sectors)

# 4x3 sectors, 6 blocks of 2x1 sectors each (blocks outnumber the drones, so
# survey claims contend). Mirrors the launch geometry.
NX, NY = 4, 3
SECTORS, CENTERS = make_sectors(NX, NY, -20.0, -15.0, 10.0, 10.0)
BLOCKS, BLOCK_CELLS = make_blocks(NX, NY, 2, 1)
AGENTS = ["bot_0", "bot_1", "drone_0", "drone_1"]

BOTS = ("bot_0", "bot_1")
DRONES = ("drone_0", "drone_1")


def fresh_fleet(n=4):
    return [AirGroundState(SECTORS, BLOCKS, BLOCK_CELLS, AGENTS)
            for _ in range(n)]


def feed(fleet, log):
    for fsm in fleet:
        for rec in log:
            fsm.apply(rec)


def snap(fsm):
    return (tuple(sorted(fsm.block_claims.items())),
            tuple(sorted(fsm.surveyed_blocks)),
            tuple(sorted(fsm.surveyed)),
            tuple(sorted(fsm.grounded)),
            tuple(sorted(fsm.claimed.items())),
            tuple(sorted(fsm.explored)),
            tuple(sorted(fsm.explored_by.items())),
            tuple(sorted(fsm.unreachable)),
            tuple(sorted((c, tuple(sorted(w)))
                         for c, w in fsm.hazard_reports.items())),
            tuple(sorted(fsm.confirmed_hazards)),
            tuple(sorted(fsm.unhealthy)),
            fsm.phase)


def assert_agreement(fleet, label):
    snaps = [snap(f) for f in fleet]
    assert all(s == snaps[0] for s in snaps), f"{label}: fleet diverged"


def R(op, agent=None, epoch=0, **kw):
    d = {"op": op, "epoch": epoch, **kw}
    if agent is not None:
        d["agent"] = agent
    return d


def survey(agent, block, cells=None):
    """The two records that take a block from unclaimed to surveyed."""
    return [R("survey_claim", agent, block=block),
            R("surveyed", agent, block=block,
              cells=cells if cells is not None else BLOCK_CELLS[block])]


# --------------------------------------------------------------- geometry ---

def test_sector_geometry():
    ids, centers = make_sectors(2, 2, 0.0, 0.0, 10.0, 10.0)
    assert ids == ["S00", "S01", "S02", "S03"]
    assert centers["S00"] == (5.0, 5.0)
    assert centers["S03"] == (15.0, 15.0)
    print("ok  sector_geometry")


def test_block_geometry():
    ids, cells = make_blocks(4, 3, 2, 1)
    assert ids == ["B00", "B01", "B02", "B03", "B04", "B05"]
    assert cells["B00"] == ["S00", "S01"]
    assert cells["B01"] == ["S02", "S03"]
    assert cells["B05"] == ["S10", "S11"]
    # every sector belongs to exactly one block
    covered = [s for b in ids for s in cells[b]]
    assert sorted(covered) == sorted(SECTORS)
    assert len(covered) == len(set(covered))
    print("ok  block_geometry")


def test_block_geometry_ragged():
    # A grid that does not divide evenly must not invent sectors.
    ids, cells = make_blocks(3, 1, 2, 1)
    assert ids == ["B00", "B01"]
    assert cells["B01"] == ["S02"]
    print("ok  block_geometry_ragged")


def test_codec_roundtrip():
    rec = {"agent": "drone_0", "block": "B01", "epoch": 0, "op": "survey_claim"}
    assert decode(encode(rec)) == rec
    assert decode(b"nope") is None
    print("ok  codec_roundtrip")


# ------------------------------------------------------------------- air ---

def test_block_claim_is_exclusive():
    fleet = fresh_fleet()
    feed(fleet, [R("survey_claim", "drone_0", block="B00"),
                 R("survey_claim", "drone_1", block="B00")])
    assert fleet[0].block_claims == {"B00": "drone_0"}
    assert_agreement(fleet, "block exclusivity")
    print("ok  block_claim_is_exclusive")


def test_drone_holds_one_block():
    fleet = fresh_fleet()
    feed(fleet, [R("survey_claim", "drone_0", block="B00"),
                 R("survey_claim", "drone_0", block="B01")])
    assert fleet[0].block_claims == {"B00": "drone_0"}
    print("ok  drone_holds_one_block")


def test_surveyed_requires_holder():
    fleet = fresh_fleet()
    # drone_1 never claimed B00, so its survey report is ignored entirely.
    feed(fleet, [R("survey_claim", "drone_0", block="B00"),
                 R("surveyed", "drone_1", block="B00",
                   cells=BLOCK_CELLS["B00"])])
    assert fleet[0].surveyed == set()
    assert fleet[0].surveyed_blocks == set()
    assert fleet[0].block_claims == {"B00": "drone_0"}
    print("ok  surveyed_requires_holder")


def test_survey_cannot_clear_beyond_its_block():
    """A drone holding B00 that names every sector clears only B00's."""
    fleet = fresh_fleet()
    feed(fleet, [R("survey_claim", "drone_0", block="B00"),
                 R("surveyed", "drone_0", block="B00", cells=list(SECTORS))])
    assert fleet[0].surveyed == set(BLOCK_CELLS["B00"])
    print("ok  survey_cannot_clear_beyond_its_block")


def test_rtb_releases_block_and_grounds():
    fleet = fresh_fleet()
    feed(fleet, [R("survey_claim", "drone_0", block="B00"),
                 R("rtb", "drone_0")])
    assert fleet[0].block_claims == {}
    assert fleet[0].grounded == {"drone_0"}
    # grounded: may not take new work
    feed(fleet, [R("survey_claim", "drone_0", block="B01")])
    assert fleet[0].block_claims == {}
    # and the other drone picks the released block up
    feed(fleet, [R("survey_claim", "drone_1", block="B00")])
    assert fleet[0].block_claims == {"B00": "drone_1"}
    # ready readmits
    feed(fleet, [R("ready", "drone_0"),
                 R("survey_claim", "drone_0", block="B01")])
    assert fleet[0].grounded == set()
    assert fleet[0].block_claims == {"B00": "drone_1", "B01": "drone_0"}
    assert_agreement(fleet, "rtb lease")
    print("ok  rtb_releases_block_and_grounds")


# ---------------------------------------------------------------- ground ---

def test_sector_unclaimable_before_survey():
    """The cross-tier gate: ground work waits on air work."""
    fleet = fresh_fleet()
    feed(fleet, [R("claim", "bot_0", sector="S00")])
    assert fleet[0].claimed == {}, "claimed an unsurveyed sector"
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00")])
    assert fleet[0].claimed == {"S00": "bot_0"}
    assert_agreement(fleet, "cross-tier gate")
    print("ok  sector_unclaimable_before_survey")


def test_sector_claim_is_exclusive():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00"),
                 R("claim", "bot_1", sector="S00")])
    assert fleet[0].claimed == {"S00": "bot_0"}
    print("ok  sector_claim_is_exclusive")


def test_explored_requires_holder():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00"),
                 R("explored", "bot_1", sector="S00")])
    assert fleet[0].explored == set()
    feed(fleet, [R("explored", "bot_0", sector="S00")])
    assert fleet[0].explored == {"S00"}
    assert fleet[0].explored_by == {"S00": "bot_0"}
    assert fleet[0].claimed == {}
    print("ok  explored_requires_holder")


# -------------------------------------------------------------- evidence ---

def test_one_hazard_is_provisional():
    """A single witness defers a sector but must not condemn it."""
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("hazard", "drone_0", cell="S00", kind="pit")])
    assert fleet[0].unreachable == set()
    assert fleet[0].confirmed_hazards == set()
    assert fleet[0].provisional_hazards() == ["S00"]
    # still claimable, just deferred
    assert "S00" in fleet[0].claimable_sectors()
    print("ok  one_hazard_is_provisional")


def test_two_witnesses_confirm():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("hazard", "drone_0", cell="S00", kind="pit"),
                 R("corroborate", "bot_1", cell="S00")])
    assert fleet[0].confirmed_hazards == {"S00"}
    assert fleet[0].unreachable == {"S00"}
    assert "S00" not in fleet[0].claimable_sectors()
    assert_agreement(fleet, "corroboration")
    print("ok  two_witnesses_confirm")


def test_one_agent_cannot_self_corroborate():
    """The lying-drone case: shouting twice is still one witness."""
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("hazard", "drone_1", cell="S00", kind="pit"),
                 R("corroborate", "drone_1", cell="S00"),
                 R("hazard", "drone_1", cell="S00", kind="pit")])
    assert fleet[0].confirmed_hazards == set(), "a single agent condemned a sector"
    assert fleet[0].unreachable == set()
    print("ok  one_agent_cannot_self_corroborate")


def test_confirmed_hazard_releases_the_claim():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00"),
                 R("hazard", "drone_0", cell="S00", kind="pit"),
                 R("corroborate", "drone_1", cell="S00")])
    assert fleet[0].claimed == {}
    assert fleet[0].unreachable == {"S00"}
    print("ok  confirmed_hazard_releases_the_claim")


def test_unhealthy_agent_is_not_a_witness():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("health", "drone_1", seq=1, ok=False),
                 R("hazard", "drone_0", cell="S00", kind="pit"),
                 R("corroborate", "drone_1", cell="S00")])
    assert fleet[0].confirmed_hazards == set()
    assert fleet[0].provisional_hazards() == ["S00"]
    print("ok  unhealthy_agent_is_not_a_witness")


def test_hazard_on_explored_sector_is_moot():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00"),
                 R("explored", "bot_0", sector="S00"),
                 R("hazard", "drone_0", cell="S00", kind="pit"),
                 R("corroborate", "drone_1", cell="S00")])
    assert fleet[0].explored == {"S00"}
    assert fleet[0].unreachable == set(), "condemned a sector already covered"
    print("ok  hazard_on_explored_sector_is_moot")


def test_hazard_outside_the_grid_ignored():
    fleet = fresh_fleet()
    feed(fleet, [R("hazard", "drone_0", cell="S99", kind="pit"),
                 R("corroborate", "drone_1", cell="S99")])
    assert fleet[0].unreachable == set()
    assert fleet[0].hazard_reports == {}
    print("ok  hazard_outside_the_grid_ignored")


# ---------------------------------------------------------------- health ---

def test_health_is_cross_tier():
    """A bot suspects a silent drone, and an unhealthy drone loses its block."""
    fleet = fresh_fleet()
    feed(fleet, [R("survey_claim", "drone_1", block="B00"),
                 R("health", "drone_1", seq=4, ok=True)])
    assert fleet[0].block_claims == {"B00": "drone_1"}
    feed(fleet, [R("suspect", "bot_0", victim="drone_1", seen_seq=4)])
    assert fleet[0].unhealthy == {"drone_1"}
    assert fleet[0].block_claims == {}, "unhealthy drone kept its block"
    print("ok  health_is_cross_tier")


def test_late_beacon_voids_suspicion():
    fleet = fresh_fleet()
    feed(fleet, [R("health", "bot_1", seq=4, ok=True),
                 R("health", "bot_1", seq=5, ok=True),
                 R("suspect", "drone_0", victim="bot_1", seen_seq=4)])
    assert fleet[0].unhealthy == set()
    print("ok  late_beacon_voids_suspicion")


def test_stale_beacon_ignored():
    fleet = fresh_fleet()
    feed(fleet, [R("health", "bot_0", seq=7, ok=True),
                 R("health", "bot_0", seq=3, ok=False)])
    assert fleet[0].unhealthy == set()
    assert fleet[0].health_seq["bot_0"] == 7
    print("ok  stale_beacon_ignored")


def test_readmission_restores_claiming():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("health", "bot_0", seq=1, ok=False),
                 R("claim", "bot_0", sector="S00")])
    assert fleet[0].claimed == {}
    feed(fleet, [R("health", "bot_0", seq=2, ok=True),
                 R("claim", "bot_0", sector="S00")])
    assert fleet[0].claimed == {"S00": "bot_0"}
    print("ok  readmission_restores_claiming")


# ----------------------------------------------------------------- phase ---

def test_done_needs_both_tiers():
    fleet = fresh_fleet()
    # Survey everything but sweep nothing: not done.
    log = []
    for b in BLOCKS:
        log += survey("drone_0", b)
        log.append(R("surveyed", "drone_0", block=b, cells=BLOCK_CELLS[b]))
    feed(fleet, log)
    assert fleet[0].surveyed_blocks == set(BLOCKS)
    assert fleet[0].phase == SURVEYING, "done with ground work outstanding"
    # Now sweep every sector.
    for s in SECTORS:
        feed(fleet, [R("claim", "bot_0", sector=s),
                     R("explored", "bot_0", sector=s)])
    assert fleet[0].phase == DONE
    assert_agreement(fleet, "termination")
    print("ok  done_needs_both_tiers")


def test_condemned_sector_still_terminates():
    fleet = fresh_fleet()
    log = []
    for b in BLOCKS:
        log += survey("drone_0", b)
    # S00 is a real pit: two witnesses condemn it, the rest get swept.
    log += [R("hazard", "drone_0", cell="S00", kind="pit"),
            R("corroborate", "bot_1", cell="S00")]
    for s in SECTORS:
        if s == "S00":
            continue
        log += [R("claim", "bot_0", sector=s), R("explored", "bot_0", sector=s)]
    feed(fleet, log)
    assert fleet[0].unreachable == {"S00"}
    assert fleet[0].phase == DONE
    print("ok  condemned_sector_still_terminates")


# ------------------------------------------------------------------ epoch ---

def test_reset_wipes_both_tiers():
    fleet = fresh_fleet()
    feed(fleet, survey("drone_0", "B00"))
    feed(fleet, [R("claim", "bot_0", sector="S00"),
                 R("hazard", "drone_0", cell="S01", kind="pit")])
    feed(fleet, [{"op": "reset", "epoch": 1}])
    f = fleet[0]
    assert f.epoch == 1
    assert (f.surveyed, f.surveyed_blocks, f.claimed, f.hazard_reports) == \
        (set(), set(), {}, {})
    # stale-epoch records are ignored
    feed(fleet, [R("claim", "bot_0", sector="S00", epoch=0)])
    assert f.claimed == {}
    # and the new epoch works
    feed(fleet, [R("survey_claim", "drone_0", block="B00", epoch=1)])
    assert f.block_claims == {"B00": "drone_0"}
    assert_agreement(fleet, "reset")
    print("ok  reset_wipes_both_tiers")


# -------------------------------------------------------------- the point ---

def test_mixed_fleet_derives_identical_state():
    """The headline: bots and drones fold one log into one state.

    fleet[0:2] stand for the two tessera bots, fleet[2:4] for the two Rust
    drones. Agreement is checked at EVERY prefix, so a divergence is caught at
    the record that caused it rather than at the end.
    """
    fleet = fresh_fleet(4)
    log = [
        R("health", "bot_0", seq=0, ok=True),
        R("health", "bot_1", seq=0, ok=True),
        R("health", "drone_0", seq=0, ok=True),
        R("health", "drone_1", seq=0, ok=True),
        # both drones grab blocks, one collides and loses
        R("survey_claim", "drone_0", block="B00"),
        R("survey_claim", "drone_1", block="B00"),
        R("survey_claim", "drone_1", block="B01"),
        # a bot jumps the gun before any survey lands
        R("claim", "bot_0", sector="S00"),
        R("surveyed", "drone_0", block="B00", cells=BLOCK_CELLS["B00"]),
        R("surveyed", "drone_1", block="B01", cells=BLOCK_CELLS["B01"]),
        # now the ground tier may work
        R("claim", "bot_0", sector="S00"),
        R("claim", "bot_1", sector="S01"),
        # drone_0 flags a pit in S02; nobody has corroborated it yet
        R("hazard", "drone_0", cell="S02", kind="pit"),
        R("explored", "bot_0", sector="S00"),
        # bot_1 drives into S02 anyway and stalls: second witness
        R("abandon", "bot_1", sector="S01"),
        R("claim", "bot_1", sector="S02"),
        R("corroborate", "bot_1", cell="S02"),
        # drone_0 runs low and hands its work back
        R("rtb", "drone_0"),
        R("survey_claim", "drone_1", block="B02"),
        # bot_1 goes quiet and is suspected
        R("suspect", "drone_1", victim="bot_1", seen_seq=0),
    ]
    for k in range(len(log)):
        feed(fleet, [log[k]])
        assert_agreement(fleet, f"prefix {k}")

    f = fleet[0]
    assert f.explored == {"S00"}
    assert f.unreachable == {"S02"}, "the corroborated pit was not condemned"
    assert f.confirmed_hazards == {"S02"}
    assert f.grounded == {"drone_0"}
    assert f.unhealthy == {"bot_1"}
    assert f.claimed == {}, "suspected bot kept a claim"
    assert f.block_claims == {"B02": "drone_1"}
    print("ok  mixed_fleet_derives_identical_state")


def test_lying_drone_cannot_deny_service():
    """Phase B: phantom hazards from one agent never condemn anything, so the
    fleet still finishes every sector."""
    fleet = fresh_fleet()
    log = []
    for b in BLOCKS:
        log += survey("drone_0", b)
    # drone_1 invents a hazard in every single sector.
    for s in SECTORS:
        log.append(R("hazard", "drone_1", cell=s, kind="pit"))
    feed(fleet, log)
    assert fleet[0].unreachable == set(), "a single liar condemned the map"
    assert len(fleet[0].provisional_hazards()) == len(SECTORS)
    # every sector is still claimable, so the sweep completes
    for s in SECTORS:
        feed(fleet, [R("claim", "bot_0", sector=s),
                     R("explored", "bot_0", sector=s)])
    assert fleet[0].phase == DONE
    print("ok  lying_drone_cannot_deny_service")


def test_conformance_fixture():
    """Replay fixtures/conformance.json and check every snapshot.

    This is the Python half of the cross-language contract. air_agent's
    `cargo test` replays the same file through the Rust fold, so if the two
    implementations drift, one of the two halves goes red at the exact record
    that caused it. Regenerate with fixtures/gen_conformance.py.
    """
    import json
    import os

    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "fixtures", "conformance.json")
    with open(path) as fh:
        doc = json.load(fh)

    st = AirGroundState(doc["sectors"], doc["blocks"],
                        doc["block_cells"], doc["agents"])
    assert canonical_snapshot(st) == doc["snapshots"][0], \
        "initial state does not match the fixture"
    for i, rec in enumerate(doc["log"]):
        st.apply(rec)
        got, want = canonical_snapshot(st), doc["snapshots"][i + 1]
        assert got == want, (
            f"snapshot {i + 1} differs after {rec}\n  got  {got}\n  want {want}")
    print(f"ok  conformance_fixture ({len(doc['log'])} records)")


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print(f"\nall {len(tests)} airground_fsm tests passed")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        print(f"FAIL: {e}")
        sys.exit(1)

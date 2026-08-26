#!/usr/bin/env python3
"""Generate fixtures/conformance.json — the cross-language fold contract.

The air/ground fold exists twice: once in Python (nodes/airground_fsm.py, what
the tessera bots run) and once in Rust (air_agent/src/fold.rs, what the drones
run). If the two ever disagree, the simulation's whole claim collapses, and
they would disagree silently.

So this writes one file holding a scripted record log plus the canonical state
snapshot after EVERY record. Both implementations replay it and assert every
snapshot matches:

    python3 nodes/test_airground_fsm.py          # Python side
    cd air_agent && cargo test                   # Rust side

Regenerate after any intentional fold change, and read the diff carefully:
a change here is a change to the protocol both tiers implement.

    python3 fixtures/gen_conformance.py
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nodes"))

from airground_fsm import (AirGroundState, canonical_snapshot,  # noqa: E402
                           make_blocks, make_sectors)

OUT = os.path.join(HERE, "conformance.json")

# Mirrors the launch geometry (airground.launch.py) and the unit tests.
GEOM = {"nx": 4, "ny": 3, "min_x": -20.0, "min_y": -15.0,
        "cell_w": 10.0, "cell_h": 10.0, "block_w": 2, "block_h": 1}
AGENTS = ["bot_0", "bot_1", "drone_0", "drone_1"]


def build_log(sectors, blocks, block_cells):
    """A log that exercises every rule at least once, including the ones that
    are supposed to do nothing. Order matters: several records are here
    precisely because they must be rejected at that point in the log."""
    L = []

    def rec(op, agent=None, epoch=0, **kw):
        d = {"op": op, "epoch": epoch, **kw}
        if agent is not None:
            d["agent"] = agent
        L.append(d)

    # everyone reports in
    for a in AGENTS:
        rec("health", a, seq=0, ok=True)

    # air: a claim collision, and a drone trying to hold two blocks
    rec("survey_claim", "drone_0", block="B00")
    rec("survey_claim", "drone_1", block="B00")      # loses
    rec("survey_claim", "drone_0", block="B01")      # already holds one
    rec("survey_claim", "drone_1", block="B01")

    # ground jumps the gun before any survey has landed: must be refused
    rec("claim", "bot_0", sector="S00")

    # a non-holder reporting a survey: must be refused
    rec("surveyed", "bot_0", block="B00", cells=block_cells["B00"])
    # a holder over-reporting: clipped to its own block
    rec("surveyed", "drone_0", block="B00", cells=list(sectors))
    rec("surveyed", "drone_1", block="B01", cells=block_cells["B01"])

    # now ground work is legal
    rec("claim", "bot_0", sector="S00")
    rec("claim", "bot_1", sector="S00")              # loses
    rec("claim", "bot_1", sector="S01")
    rec("explored", "bot_0", sector="S99")           # not a sector
    rec("explored", "bot_1", sector="S00")           # not the holder
    rec("explored", "bot_0", sector="S00")

    # evidence: one witness is provisional
    rec("hazard", "drone_0", cell="S02", kind="pit", x=-5.0, y=-5.0)
    rec("hazard", "drone_0", cell="S02", kind="pit", x=-5.0, y=-5.0)  # same one
    rec("corroborate", "drone_0", cell="S02")        # still the same one
    # a hazard outside the grid
    rec("hazard", "drone_1", cell="S99", kind="pit", x=0.0, y=0.0)

    # survey the block containing S02/S03 so it becomes claimable
    rec("survey_claim", "drone_0", block="B01")      # B01 already surveyed
    rec("survey_claim", "drone_0", block="B02")
    rec("surveyed", "drone_0", block="B02", cells=block_cells["B02"])

    # bot_1 drives in and stalls: the second, distinct witness confirms
    rec("abandon", "bot_1", sector="S01")
    rec("claim", "bot_1", sector="S02")
    rec("corroborate", "bot_1", cell="S02")

    # battery lease
    rec("survey_claim", "drone_0", block="B03")
    rec("rtb", "drone_0")
    rec("survey_claim", "drone_0", block="B04")      # grounded, refused
    rec("survey_claim", "drone_1", block="B03")      # picks up the release
    rec("ready", "drone_0")
    rec("survey_claim", "drone_0", block="B04")

    # health across tiers
    rec("health", "bot_1", seq=1, ok=True)
    rec("suspect", "drone_1", victim="bot_1", seen_seq=0)   # stale, no-op
    rec("suspect", "drone_1", victim="bot_1", seen_seq=1)   # acts
    rec("hazard", "bot_1", cell="S03", kind="pit", x=5.0, y=-5.0)  # unhealthy
    rec("health", "bot_1", seq=2, ok=True)                  # readmitted
    rec("hazard", "bot_1", cell="S03", kind="pit", x=5.0, y=-5.0)  # counts now

    # an unhealthy drone loses its block
    rec("health", "drone_1", seq=1, ok=False)
    rec("health", "drone_1", seq=2, ok=True)

    # epoch bump wipes both tiers, and stale records are ignored after it
    rec("reset", epoch=1)
    rec("claim", "bot_0", sector="S00", epoch=0)
    rec("survey_claim", "drone_0", block="B00", epoch=1)

    return L


def main():
    sectors, _ = make_sectors(GEOM["nx"], GEOM["ny"], GEOM["min_x"],
                              GEOM["min_y"], GEOM["cell_w"], GEOM["cell_h"])
    blocks, block_cells = make_blocks(GEOM["nx"], GEOM["ny"],
                                      GEOM["block_w"], GEOM["block_h"])
    log = build_log(sectors, blocks, block_cells)

    st = AirGroundState(sectors, blocks, block_cells, AGENTS)
    snapshots = [canonical_snapshot(st)]          # index 0 = before any record
    for r in log:
        st.apply(r)
        snapshots.append(canonical_snapshot(st))

    doc = {
        "_comment": ("Cross-language fold contract for simulation 3. "
                     "Generated by fixtures/gen_conformance.py; replayed by "
                     "nodes/test_airground_fsm.py and air_agent/src/fold.rs "
                     "tests. snapshots[0] is the initial state; snapshots[i+1] "
                     "is the state after log[i]."),
        "geometry": GEOM,
        "agents": AGENTS,
        "sectors": sectors,
        "blocks": blocks,
        "block_cells": block_cells,
        "log": log,
        "snapshots": snapshots,
    }
    with open(OUT, "w") as f:
        json.dump(doc, f, indent=1, sort_keys=True)
        f.write("\n")
    print(f"wrote {OUT}: {len(log)} records, {len(snapshots)} snapshots")


if __name__ == "__main__":
    main()

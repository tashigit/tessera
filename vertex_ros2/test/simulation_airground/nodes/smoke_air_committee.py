#!/usr/bin/env python3
"""Four air_agent processes in one real Vertex committee, no ROS anywhere.

This is the air tier standing on its own: four native Rust agents, four mock
airframes, real engines gossiping over real UDP. It proves the drone side end
to end (engine bring-up, record encoding, the fold, the survey loop) without
needing Docker or a ROS install, so the Rust half can be debugged in seconds
rather than through a container.

It is NOT the headline test. The headline is the mixed fleet, where two of the
four peers are tessera vertex_node processes with a Python fold; that needs
ROS and lives in airground.launch_test.py. What this catches is everything
that would break there for purely Rust-side reasons.

    python3 nodes/smoke_air_committee.py [--seconds 45]

Exit 0 means all four agents converged on identical folded state.
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
PEERS = os.path.join(SIM, "fixtures", "peers_airground.json")
AGENT = os.path.join(SIM, "air_agent", "target", "debug", "air_agent")
LOGDIR = os.path.join(SIM, "logs", "smoke")

# Craters planted under the mock airframes' rangers. S02 and S05 are real, so
# an honest survey must flag them; nothing else is.
PITS = "S02,S05"


def load_peers():
    with open(PEERS) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=45.0)
    args = ap.parse_args()

    if not os.path.exists(AGENT):
        print(f"missing {AGENT}\n  build it: (cd {SIM}/air_agent && cargo build)",
              file=sys.stderr)
        return 2
    if not os.path.exists(PEERS):
        print(f"missing {PEERS}\n  generate it: bash fixtures/gen_peers_airground.sh",
              file=sys.stderr)
        return 2

    peers = load_peers()
    shutil.rmtree(LOGDIR, ignore_errors=True)
    os.makedirs(LOGDIR, exist_ok=True)

    procs = []
    try:
        # Every peer runs an air_agent here, including the two the real
        # simulation gives to tessera bots. Consensus does not care which
        # tier a peer is on, which is exactly what makes the mixed fleet work.
        for i, me in enumerate(peers):
            link_port = 48631 + i
            others = [p for j, p in enumerate(peers) if j != i]
            cmd = [AGENT,
                   "--id", me["name"],
                   "--bind", me["addr"],
                   "--key", me["secret"],
                   "--link", f"127.0.0.1:{link_port}",
                   "--log", os.path.join(LOGDIR, f"{me['name']}.log")]
            for p in others:
                cmd += ["--peer", f"{p['public']}@{p['addr']}"]
            out = open(os.path.join(LOGDIR, f"{me['name']}.stderr"), "w")
            procs.append(subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT))

        time.sleep(1.5)      # let the agents bind their controller sockets

        for i, me in enumerate(peers):
            link_port = 48631 + i
            # Spread the spawn points so the drones do not all race for B00.
            start = f"{-20 + 12 * i},{-15 + 6 * (i % 2)}"
            cmd = [sys.executable, os.path.join(HERE, "mock_airframe.py"),
                   "--link", f"127.0.0.1:{link_port}",
                   "--start", start, "--pits", PITS]
            out = open(os.path.join(LOGDIR, f"{me['name']}.airframe"), "w")
            procs.append(subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT))

        deadline = time.time() + args.seconds
        while time.time() < deadline:
            if converged(peers):
                break
            time.sleep(1.0)
    finally:
        for p in procs:
            p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

    return report(peers)


def last_state(name):
    """The final STATE line a given agent journalled, as a dict."""
    path = os.path.join(LOGDIR, f"{name}.log")
    if not os.path.exists(path):
        return None
    last = None
    with open(path) as f:
        for line in f:
            if line.startswith("STATE "):
                last = line[len("STATE "):].strip()
    return json.loads(last) if last else None


def converged(peers):
    """Every agent has folded state, they all agree, and the whole map is
    surveyed. Used only to end the run early; the real verdict is report()."""
    snaps = [last_state(p["name"]) for p in peers]
    if any(s is None for s in snaps):
        return False
    if any(s != snaps[0] for s in snaps):
        return False
    return snaps[0]["phase"] == "done" or len(snaps[0]["surveyed_blocks"]) == 6


def report(peers):
    snaps = {p["name"]: last_state(p["name"]) for p in peers}
    missing = [n for n, s in snaps.items() if s is None]
    if missing:
        print(f"FAIL: no folded state journalled by {missing}", file=sys.stderr)
        print(f"      see {LOGDIR}/*.stderr", file=sys.stderr)
        return 1

    first = snaps[peers[0]["name"]]
    diverged = {n: s for n, s in snaps.items() if s != first}
    if diverged:
        print("FAIL: agents disagree on folded state", file=sys.stderr)
        for n, s in diverged.items():
            print(f"  {n}: {json.dumps(s, sort_keys=True)}", file=sys.stderr)
        print(f"  {peers[0]['name']}: {json.dumps(first, sort_keys=True)}", file=sys.stderr)
        return 1

    surveyed_blocks = first["surveyed_blocks"]
    hazards = first["hazard_reports"]
    print(f"all {len(peers)} agents agree")
    print(f"  blocks surveyed : {surveyed_blocks}")
    print(f"  sectors cleared : {first['surveyed']}")
    print(f"  hazard reports  : {json.dumps(hazards, sort_keys=True)}")
    print(f"  confirmed       : {first['confirmed_hazards']}")

    if not surveyed_blocks:
        print("FAIL: no block was ever surveyed, the survey loop is not running",
              file=sys.stderr)
        return 1

    # Every crater actually flown over must have been sighted by whoever
    # surveyed it. Blocks nobody reached in the time budget are not a failure.
    reached = set(first["surveyed"])
    for pit in PITS.split(","):
        if pit in reached and pit not in hazards:
            print(f"FAIL: {pit} was surveyed but no drone reported the crater",
                  file=sys.stderr)
            return 1
    print("CONSENSUS VERIFIED: one ordered history, identical fold on every agent")
    return 0


if __name__ == "__main__":
    sys.exit(main())

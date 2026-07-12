#!/usr/bin/env python3
"""Verify the per-node consensus logs agree — proof the bots are real Vertex
nodes sharing one consensus.

Each mission_coordinator writes logs/robot_<i>_consensus.log recording every
consensus event it was delivered (with the Vertex event hash), every transaction
it submitted, and every decision it took. If all four bots are genuine peers of
one Vertex session, the SEQUENCE OF EVENT HASHES in the four files must be
identical (each node may be a few events ahead/behind at shutdown — prefixes are
compared).

Usage:  python3 nodes/verify_consensus_logs.py [logs-dir] [pattern]
`pattern` defaults to robot_*_consensus.log (route exploration); the arena
scenario writes robot_*_arena.log, so pass that to verify its runs.
Exit 0 and "CONSENSUS VERIFIED" if all common prefixes match; exit 1 otherwise.
"""

import re
import sys
from pathlib import Path


def hash_sequence(path: Path):
    seq = []
    for line in path.read_text().splitlines():
        m = re.search(r"EVENT #\d+ hash=([0-9a-f]+)", line)
        if m:
            seq.append(m.group(1))
    return seq


def main():
    log_dir = Path(sys.argv[1] if len(sys.argv) > 1
                   else Path(__file__).resolve().parent.parent / "logs")
    pattern = sys.argv[2] if len(sys.argv) > 2 else "robot_*_consensus.log"
    files = sorted(log_dir.glob(pattern))
    if len(files) < 2:
        print(f"need >=2 log files in {log_dir}, found {len(files)}")
        return 1

    seqs = {f.name: hash_sequence(f) for f in files}
    for name, seq in seqs.items():
        print(f"{name}: {len(seq)} consensus events")

    ref_name, ref = max(seqs.items(), key=lambda kv: len(kv[1]))
    ok = True
    for name, seq in seqs.items():
        if name == ref_name:
            continue
        n = min(len(seq), len(ref))
        if seq[:n] != ref[:n]:
            for i in range(n):
                if seq[i] != ref[i]:
                    print(f"DIVERGENCE at event #{i + 1}: "
                          f"{ref_name}={ref[i]} vs {name}={seq[i]}")
                    break
            ok = False
        else:
            print(f"{name}: identical order with {ref_name} "
                  f"over {n} common events")
    if ok:
        print("CONSENSUS VERIFIED: all nodes delivered the same ordered "
              "event stream (byte-level hashes match)")
        return 0
    print("CONSENSUS MISMATCH — nodes are NOT sharing one ordered stream")
    return 1


if __name__ == "__main__":
    sys.exit(main())

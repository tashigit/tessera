# Simulation 3: air/ground survey-and-sweep (`airground_arena`)

Status: **v0.1 (as built)**. Third application-level simulation on the
`vertex_ros2` ↔ Vertex stack, alongside route exploration
(`../simulation/README.md`) and arena exploration
(`../simulation_arena/README.md`). Two Pioneer 3-AT sweepers and two DJI
Mavic 2 PRO surveyors clear the arena together.

The first two simulations both show one homogeneous fleet of Python agents on
`vertex_fleet`. This one exists to show two things neither can.

**Heterogeneous clients, one committee.** The bots join consensus through
tessera: `vertex_node` plus a Python fold on the `vertex_fleet` API. The
drones join by linking the `tashi-vertex` crate straight into a Rust binary,
with no rclrs, no DDS, no message packages and no rosbridge anywhere in the
process. All four are peers in one hashgraph folding one ordered log. If they
agree, tessera's contract is demonstrably a thin ROS adapter over the
protocol rather than a dialect of its own.

**Sensor asymmetry that consensus has to resolve.** The arena has sixteen
craters, and simulation 2's README records the flaw honestly: they are
invisible to a horizontal Sick LMS 291, and a Pioneer that drives into one is
trapped for good. A drone looking down sees exactly what the ground lidar
cannot. So the ground tier physically depends on the air tier for
information, and the ordered log is what turns that dependency into a map
both bots agree on. Simulation 3 takes simulation 2's documented limitation
and makes it the subject.

---

## 1. Scenario

The same interesting region as the arena scenario (x, y in [-20, 20] x
[-15, 15]; walls at ±25) is divided two ways at once. The ground tier claims
**sectors**, a 4x3 grid of 10 x 10 m cells. The air tier claims **survey
blocks**, six blocks of two sectors each. Blocks outnumber the drones, so
survey claims genuinely contend.

What consensus owns:

1. **Two claim namespaces, one exclusivity rule.** A drone claims an air
   `block`, a bot claims a ground `sector`. The first claim in consensus
   order wins, and an agent holds at most one thing at a time. The same
   primitive serves both vehicle classes with no special-casing.
2. **Ground work is gated on air work.** A sector is not claimable until some
   drone has reported `surveyed` for it. Before that it is unknown, and an
   idle bot with nothing surveyed nearby simply waits. This is a real
   dependency between the tiers, ordered by the log rather than by wall
   clock, and it is the thing a broker cannot give you: the bots are not
   waiting for a message, they are waiting for a position in an agreed
   history.
3. **Hazards need corroboration.** A `hazard` from one agent is
   *provisional*: it defers a sector but does not condemn it. A second
   **distinct** agent reporting the same cell promotes it to *confirmed*,
   which condemns it for good. Consensus makes everyone agree on what was
   *said*; corroboration is what turns that into what is *true*.
4. **Battery lease.** A drone low on flight time submits `rtb`. The fold
   releases its block to the other drone and grounds it until it reports
   `ready`. The same lease shape as the arena scenario's silence lease, with
   different physics behind it.
5. **Health, uniformly across tiers.** Every agent folds a periodic `health`
   beacon with a monotonic `seq` and a self-assessed `ok` from local sensor
   freshness. A crashed agent stops beaconing and any peer may propose
   `suspect`, which acts only if no newer beacon landed. A bot may suspect a
   drone and a drone may suspect a bot; the fold does not distinguish them.

Agent ids are **strings** here (`bot_0`, `drone_1`), not the integers the
arena scenario used: the fleet is mixed, so the id has to say which tier it
is on.

### Transaction payloads

Opaque JSON records on `VertexTransaction.payload` (all carry `epoch`;
`reset` and epoch gating come from `vertex_fleet.ReplicatedState`):

```jsonc
{ "op": "survey_claim", "agent": "drone_0", "block": "B03", "epoch": 0 }
{ "op": "surveyed",     "agent": "drone_0", "block": "B03",
                        "cells": ["S06", "S07"], "epoch": 0 }
{ "op": "hazard",       "agent": "drone_0", "cell": "S05",
                        "kind": "pit", "epoch": 0 }
{ "op": "corroborate",  "agent": "bot_1",   "cell": "S05", "epoch": 0 }
{ "op": "claim",        "agent": "bot_0",   "sector": "S06", "epoch": 0 }
{ "op": "explored",     "agent": "bot_0",   "sector": "S06", "epoch": 0 }
{ "op": "abandon",      "agent": "bot_0",   "sector": "S06", "epoch": 0 }
{ "op": "rtb",          "agent": "drone_0", "epoch": 0 }
{ "op": "ready",        "agent": "drone_0", "epoch": 0 }
{ "op": "health",       "agent": "drone_1", "seq": 41, "ok": true, "epoch": 0 }
{ "op": "suspect",      "agent": "bot_0",   "victim": "drone_1",
                        "seen_seq": 41, "epoch": 0 }
{ "op": "reset",        "epoch": 1 }
```

## 2. The fleet

```
 Native Webots (Mac)                Docker / Jazzy               Native Rust
 ───────────────────                ──────────────               ───────────
 pioneer_sweeper x2 ──rosbridge──▶  vertex_node x2
                        :9090       ground_coordinator x2
                                      (vertex_fleet, Python fold)
                                              │
                                              ├── one Vertex committee ──┐
                                              │      n = 4, f = 1        │
 mavic_surveyor x2 ──JSON lines──────────────────────────────────▶ air_agent x2
                     TCP :48633/4                                   (tashi-vertex
                                                                     direct, Rust
                                                                     fold, no ROS)
```

- **`ground_coordinator`** (`nodes/ground_coordinator.py`) extends
  `vertex_fleet.VertexAgent`; `AirGroundState` extends
  `vertex_fleet.ReplicatedState`. It adds the claim loop (which waits on the
  air tier), the physical outcome reports, corroboration, the health beacon
  and the silence lease.
- **`air_agent`** (`air_agent/`) is a native Rust binary. It links
  `tashi-vertex` at the same pinned version `vertex_core` uses, folds the log
  through its own `AirGroundState` in `src/fold.rs`, and flies a lawnmower
  pass over whichever block it holds. The FFI handles are `!Send`, so it runs
  on a current-thread runtime in one `select!` loop.
- **`pioneer_sweeper`** and **`mavic_surveyor`** (`controllers/`) are the
  Webots controllers, native on the host. Neither makes a coordination
  decision; they fly or drive where told and report what they sense.
- **`mock_pioneer`** and **`mock_airframe`** (`nodes/`) replace Webots
  headless, with straight-line kinematics and a scripted crater map.

### Why the drones deliberately do not use rosbridge

The Pioneers reach the ROS graph over rosbridge because they are already ROS
citizens. The air tier is not, and its only outward connection is a plain
newline-delimited JSON socket. That is the point: a drone needs the Vertex
wire protocol and the record schema, and nothing else, to be a full member of
the fleet. Routing it through rosbridge would work and would prove less.

### The one thing that must match

Both tiers link the **same engine protocol version**. `air_agent/Cargo.toml`
pins `tashi-vertex = "=0.14.0"`, the exact version `vertex_core` pins. A
mismatch between the drone binaries and `vertex_node` is the single mistake
that breaks the whole demonstration.

## 3. Keeping two folds honest

The fold now exists twice: `nodes/airground_fsm.py` for the bots and
`air_agent/src/fold.rs` for the drones. If they ever disagree the
simulation's central claim collapses, and they would disagree silently.

So they are pinned together by **`fixtures/conformance.json`**: a 45-record
log plus the canonical state snapshot after every single record. Both
implementations replay it and check every snapshot.

```bash
python3 nodes/test_airground_fsm.py        # Python side
(cd air_agent && cargo test)               # Rust side
```

Change one implementation and the other goes red at the exact record that
diverged. Regenerate with `python3 fixtures/gen_conformance.py` after any
intentional fold change, and read the diff carefully: a change there is a
change to the protocol both tiers implement.

This is the artifact worth stealing if you are porting a fold to a second
language. It is cheap, it is a plain JSON file, and it converts a silent
class of bug into a failing test.

## 4. Assertions (headless launch_tests)

`airground.launch_test.py`, 12 sectors in 6 blocks, one real crater at S05:

| Property | Pass condition |
|---|---|
| Cross-tier gate | no sector is ever observed claimed before it was surveyed |
| Coverage | the 11 reachable sectors explored, S05 unreachable, `phase == done` |
| Corroboration | S05 has exactly two distinct witnesses, one air and one ground |
| Claim exclusivity | no snapshot shows an agent holding two sectors |
| **Mixed-fleet agreement** | the drones' folded state, read from their own journals and produced by the **Rust** fold, equals the bots' `mission_state`, produced by the **Python** fold, field for field |
| Consensus | both `/vertex/event` streams byte-identical |
| Clean shutdown | exit codes 0 / `-SIGINT` / `-SIGTERM` |

`lying_drone.launch_test.py` is phase B, the Byzantine case. `drone_1` runs
with `--conduct phantom-hazards` and fabricates a crater in every cell it
surveys, in a world that has none:

| Property | Pass condition |
|---|---|
| No denial of service | all 12 sectors explored, `phase == done` |
| No false condemnation | `unreachable` and `confirmed_hazards` both empty |
| The lies are recorded | every fabricated cell is in the log, witnessed by the liar alone |
| Honest work unaffected | all 6 blocks still surveyed |

The mirror case, a drone reporting a real pit as clear, is covered by the
main test: there the bot that drives into the hole is itself the second
witness. Between the two, one lying drone can neither invent an obstacle nor
hide one.

The fold itself has 29 pure-Python unit tests in
`nodes/test_airground_fsm.py` and 13 Rust tests in `air_agent/src/`.

## 5. Running

### 5.1 Headless (CI; no Webots)

```bash
docker compose run --rm sim simtest        # all three simulations' suites
```

Or, inside the container, just this scenario:

```bash
python3 nodes/test_airground_fsm.py
(cd air_agent && cargo test)
launch_test airground.launch_test.py
launch_test lying_drone.launch_test.py
```

There is also an air-tier-only smoke test that needs no ROS and no Docker,
four real `air_agent` processes in a real committee over UDP. It is the
fastest way to debug the Rust half:

```bash
(cd air_agent && cargo build)
bash fixtures/gen_peers_airground.sh
python3 nodes/smoke_air_committee.py
```

### 5.2 Live, in Webots (the viewable run)

Same split as the other two scenarios: Webots runs natively on the Mac (no
arm64 Linux build), the ROS 2 / Vertex graph runs in the Jazzy container, and
the two meet over forwarded ports.

One-time host setup:

```bash
# the sweepers' rosbridge client needs websocket-client on the python3 Webots
# uses (launch Webots from a terminal so it is the same python3):
python3 -m pip install --break-system-packages websocket-client
docker compose build
```

**Terminal 1, the consensus graph** (rosbridge + 2x `vertex_node` + 2x
`ground_coordinator` + 2x `air_agent`; builds the workspace, cargo-builds the
drone binary and generates `fixtures/peers_airground.json` on first run):

```bash
docker compose run --rm --service-ports sim simairground
```

Wait for `Rosbridge WebSocket server started on port 9090` and for both
drones to print `listening for its Webots controller`. The coordinators
immediately beacon `ok: False`, which is correct: nothing is connected yet.

**Terminal 2, the world**, in native Webots (R2025a):

```bash
/Applications/Webots.app/Contents/MacOS/webots \
    vertex_ros2/test/simulation_airground/worlds/airground_arena.wbt
```

The first load takes a while: the world pulls its R2025a assets and generates
the 16 procedural pit meshes. Later loads are much faster (assets cache under
`~/Library/Caches/Cyberbotics`).

### 5.3 What you should see

1. Two Pioneers staged on the west side, two Mavics on the ground at opposite
   corners.
2. Both drones take off to 12 m and begin lawnmower passes over the blocks
   consensus assigned them. The Pioneers sit still: **this is correct**, and
   it is the cross-tier gate you are watching. Nothing is claimable until the
   air tier reports it surveyed.
3. As each block is surveyed the sectors under it become claimable and the
   Pioneers start sweeping, steering around trees and rocks on lidar.
4. A Pioneer sent toward a crater drives to the rim and stops making
   progress. After `stall_sec` it reports `abandon`, and `corroborate` if a
   drone had already flagged that cell. That second witness condemns the
   sector, and the fleet finishes without it.
5. Done when every block is surveyed and every sector is explored or
   condemned.

Watch the shared state live from any agent:

```bash
docker exec -it $(docker ps -q -f name=tessera-sim-run | head -1) bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
   && ros2 topic echo /bot_0/mission_state'
```

### 5.4 Verifying a run

Every agent writes a per-node journal to `logs/`, bots and drones alike, in
the same EVENT/STATE/TX/DECIDE shape the other two simulations use. The most
interesting line is a drone's final `STATE`: it is the **Rust** fold's view,
and it names which **Python** bot explored each sector.

```bash
tail -1 logs/drone_0_airground.log
```

(Delete `logs/*.log` before a fresh run: the files append across runs.)

## 6. Assumptions & limitations (v0.1)

- **Pose is GPS ground truth**, as in the other two scenarios. The property
  under test is the coordination fabric, not SLAM.
- **Pit detection is geometric, not vision-based.** The drones carry a
  downward `DistanceSensor` in the Mavic proto's `bodySlot`, and a crater
  reads as ground further away than the drone's altitude. This keeps the
  signal deterministic and the assertions crisp. A real detector on the
  gimbal camera would change nothing in the protocol: the fold's
  corroboration rule is the point, not how a hazard was first spotted.
- **Battery is a software model**, not the Mavic proto's `battery` field, so
  the `rtb` lease fires at reproducible times in both the live world and the
  headless mock.
- **The drones do not avoid each other or the trees.** They fly at 12 m,
  above the canopy, on a fixed pass. Air-tier collision avoidance is out of
  scope for a coordination-fabric harness; sector and block exclusivity keep
  them apart at the task level, which is the layer under test.
- **Corroboration is two witnesses, flat.** There is no weighting by tier, no
  decay, and no way to retract a hazard. A world with a third tier, or with
  hazards that come and go, would want more. Two is enough to show the
  difference between agreeing on a claim and believing it.
- **Byzantine conduct is limited to what the app layer can express.** The
  `--conduct` modes make a drone lie in its records. They do not make it
  equivocate at the consensus layer, which is Vertex's problem rather than
  this simulation's, and is covered by the engine's own tests.

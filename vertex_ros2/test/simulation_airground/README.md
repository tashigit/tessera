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

**Sensor asymmetry that consensus has to resolve.** Simulation 2's README
records a flaw honestly: a crater is invisible to a horizontal Sick LMS 291,
and a Pioneer that drives into one is trapped for good. A drone looking down
sees exactly what the ground lidar cannot. So the ground tier physically
depends on the air tier for information, and the ordered log is what turns
that dependency into a map both bots agree on. Simulation 3 takes that
documented limitation and makes it the subject.

Doing so needed a real hole to point at, which the arena turned out not to
have: its sixteen `Pit` nodes are berms, not craters, measured at +0.40 to
+1.14 m and never negative. This world therefore generates its own ground.
See §6.

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
  on a current-thread runtime as two cooperative tasks: a recv loop that is
  never cancelled, because `recv_message()` is not cancellation-safe and
  `select!` dropping it corrupts the heap, and a control loop that selects
  only over cancellation-safe futures. This is `vertex_core`'s shape.
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

## 3. Keeping three folds honest

The fold exists three times: `nodes/airground_fsm.py` for the bots,
`air_agent/src/fold.rs` for the drones, and `viewer/airground_fold.js` so the
viewer can decode `/vertex/event` itself rather than trust a summary. If any
two disagree the simulation's central claim collapses, and they would
disagree silently.

So they are pinned together by **`fixtures/conformance.json`**: a 45-record
log plus the canonical state snapshot after every single record. Both
implementations replay it and check every snapshot.

```bash
python3 nodes/test_airground_fsm.py        # Python
(cd air_agent && cargo test)               # Rust
node viewer/airground_fold.test.js         # JavaScript
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

`false_clear.launch_test.py` is the mirror case, and the more dangerous one.
`drone_1` runs with `--conduct false-clear`: it flies over a real crater, its
ranger sees it, and it reports the block clean anyway.

Inventing a hazard is cheap to defend against, because nobody corroborates a
fiction. Hiding one is not, because the fleet has no reason to doubt a clear
report and the ground tier is blind to exactly this kind of obstacle. So the
defence cannot be "detect the lie". It is that the truth arrives anyway, from
the other tier:

| Property | Pass condition |
|---|---|
| The lie really lands | no drone ever witnesses the crater, though both flew over it |
| The truth arrives anyway | the ground tier witnesses it, and no sector is left unexamined |
| Coverage still completes | every reachable sector swept, every sector resolved |
| Agreement holds | both bots reach the same verdict about the hidden crater |

The liar's own journal is the clearest evidence that the conduct is real
rather than a drone that simply missed something. It reads:

```
DECIDE surveyed B02, 1 sighting(s)
```

Its ranger saw the crater. It reported the block clean regardless. The fold
ends the run with `{"S05": ["bot_0", "bot_1"]}`: no air witness at all, the
crater condemned by the two robots that could not cross it.

The cost of the lie is worth stating precisely: one robot's time, and a sector
that took a physical attempt to condemn instead of a 12 m flyover. What the
lie cannot do is corrupt the shared map or stall the mission.

Between the two tests, one lying drone can neither invent an obstacle nor hide
one.

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
launch_test false_clear.launch_test.py
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
4. Flagged sectors go last. A hazard with one witness is a preference, not a
   veto, so the bots clear everything else first and give a second witness
   time to arrive.
5. A Pioneer eventually attempts the flagged sector and drives into the
   crater. It does not report the sector explored even once it is within
   `cover_radius` of the centre, because it is below grade and falling in is
   not coverage. Unable to climb out, after `immobilized_sec` it reports
   `corroborate` for the sector it is stuck in and stops claiming. That is the
   second witness, so the sector is condemned, and the bot degrades the fleet
   rather than starving it.
6. Sectors blocked by something other than a crater get condemned too, just
   more slowly: a tree cluster stops a Pioneer without any drone seeing it, so
   it takes `max_attempts` failures from each of two bots.
7. Done when every block is surveyed and every sector is explored or
   condemned. Expect a bot to finish the run sitting in the crater.

Watch the shared state live from any agent:

```bash
docker exec -it $(docker ps -q -f name=tessera-sim-run | head -1) bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
   && ros2 topic echo /bot_0/mission_state'
```

### 5.4 The live viewer

`viewer/airground_viewer.html` is a self-contained page (no build step, no
external dependency) that connects to the same rosbridge the Webots
controllers use. Open it, press Connect, and the defaults match the live
launch.

```bash
open vertex_ros2/test/simulation_airground/viewer/airground_viewer.html
```

It shows what the arena's viewer cannot: a fleet split across two tiers. Air
blocks are drawn as a backdrop under the ground sectors, unsurveyed sectors
are dark so the cross-tier gate is visible as the map fills in behind the
drones, and every hazard is badged with its witness count, amber at one and
red at two.

**Click any sector** for its provenance. State tells you what a sector is; the
inspector tells you how it got there and who did it. Which drone surveyed it,
which bots raced for the claim and which lost, how many times each gave up,
who witnessed a hazard and in what order, who finally reached the centre. A
typical sector reads:

```
block        B00 (S00, S01)
surveyed by  drone_0 from the air
claims       bot_0 tried, lost the race     <- before the survey landed
             bot_0 claimed it
             bot_1 tried, lost the race
explored by  bot_0 reached the centre
now          explored
```

Each agent card also carries a running tally of what it has actually
contributed to the log, which is a different question from what it currently
holds: surveys flown and hazards sighted for a drone, sectors covered and
given up for a bot, claims lost for either.

Two things about the drones are worth knowing. Their **agents** publish
nothing to ROS at all, which is the scenario's whole claim, so their column in
the consensus matrix is this page's own fold of `/vertex/event`, decoded in
JavaScript by the same rules the Rust fold uses. Their **airframes** do
publish a pose for observers (`controllers/mavic_surveyor`'s
`TelemetryBeacon`), exactly as the Pioneers' controllers do, so the map can
show where they physically are. Nothing reads that back and no decision
depends on it; turn it off and the fleet behaves identically, the map just
falls back to drawing each drone over the block consensus says it holds.

That JS fold is a third implementation of the same rules, so it replays
`fixtures/conformance.json` too:

```bash
node viewer/airground_fold.test.js
```

One fixture, three languages, and a divergence fails at the exact record that
caused it.

### 5.5 Verifying a run

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
- **The world's ground is ours, not the arena's** (`worlds/gen_world.py`).
  Simulation 2's sixteen `Pit` nodes are not holes: the proto's height is
  `(1 - g) * g * size.z` with `g` a gaussian peaking at the centre, so it is
  zero at the centre, zero at the rim, and about 1.2 m in between. Measured by
  flying a drone transect, ground height across that arena ran +0.40 to
  +1.14 m and never went negative. A horizontal lidar sees a berm, so the
  premise of this whole scenario had no physical basis there. One elevation
  grid with one real crater replaces the flat floor and all sixteen pits, and
  is ~60x cheaper in collision triangles into the bargain. Everything else,
  trees rocks deer and footprint, is the arena's.
- **The crater is sized against `cover_radius`, not just for depth.** It is
  3 m deep over a 4.5 m radius. The radius is the subtle half: a bot credits a
  sector once it is within `cover_radius` (2 m) of the centre, so a crater
  narrower than that ring gets reported explored from flat ground and never
  met at all. It has to be WIDER than the credit radius. The depth and the
  46 degree walls are the other half, because at 30 degrees a Pioneer simply
  drives through.
- **A crater worth warning about will swallow a Pioneer**, and no profile
  avoids that: any bowl a robot can fall into is one it can drive to the
  middle of. So the bot does fall in, refuses to credit the sector while below
  grade, and reports being immobilized. The `immobilized_sec` guard then stops
  it claiming, so it degrades the fleet instead of starving it. That is also
  why there is one crater and not several: two can cost both bots, and the
  sweep never finishes.
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

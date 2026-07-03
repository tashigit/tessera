# Simulation Test Plan — `vertex_ros2` consensus-coordinated route exploration

Status: **v0.3 (as built)**. The v0.1 plan described a sequential, round-based
protocol; the implementation evolved to a **parallel** model (all bots move
concurrently, consensus assigns routes exclusively) and this document
describes what is actually built and verified. All five sub-scenarios
(N1-N5, §4) are implemented, and the headless suite runs in CI.
Scope: application-level acceptance of the `vertex_ros2` ↔ Vertex consensus
stack, driven by a fleet of 4 simulated robots in a physics world.

Scenario under test: **Consensus-coordinated route exploration**. Four robots
must get from A to B across a map with four alternative routes, not knowing
which are open. Every bot claims a route through Vertex; consensus order
arbitrates the claims so that no two bots ever hold the same route, and all
assigned bots probe their routes at the same time. A blocked bot reports the
outcome, returns, and claims another free route. The first arrival fixes the
winner route and every remaining bot converges onto it. Correct behaviour is a
leaderless, totally-ordered distributed decision loop of the kind Vertex
exists to provide.

---

## 1. Testing strategy across layers

Vertex's contract is *totally-ordered, cryptographically-final delivery of
opaque bytes*. Ordering correctness is independent of robot bodies, sensors,
and physics, so it is tested headless and must not be verified through a
simulator where flakiness and cost would swamp the signal. A robot simulator
is required only at the application layer, where the acceptance criteria are
physical.

| Layer | What it proves | Where it's tested | Simulator |
|---|---|---|---|
| **L0 — engine** | 3 real engines reach identical total order over UDP | `vertex_core/tests/multi_node.rs` | none |
| **L1 — ROS bridge** | `vertex_node` publishes events in Vertex order; byte-identical `/vertex/event` across peers; no publishes while `Inactive`; lifecycle cycles | `test/system_three_peers.launch_test.py` | headless `launch_test` |
| **L2 — endurance** | no RSS growth under 10-min load | `test/soak.launch_test.py` | headless |
| **L3 — application** | agreement on the ordered log yields exclusive route assignments, propagates the same outcomes, and converges every robot onto the same proven route, all driving **physical** motion | `route_exploration.launch_test.py` (headless) + the live Webots harness (this directory) | Webots for the live run; a mock replaces it headless |

L3 does not re-test consensus. Ordering, byte-identical streams, and
no-publish-in-`Inactive` are proven at L0/L1 far more cheaply. L3 *reuses* the
byte-identical assertion as a nearly-free cross-check but exists to validate
the physical, concurrent coordination. A green L1 with a red L3 points at the
application protocol or the simulation wiring, not the engine.

### 1.1 What the contract actually surfaces

Vertex is a hashgraph-family aBFT engine (gossip-about-gossip + virtual
voting). The application does **not** see the internal median-timestamp
voting; it receives the **finalized total order** of events plus per-event
`consensus_at` / `created_at` timestamps (fields on `VertexEvent`). Every
protocol decision below (which bot holds which route, which routes are
blocked, which route wins) is a pure function of that one ordered log, applied
identically on all four robots. The "line up the logs from all 4 machines and
they match" verification is exactly the byte-identical `/vertex/event`
assertion in `route_exploration.launch_test.py`, plus
`nodes/verify_consensus_logs.py` for live runs.

### 1.2 Simulator choice

Webots R2025a. On Apple Silicon it runs **natively on the host** (there is no
arm64 Linux build) while the ROS 2 / Vertex graph runs in the arm64 Jazzy
container; the two meet over rosbridge's WebSocket. See
`worlds/README.md` for the split architecture. The headless CI-facing test
replaces Webots with `nodes/mock_robot.py`, so the consensus loop is asserted
without any simulator.

---

## 2. Fleet size rationale — why 4 robots

Vertex is BFT with the standard `n ≥ 3f + 1` quorum:

| Peers `n` | Faults tolerated `f = ⌊(n−1)/3⌋` |
|---|---|
| 3 | **0** |
| **4** | **1** |
| 7 | 2 |

Four robots is the smallest fleet that tolerates one faulty/crashed robot,
which the planned fault-injection scenario (§4, N3) would exercise. Four
routes pair naturally with four robots (each bot has a "home lane" it prefers,
so initial departures are straight out). The scenario scales to `N ≥ 4`; 4 is
the documented minimum.

---

## 3. Scenario — parallel consensus-coordinated route exploration

Four routes `R1..R4` (lanes) run west to east; a closed route's barrier sits
at `x = 1.5`, deep in its corridor. All robots start staged on the west side,
one per lane mouth.

```
                 R1  y=+2.25  ── barrier @ x=1.5 ──┐
   staging  ─────R2  y=+0.75  ──────────────────── │──  goal line
   x=-4     ─────R3  y=-0.75  ────────────────────  │   x=3.6
                 R4  y=-2.25  ─────────────────────┘
   dividers at y = 1.5 / 0 / -1.5, corridor x in [-3, 3]
```

Every bot claims a route over Vertex; consensus assigns routes exclusively and
all assigned bots explore at once. Blocked bots report, return, and re-claim;
the first arrival fixes the winner and everyone converges onto it.

### 3.1 Protocol — a replicated state machine over the ordered log

Every robot maintains identical state, computed purely from `/vertex/event` in
consensus order (`nodes/mission_fsm.py`, no ROS/Webots dependencies, unit
tested on any host):

```
epoch          := current mission epoch (bumped by `reset`; stale records ignored)
assigned       := bot -> route, exclusive (first claim in consensus order wins)
blocked        := set of routes reported blocked
arrived        := set of bots that reached the end
winner_route   := route of the first `arrived` (None until one lands)
phase          := exploring | converging (winner set) | done (all arrived)
```

Rules, applied identically on every bot:

1. **Claim.** Every unassigned, unarrived bot claims a free route (its home
   lane first) every `claim_interval`. Claims are arbitrated by consensus
   order: the first claim for a route wins it, losers just claim again. No two
   bots are ever assigned the same route.
2. **Probe.** Every assigned bot drives its route concurrently with the
   others. Nobody waits on anybody.
3. **Report.** A bot that stops making progress inside its lane reports
   `blocked`: the route is marked blocked, its assignment is released
   immediately, and the bot returns to staging and claims another free route.
   A bot whose pose passes the goal line **on its claimed route's row**
   reports `arrived`.
4. **Converge.** The first `arrived` in consensus order fixes `winner_route`:
   all in-flight assignments clear and every remaining bot drives the winner
   route. Entries into the winner lane are staggered by a deterministic,
   consensus-derived rank (identical on every node) so the funnel never has
   two cars cornering at once.
5. **Re-open.** If the winner route is later reported blocked (the world
   changed), the winner clears and exploration re-opens.
6. **Recover.** When nothing is claimable and nobody is exploring, bots
   periodically re-claim a blocked route with `retry: true`, so the fleet
   recovers as soon as the user re-opens a route. When the supervisor
   broadcasts a world change, coordinators relay `unblock_all` into Vertex so
   stale blocked marks are dropped at the same consensus point on every bot.
7. **Lease.** A crashed explorer can neither report nor free its route. When
   another bot's assignment has produced no outcome for `lease_sec`, every
   live bot proposes a `timeout`; the first one in consensus order releases
   the assignment (the route becomes claimable again, not blocked) and the
   duplicates are no-ops. The lease restores progress; safety (exclusivity)
   never depended on it.
8. **Terminate.** `done` when every bot has arrived.

Because these rules are a deterministic function of the same byte-identical
ordered log, all four robots derive identical assignments, outcomes, and the
same winner, with no central coordinator.

### 3.2 Per-robot node graph

Each robot `i` runs, in namespace `/robot_i`:

```
 ┌───────────────────────────────────────────────────────────────────┐
 │ robot_i                                                             │
 │                                                                     │
 │  waypoint_follower (Webots controller, native on the host;          │
 │        │            bridges pose/barrier/drive over rosbridge)      │
 │        ▲ drive: route id | STAGING | STOP                           │
 │        │                                                            │
 │  mission_coordinator   (the replicated state machine, §3.1)         │
 │     │  claim / blocked / arrived / unblock_all / reset              │
 │     │        ──────────────────────────────▶  /robot_i/vertex/tx    │
 │     ◀── ordered log ──────────────────────    /robot_i/vertex/event │
 │                                                                     │
 │  vertex_node  (the crate under test; peers with the other 3)        │
 └───────────────────────────────────────────────────────────────────┘
```

- **`vertex_node`**: the binary under test. One per robot, peered with the
  other three over UDP (the L1 mesh, scaled 3 → 4).
- **`mission_coordinator`**: implements §3.1. Folds `/vertex/event` into the
  FSM, emits claims and physical outcomes, and translates its role
  (explore / converge / wait / done) into a drive command. Also writes a
  per-node consensus log (`logs/robot_<i>_consensus.log`) recording every
  delivered event hash, every submitted transaction, and every decision.
- **`waypoint_follower`**: drives the waypoint list of the assigned route (or
  holds at staging / parks at the goal). No Nav2 planner and no costmap:
  motion stays deterministic and assertions crisp. Physical bot-to-bot
  collision avoidance lives here (peer pose radio + deterministic rules; see
  `worlds/README.md`). Consensus only guarantees route exclusivity.

### 3.3 Transaction payloads

Vertex payloads are opaque (`VertexTransaction.payload`, `uint8[]`); the
coordinator encodes small JSON records. All records carry `epoch` so stale
messages from before a reset are ignored:

```jsonc
{ "op": "claim",   "bot": 2, "route": "R1", "epoch": 0 }
{ "op": "claim",   "bot": 2, "route": "R1", "epoch": 0, "retry": true }  // all-blocked recovery
{ "op": "blocked", "bot": 2, "route": "R1", "epoch": 0 }
{ "op": "arrived", "bot": 1, "route": "R2", "epoch": 0 }
{ "op": "timeout", "bot": 3, "victim": 2, "route": "R3", "epoch": 0 }  // lease expiry
{ "op": "unblock_all", "bot": 3, "seq": 2,  "epoch": 0 }  // user changed the barriers
{ "op": "reset",   "epoch": 1 }                           // supervisor reset: fresh epoch
```

Route ids are fixed labels shared with `config/routes.yaml`, so all peers key
state identically.

---

## 4. Sub-scenarios & assertions

| ID | Scenario | Status | Key assertions |
|---|---|---|---|
| **N1** | **Single blockage**: `R1` blocked, the rest open | **implemented** (`route_exploration.launch_test.py`, headless) | **All reach the end**: every bot ends `arrived`, `phase == done`. **Blocking discovered**: `R1` in every bot's `blocked` set. **Route exclusivity**: no snapshot ever shows two bots assigned one route. **Agreement**: single winner route on all bots, and all 4 `/vertex/event` streams byte-identical. **Clean shutdown**: exit codes 0 / `-SIGINT` / `-SIGTERM`. |
| **N2** | **Multiple blockages + stale-block stress**: one route open, then the open set flipped mid-mission | **implemented** (live harness: `WEBOTS_AUTOTEST=open<k>` and `turn3` in `route_supervisor.py`) | Fleet discovers the blocks, re-explores after `unblock_all`, converges, and every car physically reaches the goal area (`SUCCESS` line). Live collision monitor: no two cars within 0.24 m. Per-node consensus logs byte-identical (`verify_consensus_logs.py`). |
| **N3** | **Fault injection**: SIGKILL an assigned explorer (engine + coordinator + body) mid-probe, with its route the ONLY open one | **implemented** (`fault_injection.launch_test.py`) | **Lease liveness:** survivors propose `timeout`, the dead bot's route is released and re-claimed, all three survivors arrive on it. **f=1 at n=4:** the three surviving `/vertex/event` streams keep finalizing and stay byte-identical. Exclusivity holds throughout. |
| **N4** | **Lifecycle churn**: `deactivate → activate` the first-arrived robot's `vertex_node` via `/vertex/transition` while the fleet keeps moving | **implemented** (`lifecycle_churn.launch_test.py`) | **No `Inactive` leakage:** zero messages on the churned node's `/vertex/event` while inactive, while injected no-op traffic demonstrably finalizes on the live nodes in the same window. Both transitions succeed, the mission completes, live streams stay byte-identical. (Re-delivery of events missed while inactive is upstream scope; the churned robot needs none because it already arrived.) |
| **N5** | **Randomized soak**: back-to-back missions, each with a random blocked set (always ≥ 1 route open), restarted via consensus `reset` epochs | **implemented** (`soak_missions.launch_test.py`, `SOAK_SECONDS`) | **Consistent termination:** every mission ends `done` on all four robots with one agreed winner that is never a blocked route. **No leak:** each `vertex_node` RSS grows < 50 MB after the first-mission warm-up. Streams end byte-identical. |

### 4.1 Pass/fail table (implemented checks)

| Property | Measured from | Pass condition |
|---|---|---|
| **Route exclusivity** | every `mission_state` snapshot | no instant shows two bots assigned the same route |
| **Shared-state agreement** | `/vertex/event` per robot (headless) or `logs/robot_*.log` (live) | byte-identical ordered event prefixes on all live peers |
| **Convergence** | `winner_route` per robot | identical, non-None, and not a blocked route |
| **Liveness** | `phase` per robot | all robots reach `done` within the test deadline |
| **Physical completion** | Supervisor ground truth (live) | every car past the goal area; `SUCCESS` printed |
| **No collisions** | Supervisor pairwise distance monitor (live) | no pair of centres ever closer than 0.24 m |
| **Lease liveness** | N3: survivors' `mission_state` | the dead explorer's route is released and the mission completes with 3 of 4 nodes |
| **No `Inactive` leakage** | N4: churned node's `/vertex/event` | 0 messages while inactive, with live traffic finalizing in the window |
| **Endurance** | N5: `vertex_node` RSS + per-mission checks | every mission terminates consistently; RSS growth < 50 MB after warm-up |
| **Clean shutdown** | `launch_testing` exit codes | 0 / `-SIGINT` / `-SIGTERM` (plus the victim's intentional `-SIGKILL` in N3) |

---

## 5. Assumptions, limitations & caveats

- **Claim arbitration is Vertex's total order.** A route's owner is the first
  claim for it in consensus order. This is correct precisely because all
  robots share one byte-identical ordered log; it is the property under test,
  not an assumption layered on top.
- **The lease bounds liveness, not safety.** Route exclusivity never depends
  on the lease: while a dead explorer holds a route, nobody else is on it.
  The `timeout` only restores progress. `lease_sec` must exceed the
  worst-case probe time or a slow-but-alive explorer gets its route revoked;
  that is recoverable (the bot re-enters the claim loop) but wasteful. The
  live launch file uses 45 s, the fault-injection test 8 s.
- **No retroactive catch-up.** Vertex orders events a node observes *while
  participating*. A node that is `Inactive` or crashed during decisions does
  not automatically relearn them on rejoin without a running-session
  state-transfer path (`vertex.joining_running_session`).
- **Determinism.** The state machine and waypoint motion are deterministic,
  but the real engines' network timing is not, so which bot wins the race to
  claim a given route varies run to run. Assertions therefore check invariants
  (exclusivity, agreement, convergence, termination), never a fixed
  assignment or exact timings. The launch_test additionally sets
  `random_routes: False` for deterministic tie-breaking on the robot side.
- **Layer boundary.** L3 is not a Vertex correctness test; L0/L1 are the
  authoritative ordering tests.
- **Out of scope.** `whitened_signature` and `SyncPoint` payloads (L1 /
  upstream).

---

## 6. Harness layout (as built)

```
vertex_ros2/test/simulation/
├── README.md                        # this document
├── worlds/
│   ├── routes_4bot.wbt              # standalone demo world (explorer_demo)
│   ├── routes_4bot_ros2.wbt         # ROS 2 world (waypoint_follower per bot)
│   └── README.md                    # quickstart, split architecture, controls
├── protos/
│   └── ExplorerBot.proto            # 4-wheel skid-steer car: GPS, IMU, front_ds,
│                                    # peer_tx/peer_rx pose radio (collision avoidance)
├── config/
│   └── routes.yaml                  # lane/waypoint/barrier geometry (reference; the
│                                    # nodes mirror these values as constants)
├── controllers/
│   ├── waypoint_follower/           # per-bot Webots controller (native, rosbridge client)
│   ├── route_supervisor/            # keyboard barrier control, reset, autotest fixture
│   └── explorer_demo/               # iteration-1 standalone driver (no ROS)
├── nodes/
│   ├── mission_fsm.py               # pure replicated state machine (§3.1)
│   ├── test_mission_fsm.py          # 13 unit tests, plain python3
│   ├── mission_coordinator.py       # ROS 2 wrapper: claims, outcomes, lease, drive
│   ├── mock_robot.py                # headless stand-in for Webots (freeze/blocked hooks)
│   └── verify_consensus_logs.py     # proves live logs share one ordered stream
├── fixtures/
│   ├── gen_peers4.sh                # 4 keypairs + addrs (127.0.0.1:47611-47614)
│   └── peers4.json
├── logs/                            # per-node consensus logs from live runs
├── route_exploration.launch.py      # container side: rosbridge + 4x (vertex_node
│                                    #   + mission_coordinator)
├── route_exploration.launch_test.py # N1 headless integration test
├── fault_injection.launch_test.py   # N3: crash the explorer, lease recovers
├── lifecycle_churn.launch_test.py   # N4: deactivate/activate mid-mission
└── soak_missions.launch_test.py     # N5: randomized mission soak (SOAK_SECONDS)
```

### 6.1 Launch composition (per robot, ×4)

```python
# per robot i (route_exploration.launch.py / route_exploration.launch_test.py):
launch_ros.actions.Node(
    package="vertex_ros2", executable="vertex_node", name=f"vertex{i}",
    remappings=[(f"/vertex/{t}", f"/robot_{i}/vertex/{t}")
                for t in ("tx","event","sync_point","status","transition")]
              + [("/vertex/lifecycle/state", f"/robot_{i}/vertex/lifecycle/state")],
    parameters=[{
        "vertex.bind_address": me["addr"],
        "vertex.secret_key_base58": me["secret"],
        "vertex.peers": peer_specs,          # the other 3
        "options.heartbeat_us": 50000,
    }],
),
# + mission_coordinator (and mock_robot in the headless test) for robot_i
```

The 4-peer mesh, remapping, lifecycle-via-`/vertex/transition`, and
byte-identical event check are lifted directly from
`system_three_peers.launch_test.py` (scaled 3 → 4).

### 6.2 Barrier detection — by stall, not by range sensor

A robot reports "blocked" when it **stops making forward progress** inside its
lane, on its target row, while pushing east (it is pressing against a raised
barrier). This is deliberately *not* a forward distance-sensor reading: on a
fast, light chassis the sensor ray dips toward the floor under acceleration
pitch and fires phantom "barrier ahead" hits. Progress-stall is immune to that
and is more realistic. The stall clock is frozen while the bot is maneuvering,
off its row, or reacting to another bot, so traffic is never misreported as a
barrier. `arrived` requires passing the goal line **on the claimed route's
row** (both x and y), so a bot shoved down a different open lane cannot credit
its assigned route with a phantom arrival. The front distance sensor still
exists as a last-resort proximity stop; bot-to-bot avoidance itself runs on
the peer pose radio (see `worlds/README.md`).

---

## 7. Assertion sketches (as implemented)

Route exclusivity and convergence, from each bot's published `mission_state`
(`route_exploration.launch_test.py`):

```python
assigned = state["assigned"]                      # bot -> route, every snapshot
assert len(set(assigned.values())) == len(assigned)   # no route held twice

# on completion, on every bot:
assert sorted(state["arrived"]) == [0, 1, 2, 3] and state["phase"] == "done"
assert state["winner_route"] is not None          # same single value on all bots
assert "R1" in state["blocked"]                   # the physical block was learned
```

Shared-state agreement (the L1 byte-identical check, reused):

```python
for idx in range(min_len):                        # per-event, across all 4 bots
    assert events[0][idx] == events[i][idx]       # (hash, all payload bytes)
```

Live-run equivalents: `verify_consensus_logs.py` diffs the four per-node event
hash sequences, and the Supervisor's autotest monitor asserts physical
completion and pairwise separation ≥ 0.24 m. The post-shutdown clean-exit
check is identical to L1 (`assertExitCodes` allowing `0 / -SIGINT /
-SIGTERM`).

---

## 8. Running

Headless test suite (no Webots; this is what CI runs): fsm unit tests, then
the N1 exploration, N3 fault-injection, and N4 lifecycle-churn launch_tests:

```bash
docker compose run --rm sim simtest
```

Mission soak (N5, long-running):

```bash
SOAK_SECONDS=300 docker compose run --rm -e SOAK_SECONDS sim simsoak
```

Live simulation (native Webots on the Mac + the containerised ROS/Vertex
graph; details and controls in `worlds/README.md`):

```bash
# terminal 1: rosbridge + 4x vertex_node + 4x mission_coordinator
docker compose run --rm --service-ports sim

# terminal 2: the world, in native Webots
/Applications/Webots.app/Contents/MacOS/webots \
    vertex_ros2/test/simulation/worlds/routes_4bot_ros2.wbt
```

Scripted live scenarios (headless Webots, used to validate this harness):

```bash
WEBOTS_AUTOTEST=turn3 webots --no-rendering --batch --mode=fast \
    vertex_ros2/test/simulation/worlds/routes_4bot_ros2.wbt
# open2 / open3 ... : exclusive-open one route once all cars are committed
# turn3            : open R2, then flip to R1-only mid-mission (stale-block stress)
python3 vertex_ros2/test/simulation/nodes/verify_consensus_logs.py
```

Unit tests (any host, no ROS):

```bash
python3 vertex_ros2/test/simulation/nodes/test_mission_fsm.py
```

CI: the `sim-test` job in `.github/workflows/ci.yml` runs `sim simtest` on
every push and pull request, and the nightly `soak` job also runs
`sim simsoak` (5 min). The live Webots scenarios remain host-run (Webots has
no arm64 Linux build for the container).

---

## 9. Traceability — mapping to acceptance criteria & tickets

| This harness | Existing criterion (design §7) | Relationship |
|---|---|---|
| N1 agreement + byte-identical events | "events published in Vertex order; byte-identical across peers (G4)" | re-confirms at 4 peers, and shows the ordered log driving a concurrent multi-bot decision |
| N1 route exclusivity, convergence, liveness | *(none: application layer)* | new; the coordinated-exploration demonstration |
| N2 stale-block recovery (`unblock_all`, winner re-open) | *(none: application layer)* | new; world mutation mid-mission |
| N3 lease liveness + survivor agreement | "n=4 tolerates f=1" + finality survives crash | only possible at ≥ 4 peers; the survivors' byte-identical streams show finality holds with a peer dead |
| N4 no-`Inactive` leakage | "no ROS publishes occur in `Inactive`" | re-confirms the L1 property under a moving fleet with live consensus traffic |
| N5 mission soak | "no unbounded memory growth under 10-min load" | application-scenario analogue of `soak.launch_test.py`: churn from resets, claims, and outcomes instead of a raw tx firehose |

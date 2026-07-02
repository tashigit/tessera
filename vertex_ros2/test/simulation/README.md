# Simulation Test Plan — `vertex_ros2` consensus-coordinated route exploration

Status: **test plan** (v0.1 — not yet wired into CI).
Scope: application-level acceptance of the `vertex_ros2` ↔ Vertex consensus
stack, driven by a fleet of ≥ 4 simulated robots in a physics world.

Scenario under test: **Consensus-coordinated route exploration** — four robots
must get from A to B across a map with four alternative routes, not knowing which
are open. Each round they reach consensus on *which robot explores next and which
route it takes*; that robot probes the route and reports **blocked** or
**arrived**; the fleet re-decides from the finalized outcome and repeats until a
route is proven open — after which the remaining robots all follow that route.
Correct behaviour is a leaderless, totally-ordered distributed decision loop of
the kind Vertex exists to provide.

---

## 1. Testing strategy across layers

Vertex's contract is *totally-ordered, cryptographically-final delivery of
opaque bytes*. Ordering correctness is independent of robot bodies, sensors, or
physics, so it is tested headless and must not be verified through a simulator
where flakiness and cost would swamp the signal. A robot simulator is required
only at the application layer, where the acceptance criteria are physical.

| Layer | What it proves | Where it's tested | Simulator |
|---|---|---|---|
| **L0 — engine** | 3 real engines reach identical total order over UDP | `vertex_core/tests/multi_node.rs` | none |
| **L1 — ROS bridge** | `vertex_node` publishes events in Vertex order; byte-identical `/vertex/event` across peers; no publishes while `Inactive`; lifecycle cycles | `test/system_three_peers.launch_test.py` | headless `launch_test` |
| **L2 — endurance** | no RSS growth under 10-min load | `test/soak.launch_test.py` | headless |
| **L3 — application** | agreement on the ordered log elects the same explorer, propagates the same route outcomes, and converges every robot onto the same proven route — all driving **physical** motion | **this plan** | **required** |

L3 is the only layer that needs a simulator. Its acceptance criteria are
physical — "exactly one robot explores at a time", "the physically-moving robot
is the consensus-elected one", "everyone converges on the proven-open route and
reaches B" — asserted against ground-truth poses and traversed routes, not
against a byte stream. The consensus stream is the *input* to each robot's
decision; the physics is where the correctness and consistency of the resulting
motion is observed.

L3 does not re-test consensus. Ordering, byte-identical streams, and
no-publish-in-`Inactive` are proven at L0/L1 far more cheaply. L3 *reuses* the
byte-identical assertion as a nearly-free cross-check but exists to validate the
physical, multi-round coordination. A green L1 with a red L3 points at the
application/protocol or the simulation wiring — not the engine.

### 1.1 What the contract actually surfaces

Vertex is a hashgraph-family aBFT engine (gossip-about-gossip + virtual voting).
The application does **not** see the internal median-timestamp voting; it
receives the **finalized total order** of events plus per-event `consensus_at` /
`created_at` timestamps (fields on `VertexEvent`). Every protocol decision below
— who is elected, which routes are blocked/open, which route wins — is a pure
function of that one ordered log, applied identically on all four robots. The
"line up the logs from all 4 machines and they match" verification is exactly
the byte-identical `/vertex/event` assertion already in
`system_three_peers.launch_test.py`.

### 1.2 Simulator choice

`webots_ros2` (first-party, Cyberbotics) is the baseline: `WebotsLauncher` +
`Ros2Supervisor` integrate with `launch`/`launch_testing`, it runs headless
(`webots --no-rendering --batch --mode=fast`) for CI, and the **Supervisor** API
gives deterministic ground-truth poses and lets the harness drop a barrier on a
chosen route at a chosen time — exactly what the assertions need. Gazebo
(Harmonic) + `ros_gz` is an equally valid alternative; the scenario is
simulator-agnostic and the choice affects only the world file and the
ground-truth-readout node (see §8 for the Gazebo swap).

---

## 2. Fleet size rationale — why ≥ 4 robots

Vertex is BFT with the standard `n ≥ 3f + 1` quorum:

| Peers `n` | Faults tolerated `f = ⌊(n−1)/3⌋` |
|---|---|
| 3 | **0** |
| **4** | **1** |
| 7 | 2 |

Four robots is the smallest fleet that tolerates one faulty/crashed robot — the
property `N3` (fault injection: crash the elected explorer mid-probe) exercises.
It also provides three waiting observers per round to assert election and state
agreement. Four routes pair naturally with four robots. The scenario scales to
`N ≥ 4`; 4 is the documented minimum.

---

## 3. Scenario — Consensus-coordinated route exploration

Four routes `R0..R3` fan out from a junction near start **A** and rejoin near
goal **B**; a closed route's barrier sits **deep** in its corridor. All robots
start staged at A. The fleet explores routes **one at a time**, coordinated
entirely through the Vertex total order, until a route is proven open; the rest
then follow it.

```
                        R0 ───────────────█  (barrier: blocked)
                       ╱                     ╲
   A ● ─ staging ─ junction ─ R1 ─────────────┤─ ● B
   (all start)          │     R2 ──────────────│  (goal)
                         ╲    R3 ───────────────╱
     Round 1: consensus elects (bot, route). Only that bot leaves staging.
     It probes; reports blocked/arrived. Fleet re-elects from the outcome.
     On first 'arrived', the remaining bots follow the proven route.
```

### 3.1 Protocol — a replicated state machine over the ordered log

Every robot maintains identical state, computed purely from `/vertex/event` in
consensus order:

```
round        := current exploration round (starts at 0)
tried[route] := unknown | blocked | open
explorer(r)  := bot of the FIRST Claim with round == r, in consensus order
winner_route := route of the FIRST Arrived event  (None until one arrives)
phase        := EXPLORING | CONVERGING (winner_route set) | FAILED (all blocked)
```

Round `r`, per robot:

1. **Elect.** Each idle robot with an untried route may publish
   `Claim{bot, route, round=r}` to `/vertex/tx`. The winner is the first such
   claim in consensus order — identical on every robot. → this is "consensus on
   who goes first and which route".
2. **Probe.** Only `explorer(r)` leaves staging and drives its claimed route; all
   others wait. (One explorer at a time — see §4 safety.)
3. **Report.**
   - Barrier ahead → `Blocked{route, round=r}`. `tried[route]=blocked`; round++;
     back to step 1.
   - Reached B → `Arrived{route, round=r}`. `tried[route]=open`;
     `winner_route=route`; phase → CONVERGING.
   - Explorer stalls (crash/no progress) → any robot may publish
     `Timeout{round=r}` after `LEASE`; whichever of Blocked/Arrived/Timeout is
     first in consensus order ends the round → re-elect. (Liveness under fault.)
4. **Converge.** In CONVERGING, every remaining robot drives `winner_route` to B.
5. **Terminate.** EXPLORING ends when a route is proven open; if all four routes
   become `blocked` with no arrival, phase → FAILED and all robots abort
   consistently (a valid, agreed negative outcome — see `N5`).

Because steps 1–5 are a deterministic function of the same byte-identical
ordered log, all four robots elect the same explorer, record the same outcomes,
and converge on the same route — with no central coordinator. Each round makes
progress (an untried route becomes blocked, or a route is proven open), so the
mission terminates in ≤ 4 rounds.

### 3.2 Per-robot node graph

Each robot `i` runs, in namespace `/robot_i`:

```
 ┌───────────────────────────────────────────────────────────────────┐
 │ robot_i                                                             │
 │                                                                     │
 │  waypoint_follower ──cmd_vel──▶ (Webots diff-drive)                 │
 │        ▲ drive/wait + current_route                                 │
 │        │                                                            │
 │  mission_coordinator   (the replicated state machine, §3.1)         │
 │     │  Claim / Blocked / Arrived / Timeout ──▶ /robot_i/vertex/tx   │
 │     ◀── ordered log ─────────────────────────  /robot_i/vertex/event│
 │     ▲  progress stall on current route => barrier ahead             │
 │                                                                     │
 │  vertex_node  (the crate under test; peers with the other 3)        │
 └───────────────────────────────────────────────────────────────────┘
```

- **`vertex_node`** — the binary under test. One per robot, peered with the
  other three (the L1 mesh, scaled 3 → 4).
- **`mission_coordinator`** — test-fixture node implementing §3.1. Publishes
  Claim/Blocked/Arrived/Timeout; drives the follower according to whether it is
  the current explorer, a waiter, or converging.
- **`waypoint_follower`** — drives the waypoint list of the assigned route (or
  holds at staging); no Nav2 planner, no costmap plugin — motion stays
  deterministic and assertions crisp.

### 3.3 Transaction payloads

Vertex payloads are opaque (`VertexTransaction.payload`, `uint8[]`); the
coordinator encodes small CBOR/JSON records, all carrying `round` so stale
messages from a prior round are ignored:

```jsonc
{ "op": "claim",   "bot": 2, "route": "R0", "round": 0 }
{ "op": "blocked", "bot": 2, "route": "R0", "round": 0 }
{ "op": "arrived", "bot": 1, "route": "R2", "round": 1 }
{ "op": "timeout", "bot": 3, "round": 0 }              // any bot flags a stalled explorer
```

Route ids are fixed labels from `config/routes.yaml`, so all peers key state
identically.

---

## 4. Sub-scenarios & assertions

All sub-scenarios share the 4-robot / 4-route world; they differ in which routes
are blocked, timing, and fault injection. Ground truth (poses, barrier poses,
route traversed) comes from the Webots Supervisor, sampled at ≥ 10 Hz.

The signature **safety property** unique to this protocol: **exploration mutual
exclusion** — at most one robot is ever past the staging line into an unproven
route at a time.

| ID | Scenario | Key assertions |
|---|---|---|
| **N1** | **Single blockage** — `R0` blocked; a later route open | **Election agreement:** the physically-moving explorer each round == `explorer(round)` computed from every robot's log (identical across robots). **Mutual exclusion:** ≤ 1 robot in an unproven route at all samples. **Agreement:** all 4 `/vertex/event` byte-identical (reused L1); `tried[]` identical across robots at each round boundary. **Convergence:** all robots end on the same `winner_route` and reach B. **Liveness:** mission completes in ≤ 4 rounds, ≤ `T_max`. |
| **N2** | **Multiple blockages** — 2–3 routes blocked before an open one | **Multi-round liveness:** each blocked round increments `round` and re-elects; mission still converges. **Order determinism:** the applied event sequence (claims, outcomes) is identical across all robots. |
| **N3** | **Fault injection** — crash `explorer(r)`'s `vertex_node` (and/or process) *mid-probe*, before it reports | **Lease liveness:** a `Timeout{r}` ends the stalled round; the fleet re-elects and completes (`n=4, f=1`). **Finality:** any outcomes finalized before the crash remain in the survivors' state. **Agreement:** survivors' state stays converged. |
| **N4** | **Lifecycle churn** — `deactivate → activate` a *waiting* robot via `/vertex/transition`, timed `active` across the rounds it participates in | **No `Inactive` leakage:** 0 messages on that robot's `/vertex/event` while `inactive`. **Rejoin:** after reactivation it re-converges and follows `winner_route`. (A robot `Inactive` *during* rounds may miss those events — see §5; N4 keeps it `active` across the decisions it must act on.) |
| **N5** | **Randomized missions (soak)** — 10 min of back-to-back missions, each with a random blocked-route set (including occasional **all-blocked**) | **Consistent termination:** every mission ends with all robots agreeing on the same `winner_route`, or all agreeing FAILED (all-blocked). **No deadlock/starvation.** **No RSS growth** > 50 MB after minute 1 (Supervisor + `ps`). |

### 4.1 Pass/fail table

| Property | Measured from | Pass condition |
|---|---|---|
| **Exploration mutual exclusion** | Supervisor poses vs. staging line | ≤ 1 robot past staging into an unproven route at every sample |
| **Election agreement** | `explorer(round)` per robot's log vs. the moving robot | identical across all live robots and == the physically-moving robot, every round |
| **Shared-state agreement** | `/vertex/event` per robot + published `tried[]` | byte-identical ordered event prefixes and identical `tried[]` across all live peers |
| **Order determinism** | applied-event sequence per robot (N2) | identical across all robots |
| **Convergence** | `winner_route` per robot + routes physically traversed | identical `winner_route`; all live robots reach B via it |
| **Liveness** | round count + goal-reached times | ≤ 4 rounds; every live robot reaches B ≤ `T_max` (except agreed all-blocked) |
| **Fault tolerance / lease** | N3 | stalled round ends via `Timeout`; mission completes with explorer down |
| **No `Inactive` leakage** | N4 | 0 messages on a node's `/vertex/event` while `inactive` |
| **Endurance** | N5 | RSS growth ≤ 50 MB after 60 s; every mission terminates consistently |

---

## 5. Assumptions, limitations & caveats

- **Election tie-break is Vertex's total order.** `explorer(r)` = first
  `Claim{round=r}` in consensus order. This is correct precisely because all
  robots share one byte-identical ordered log; it is the property under test, not
  an assumption layered on top.
- **Lease bounds liveness, not safety.** If the elected explorer crashes before
  reporting, mutual exclusion still holds (nobody else was moving); the `LEASE` +
  `Timeout` only restores *progress*. `LEASE` must exceed a route's worst-case
  probe time to avoid spurious timeouts; assertions use invariants, not `LEASE`
  timing.
- **No retroactive catch-up.** Vertex orders events a node observes *while
  participating*. A node `Inactive` (N4) or crashed during rounds does not
  automatically relearn missed decisions on rejoin without a running-session
  state-transfer path (`vertex.joining_running_session`, out of scope / upstream
  TAS-96). N4 keeps the churned robot `active` across the decisions it acts on;
  N3 crashes the explorer such that pre-crash outcomes are already finalized.
- **Determinism.** The state machine and waypoint motion are deterministic, but
  the real engines' network timing is not — so which robot *wins the race to
  claim* a round can vary run to run. Assertions therefore check invariants
  (exactly one explorer; the mover == the elected one; convergence; termination),
  never a fixed explorer sequence or exact timings.
- **Layer boundary.** L3 is not a Vertex correctness test; L0/L1 are the
  authoritative ordering tests.
- **Out of scope.** `whitened_signature` and `SyncPoint` payloads (L1 / upstream
  TAS-92, TAS-95).

---

## 6. Harness layout

```
vertex_ros2/test/simulation/
├── README.md                        # this plan
├── worlds/
│   └── routes_4bot.wbt              # A/B, junction, 4 corridors, 4 TurtleBot3, Supervisor
├── config/
│   └── routes.yaml                  # waypoint list per route + junction + staging + barriers
├── nodes/
│   ├── mission_coordinator.py       # replicated state machine (§3.1) + claim/report
│   ├── waypoint_follower.py         # drive assigned route / hold at staging (cmd_vel)
│   └── state_probe.py               # (optional) republish {round, explorer, tried, winner} for asserts
├── fixtures/
│   └── peers4.json                  # 4 keypairs+addrs (gen_test_keys.sh -n 4)
├── route_exploration.launch_test.py # N1/N2 (a param selects the blocked-route set)
├── fault_injection.launch_test.py   # N3
├── lifecycle_churn.launch_test.py   # N4
└── soak_missions.launch_test.py     # N5 (nightly)
```

Reuses the existing `gen_test_keys.sh` (extended to `-n 4`) and the `peers.json`
convention from `test/system_three_peers.launch_test.py`.

### 6.1 Launch composition (per robot, ×4)

```python
# webots + Ros2Supervisor started once. Then, per robot i:
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
# + mission_coordinator + waypoint_follower for robot_i
```

The 4-peer mesh, remapping, lifecycle-via-`/vertex/transition`, and
byte-identical event check are lifted directly from
`system_three_peers.launch_test.py` (scaled 3 → 4); this harness inherits a
proven pattern and adds only the Webots world and the two small fixture nodes.

### 6.2 Barrier detection — by stall, not by range sensor

A robot reports "blocked" when it **stops making forward progress** toward its
current waypoint (it is pushing against a raised barrier), i.e. no progress for
`STALL_WINDOW` seconds. This is deliberately *not* a forward distance-sensor
reading: on a fast, light chassis the sensor ray dips toward the floor under
acceleration pitch and fires phantom "barrier ahead" hits. Progress-stall is
immune to that and is more realistic. Only the explorer drives, so only it can
stall; consensus distributes the outcome. (`arrived` is still by GPS x > goal.)

---

## 7. Assertion sketches

Exploration mutual exclusion + election agreement, from the Supervisor:

```python
# at every sample: at most one robot is past the staging line into an unproven route
movers = [r for r in robots
          if past_staging(pose(r)) and route_of(r) not in proven_open]
assert len(movers) <= 1, f"MUTEX VIOLATION: {movers} exploring at once"

# and the mover is the consensus-elected explorer for the current round
if movers:
    assert movers[0] == explorer_from_log(round)   # identical on every robot's log
```

Shared-state agreement (reused shape from L1, plus derived state):

```python
for idx in range(min_len):                            # byte-identical event log
    e0 = received[0][idx]
    for i in live_peers[1:]:
        assert bytes(e0.hash) == bytes(received[i][idx].hash)
        assert ([bytes(t.payload) for t in e0.transactions]
             == [bytes(t.payload) for t in received[i][idx].transactions])

# identical derived state across robots (via state_probe)
assert same_across(live_peers, lambda i: (tried[i], winner_route[i]))
```

Convergence + liveness:

```python
assert same_across(live_peers, lambda i: winner_route[i]) and winner_route[0] is not None
for X in live_robots:
    assert reached_goal(X, via=winner_route[0]) and goal_time(X) <= T_MAX
assert rounds_used <= len(ROUTES)
```

The post-shutdown clean-exit check is identical to L1 (`assertExitCodes`
allowing `0 / -SIGINT / -SIGTERM`).

---

## 8. Running

Local (headless, Docker — extends the existing Jazzy harness with `webots_ros2`):

```bash
# docker/Dockerfile gains: ros-jazzy-webots-ros2, webots
docker compose run --rm sim-routes                  # N1/N2
docker compose run --rm sim-fault                   # N3
SOAK_SECONDS=600 docker compose run --rm sim-soak   # N5, nightly
```

CI: add a `sim` job to `.github/workflows/ci.yml` mirroring the `test` job but
invoking the launch_tests with `webots --no-rendering --batch --mode=fast`. Keep
`N5` on the nightly schedule next to the existing soak.

**Gazebo swap:** replace `worlds/routes_4bot.wbt` with an SDF world +
`ros_gz_sim`, read ground truth from `/model/robot_i/pose`, and drop barriers via
`ros_gz` factory spawn instead of the Webots Supervisor. The route config,
`mission_coordinator`, `waypoint_follower`, protocol, and every assertion in
§4/§7 are unchanged.

---

## 9. Traceability — mapping to acceptance criteria & tickets

| This plan | Existing criterion (design §7 / TAS-69) | Relationship |
|---|---|---|
| N1/N2 election + state agreement | "events published in Vertex order; byte-identical across peers (G4)" | reuses / re-confirms at 4 peers, and shows the ordered log driving a multi-round decision |
| N4 no-`Inactive` leakage | "no ROS publishes occur in `Inactive`" | re-confirms under a moving fleet |
| N3 | "n=4 tolerates f=1" + finality-survives-crash + lease liveness (extends the L1 "n=3 tolerates 0" note) | new — only possible at ≥ 4 peers |
| N1 mutual exclusion, convergence, liveness | *(none — application layer)* | new — the coordinated-exploration demonstration |
| N5 | "no unbounded memory growth under 10-min load" | physical-scenario analogue of `soak.launch_test.py` |

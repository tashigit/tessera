# Simulation 2 — consensus-coordinated arena exploration (`pioneer_arena`)

Status: **v0.1 (as built)**. Second application-level simulation on the
`vertex_ros2` ↔ Vertex stack, alongside the route exploration harness
(`../simulation/README.md`). Five Pioneer 3-AT robots cooperatively sweep an outdoor arena;
every inter-robot decision is a deterministic fold of the Vertex ordered log,
through the same `vertex_fleet` consumer API the route scenario exercises.

The scenario is a port of an earlier MQTT-based multi-robot exploration stack.
That stack pushed frontier coordination, mission status, and a telemetry-health
*voting protocol* through a 4-broker MQTT cluster, and its own docs noted the
voting depended on the broker's total ordering for state machine replication.
Here the ordered log is the primitive, so the vote/tally machinery disappears:
what needed a broker cluster plus a hand-rolled consensus protocol is one
`ReplicatedState` fold. Only the Webots assets came across: the arena world
(trees, rocks, pits, deer) and the five Pioneer 3-AT robots with their Sick
LMS 291 lidars and webcams.

---

## 1. Scenario

The arena's interesting region (x, y in [-20, 20] x [-15, 15]; walls at ±25)
is divided into a fixed grid of sectors (5x4 of 8 x 7.5 m live; 4x2 headless).
A sector is *explored* when a robot physically reaches its centre. The fleet
is done when every sector is explored or condemned unreachable.

What consensus owns:

1. **Claim.** An idle, healthy bot claims the nearest free sector every
   `claim_interval`. Claims are arbitrated by consensus order: the first claim
   for a sector wins it, losers just claim again. A bot holds at most one
   sector; no sector is ever held twice.
2. **Report.** The claim holder reports `explored` on covering the sector
   centre. A holder that stops making progress (obstacle cluster, pit)
   releases the sector with `abandon`; after `max_attempts` of its own
   failures it condemns the sector with `unreachable`. Only the holder can
   report — a bot whose claim was released must re-claim first, so coverage
   is never credited off a stale assignment.
3. **Health.** Every bot folds a periodic `health` beacon into the log:
   monotonic `seq`, self-assessed `ok` from its local sensor-stream freshness
   (lidar/camera ages, the live equivalent of the MQTT stack's per-stream
   telemetry timeouts). A not-ok beacon makes the bot fleet-wide unhealthy at
   that consensus point: its claim is released, new claims are refused, and
   its detections are rejected. A fresh ok beacon readmits it. No votes, no
   tallies — every bot folds the same beacons in the same order, so the
   verdicts agree everywhere by construction.
4. **Silence lease.** A crashed bot never sends a not-ok beacon. Any live bot
   may propose `suspect` carrying the victim's last folded beacon `seq`; the
   fold acts only if no newer beacon landed, so a late beacon voids every
   in-flight suspicion, duplicates are no-ops, and the first suspicion in
   consensus order wins (the same shape as the route scenario's lease
   timeout).
5. **Detections.** A robot that sights something (a deer, a flagged rock)
   relays it as a `detection` record. The fold accepts it only if the
   reporter is healthy at that point in the log — the predecessor's "reach
   consensus on robot health before accepting detections", now a pure rule
   applied identically on every bot.

### Transaction payloads

Opaque JSON records on `VertexTransaction.payload` (all carry `epoch`;
`reset` and epoch gating come from `vertex_fleet.ReplicatedState`):

```jsonc
{ "op": "claim",       "bot": 2, "sector": "S07", "epoch": 0 }
{ "op": "explored",    "bot": 2, "sector": "S07", "epoch": 0 }
{ "op": "abandon",     "bot": 2, "sector": "S07", "epoch": 0 }
{ "op": "unreachable", "bot": 2, "sector": "S07", "epoch": 0 }
{ "op": "health",      "bot": 2, "seq": 41, "ok": true, "epoch": 0 }
{ "op": "suspect",     "bot": 3, "victim": 2, "seen_seq": 41, "epoch": 0 }
{ "op": "detection",   "bot": 1, "seq": 0, "label": "deer",
                       "x": -14.6, "y": 9.6, "epoch": 0 }
{ "op": "reset",       "epoch": 1 }
```

## 2. Per-robot node graph

Each robot `i` runs, in namespace `/robot_i` (same launch composition as the
route scenario, scaled 4 → 5 peers, `fixtures/peers5.json`):

```
 ┌────────────────────────────────────────────────────────────────────┐
 │ robot_i                                                            │
 │                                                                    │
 │  pioneer_explorer  (Webots controller, native on the host;         │
 │        │            pose/telemetry/goto over rosbridge)            │
 │        ▲ goto: sector id | HOLD | STOP                             │
 │        │                                                           │
 │  arena_coordinator    (the replicated fold, nodes/arena_fsm.py)    │
 │     │  claim / explored / abandon / unreachable / health /         │
 │     │  suspect / detection      ──────────▶  /robot_i/vertex/tx    │
 │     ◀── ordered log ────────────────────    /robot_i/vertex/event  │
 │                                                                    │
 │  vertex_node  (the binary under test; peers with the other 4)      │
 └────────────────────────────────────────────────────────────────────┘
```

- **`arena_coordinator`** (`nodes/arena_coordinator.py`) extends
  `vertex_fleet.VertexAgent`; `ArenaState` extends
  `vertex_fleet.ReplicatedState`. It adds the scenario logic: the claim loop,
  physical outcome reports, the health beacon, the silence lease, the
  detection relay, and a per-node consensus log
  (`logs/robot_<i>_arena.log`, same EVENT/STATE/TX/DECIDE format the route
  scenario uses; verify a live run with
  `python3 ../simulation/nodes/verify_consensus_logs.py logs 'robot_*_arena.log'`
  from this directory; the script is shared with the route scenario).
- **`pioneer_explorer`** (`controllers/pioneer_explorer/`) drives toward the
  commanded sector centre with lidar-reactive avoidance. No map, no Nav2:
  motion stays deterministic and assertions crisp. There is no
  controller-to-controller channel; all cross-robot coordination is Vertex.
- **`mock_pioneer`** (`nodes/mock_pioneer.py`) replaces Webots headless:
  straight-line kinematics, scripted sensor-stream failure/recovery, planted
  detections.

## 3. Why 5 robots

The source world has five robots, and n = 5 still tolerates f = 1 under
Vertex's `n >= 3f + 1`. It also exercises the mesh at a size the route
scenario (n = 4) does not.

## 4. Assertions (headless launch_test)

`arena_exploration.launch_test.py`, 8 sectors, robot 4's sensor streams die
at t=6 s and recover at t=45 s, a detection planted on healthy robot 1 and
one on robot 4 mid-outage:

| Property | Pass condition |
|---|---|
| Full coverage | every sector `explored`, `phase == done`, on all 5 bots |
| Claim exclusivity | no `mission_state` snapshot shows a bot holding two sectors |
| Health agreement | robot 4's unhealthy episode observed on every bot, and the final state shows it readmitted |
| Detection gating | exactly the healthy detection accepted, identical list everywhere |
| Consensus | all 5 `/vertex/event` streams byte-identical |
| Clean shutdown | exit codes 0 / `-SIGINT` / `-SIGTERM` |

The fold itself (exclusivity, holder-only reports, stale-seq beacons, suspect
no-ops, detection gating, epochs) has 15 pure-python unit tests in
`nodes/test_arena_fsm.py`.

## 5. Running and viewing

### 5.1 Headless (CI; no Webots)

```bash
docker compose run --rm sim simtest        # both simulations' suites
# or just this scenario, inside the container:
#   python3 nodes/test_arena_fsm.py
#   launch_test arena_exploration.launch_test.py
```

### 5.2 Live, in Webots (the viewable run)

Same split as the route scenario: Webots runs natively on the Mac (no arm64
Linux build), the ROS 2 / Vertex graph runs in the Jazzy container, and the
two meet over rosbridge's WebSocket on port 9090.

One-time host setup:

```bash
# the follower needs websocket-client on the python3 Webots uses
# (launch Webots from a terminal so it is the same python3):
python3 -m pip install --break-system-packages websocket-client
# and the environment image, if never built:
docker compose build
```

**Terminal 1 — the consensus graph** (rosbridge + 5x `vertex_node` +
5x `arena_coordinator`; builds the workspace and generates
`fixtures/peers5.json` on first run):

```bash
docker compose run --rm --service-ports sim simarena
```

Wait for `Rosbridge WebSocket server started on port 9090` in the output.
The coordinators immediately start beaconing `health ... ok: False` — that is
correct: no robots are connected yet, so no telemetry streams exist.

**Terminal 2 — the world**, in native Webots (R2025a):

```bash
/Applications/Webots.app/Contents/MacOS/webots \
    vertex_ros2/test/simulation_arena/worlds/pioneer_arena.wbt
```

The first load takes a while: the world pulls its R2025a assets (Pioneer
robots, trees, rocks, deer textures) from the Webots CDN and generates the
16 procedural pit meshes. Later loads are much faster (assets cache under
`~/Library/Caches/Cyberbotics`).

### 5.3 What you should see

The 3D view shows only what a real deployment would show: robots driving.
The shared state is observed the way an operator would observe it — from
each node's stream and logs (deliberate: there are no in-world markers,
because the world would not carry any).

1. Five Pioneer 3-AT robots staged in a column on the west side (`robot_0`
   south to `robot_4` north).
2. Each controller prints `[pioneer_explorer] bridged to /robot_i/* via
   rosbridge` in the Webots console; within a few seconds terminal 1 flips to
   `health ... ok: True` beacons as telemetry arrives.
3. Robots leave staging as consensus assigns them sectors (nearest free
   sector first), steering around trees, rocks and each other on lidar.
4. Terminal 1 shows the shared state evolving as transactions: `claim`,
   `explored`, and the occasional `abandon`/`unreachable`, identical on
   every node.
5. The mission is done when every sector is explored or condemned — phase
   `done` in `mission_state` and in every per-node log.

Watch any robot's derived state live (third terminal):

```bash
docker exec -it $(docker ps -q -f name=tessera-sim-run | head -1) bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
   && ros2 topic echo /robot_0/mission_state'
```

Try a live experiment: pause Webots mid-run (`Ctrl+0` or the pause button).
Telemetry stops, every coordinator's next beacon reports `ok: False`, and the
fleet marks all robots unhealthy and releases their claims. Resume, and the
next beacons readmit everyone and the sweep continues — the MQTT
predecessor's whole voting protocol, visible as two lines of log.

### 5.4 Verifying a live run

Each coordinator writes `logs/robot_<i>_arena.log` (host-visible via the bind
mount): every consensus event delivered (with its hash), the folded state,
every transaction submitted, every decision. Prove all five robots shared one
ordered stream:

```bash
python3 ../simulation/nodes/verify_consensus_logs.py logs 'robot_*_arena.log'
# -> CONSENSUS VERIFIED: identical ordered event stream on every node
```

(Delete `logs/*.log` before a fresh run: the files append across runs.)

### 5.5 Stopping and troubleshooting

Stop Webots first, then Ctrl+C in terminal 1. If `docker compose run` is
interrupted abnormally, the container can survive its CLI: check
`docker ps` and `docker stop` any leftover `tessera-sim-run-*` (a leftover
holds port 9090 and its coordinators keep publishing into the next session).

| Symptom | Resolution |
|---|---|
| Robots never move, coordinators stay `ok: False` | The controllers are not connected: check `websocket-client` is importable by the python3 on the PATH Webots was launched from, and that port 9090 is forwarded (`--service-ports`). |
| A robot sits still for a long time | Usually recoverable: a wedged robot prints `wedged at (x, y) — backing off` and reverses free, and a stalled sector is `abandon`ed for someone else after `stall_sec`. A robot trapped in a pit (the horizontal lidar cannot see holes) logs `IMMOBILIZED` after `immobilized_sec` and stops claiming, so the rest of the fleet finishes without it. |
| `Port 1234 is already in use` warning in Webots | Another Webots instance is open (e.g. the route world). Harmless: Webots falls back to the next port. |
| Robots erratic, poses jumping | Two copies of this world are attached to the same rosbridge (e.g. a GUI run plus a forgotten headless run): both publish `/robot_i/pose`. Exactly one Webots world may be attached to the graph at a time. |
| Long white/frozen window on first open | Asset download + pit-mesh generation on first load. Wait it out once. |
| Headless smoke run | `webots --no-rendering --batch --stdout --stderr --mode=fast <world>` mirrors the GUI run and prints controller output to the terminal. |

## 6. Use-case parity with the pioneer (MQTT) stack

Verified against the pioneer source (its `telemetry_monitor_node`,
`consensus_handler_node`, `foxmq_bridge_node`, `goal_selector_node`, and
docs). Legend: **ported** = same observable behaviour on the new transport;
**subsumed** = the need is met by a different, simpler mechanism;
**dropped** = intentionally not carried over, with the reason.

| Pioneer use case | Status here | How |
|---|---|---|
| Cooperative exploration, no duplicated work | ported | exclusive sector `claim`s in consensus order (pioneer used frontier scoring + a hash mutex) |
| Full-area coverage with completion criterion | ported | `explored`/`unreachable` fold; done when the grid is exhausted (pioneer had no explicit termination) |
| Territory assignment (`TerritoryAssignment`) | subsumed | a claimed sector IS an exclusive territory; no separate message needed |
| Mission status sharing (`MissionStatus`) | subsumed | `phase`/`role` derived identically on every node from the fold; published as `mission_state` |
| Per-stream telemetry health (odom, cmd_vel, camera, scan; 2-3 s timeouts) | ported | pose (odom), lidar and camera freshness gate the self-assessed `health` beacon; cmd_vel has no robot-side equivalent (`goto` flows the other way) |
| Health consensus vote (4-of-5, partial timer, TTL, result broadcast) | subsumed | the beacon fold: every node derives identical verdicts from the ordered log, so the vote/tally/result machinery has nothing left to do |
| Excluding a silent (crashed) robot | ported | `suspect` silence lease (pioneer relied on missing voters) |
| Readmitting a recovered robot | ported | a fresh ok beacon; exercised in the launch_test |
| Detection gating on robot health (`DetectionReport`) | ported | `detection` records folded only from healthy reporters; unit + launch tested |
| Frontier sharing (`coordination/frontiers`) | subsumed | the fixed sector grid replaces the frontier pipeline; the shared thing (who explores where) is the claim |
| Path sharing (`coordination/paths`) | dropped | advisory data nobody agreed on; robots avoid each other physically (lidar), and sector exclusivity keeps them apart at the task level |
| Occupancy-grid mapping / SLAM products | dropped | out of scope for a coordination-fabric harness (§7); local autonomy can be re-added without touching the protocol |
| A* planning + Pure Pursuit following | subsumed | direct steering with lidar-reactive avoidance, wedge escape, and stall reporting — deterministic and assertable |
| Camera image pipeline (`image_raw` into ROS) | dropped | the camera is enabled and its freshness feeds health; frames themselves had no cross-robot consumer in pioneer either |
| Sensor/command bridging (`/tmp` file polling) | subsumed | rosbridge WebSocket + onboard GPS/IMU (the file bridge cannot cross the container boundary and was a latency tax) |
| Operator monitoring (consensus_monitor, system_health) | subsumed | `mission_state` topic, per-node consensus logs, `verify_consensus_logs.py` |
| Spatial analysis visualizer | dropped | nothing consumed it programmatically; logs are the observation model |
| Fleet restart | ported (stronger) | pioneer restarted via shell scripts; here `/reset` bumps the epoch through consensus |

## 7. Assumptions & limitations (v0.1)

- **Pose is GPS ground truth**, as in the route scenario: the property under
  test is the coordination fabric, not SLAM. The occupancy-grid / frontier
  pipeline of the MQTT predecessor is deliberately replaced by the fixed
  sector grid — it kept every robot's *local* autonomy but moved all *shared*
  state into the fold, which is the porting pattern the tutorial teaches.
- **Detections in the live world are not vision-based** (v0.1: the mock
  plants them; the camera is enabled and its freshness feeds the health
  beacon). Wiring a real detector onto the webcam changes nothing in the
  protocol: the fold's health gate is the point.
- **No supervisor robot.** The predecessor's supervisor wrote ground-truth
  poses to files; onboard GPS replaced it. A reset/world-change supervisor in
  the route scenario's style can be added later (`/reset` is already wired in
  the coordinator).
- **Crash-the-robot fault injection** (SIGKILL, exercising `suspect` on real
  silence) is unit-tested in the fold and implemented in the coordinator, but
  not yet a dedicated launch_test — the route scenario's N3 covers the
  engine-level property at n = 4.
- **Pits can permanently trap a robot.** The arena's craters are invisible to
  a horizontal lidar, and a Pioneer that drives into a deep one cannot climb
  out. The controller's reverse-and-turn escape handles wedges (trees, rocks,
  deer, other robots); for a genuine trap the coordinator declares itself
  immobilized after `immobilized_sec` and stops claiming, so a trapped robot
  degrades the fleet instead of deadlocking it. Mission completion then means
  every sector explored or condemned by the robots still mobile.

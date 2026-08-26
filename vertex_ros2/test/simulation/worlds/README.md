# Webots world: `routes_4bot.wbt` (quickstart)

Iteration-1 simulation for the route-exploration test plan (see `../README.md`).
Four bots, four routes (lanes), barriers you can raise/lower at runtime.

## Run it

Open the world in the installed Webots (R2025a):

```bash
# macOS
/Applications/Webots.app/Contents/MacOS/webots \
    vertex_ros2/test/simulation/worlds/routes_4bot.wbt
```

or just double-click `routes_4bot.wbt` in Finder / `File ▸ Open World` in Webots.

## What you'll see

- Four coloured robots (`robot_0..3`) staged on the west side, one per lane.
- With the standalone `explorer_demo` controller they drive east along their
  lane and stop at the green goal line (printing `ARRIVED`).

## Block a route

Click the 3D viewport (to give it keyboard focus), then:

| Key | Action |
|-----|--------|
| `1` `2` `3` `4` | **exclusive open**: open only that route, block the other three (the "find the one open path" test) |
| `T` | random test: open one random route, block the other three |
| `0` | open every route (clear all barriers) |
| `R` | **reset**: send all cars to staging, open all routes, restart the mission (fresh consensus epoch) |

**Parallel exploration:** every bot claims a route over Vertex and consensus
order assigns routes **exclusively**: no two bots ever share a route, and all
assigned bots drive at once (each prefers its own lane, so departures are
straight out). A bot that hits a barrier reports **blocked**, returns to the
start, and immediately claims another free route. Nobody waits on anybody.
The first bot to reach the end fixes the **winner route**: everyone else
abandons their exploration, traverses back, and takes the winner route to the
end. If every route is blocked, bots periodically retry one so the fleet
recovers as soon as you reopen a route.

**Collision safety:** consensus guarantees route exclusivity (never two bots in
one lane while exploring); physically, all lane changes run through transfer
columns (west `x=-3.5`, east `x=4.0`) kept clear of parked cars, and arrived
cars park on their own row at `x=4.5`. On top of that, every car broadcasts
its pose on a sim-local radio (the proto's `peer_tx`/`peer_rx`
Emitter/Receiver) and hears the other three, so each reacts to traffic the
single forward ray cannot see (`assess_peers` in `waypoint_follower.py`):

- **convoy spacing**: hold ~0.65 m behind a same-direction car ahead
- **crossing traffic**: the lower bot id has right of way, the higher id stops
- **head-on in a lane**: both cars shade right and pass inside the lane
- **head-on in the open**: the higher id stops, the lower id drives around it
- **stationary car in the open** (parked or yielding): steer around it, with
  wall and divider bands excluded from the dodge
- **stationary car in a lane**: queue behind it when exploring, squeeze past on
  the right when returning
- **emergency bubble**: never advance onto any car within 0.44 m ahead

All rules are deterministic (ids break every tie), any active peer reaction
freezes the stall clock so waiting for traffic is never misreported as
`blocked`, and the forward distance sensor plus the timed standoff breaker
remain as last-resort backstops.

## Layout (ENU metres)

```
                 R1  y=+2.25  ── barrier @ x=1.5 ──┐
   staging  ─────R2  y=+0.75  ──────────────────── │──  goal line
   x=-4     ─────R3  y=-0.75  ────────────────────  │   x=3.6
                 R4  y=-2.25  ─────────────────────┘
   dividers at y = 1.5 / 0 / -1.5, corridor x in [-3, 3]
```

## Files

| Path | Role |
|------|------|
| `worlds/routes_4bot.wbt` | the world |
| `protos/ExplorerBot.proto` | the robot: a 4-wheel skid-steer **car** (GPS + IMU). "Blocked" is detected by forward-progress **stall**, not a range sensor (avoids pitch-induced phantom hits) |
| `controllers/route_supervisor/` | keyboard route blocking + ground-truth access |
| `controllers/explorer_demo/` | iteration-1 standalone driver (drive + stop-at-barrier) |
| `config/routes.yaml` | lane/waypoint/barrier geometry, shared with the ROS 2 nodes |

## ROS 2 / Vertex integration (iteration 2): native Webots + Docker ROS

The consensus-coordinated exploration (`../README.md` §3) runs as a split:
Webots stays **native on the Mac** (it has no arm64 Linux build, so this keeps
the GPU and avoids x86 emulation), and the ROS 2 / Vertex graph runs in the
**arm64 Jazzy container**. They meet over rosbridge's WebSocket.

```
  Mac (native Webots)                      Docker (arm64, ros:jazzy)
  ┌───────────────────────────┐           ┌──────────────────────────────────┐
  │ routes_4bot_ros2.wbt       │  ws:9090  │ rosbridge_server                  │
  │  robot_i: waypoint_follower├──────────►│   ├─ /robot_i/pose, /barrier  ───►│  mission_coordinator_i
  │  (websocket-client bridge) │◄──────────┤   └─ /robot_i/drive          ◄────┤   ├─ /robot_i/vertex/tx ─► vertex_node_i
  │  route_supervisor (keys)   │           │                                   │   └─ /robot_i/vertex/event ◄─┘
  └───────────────────────────┘           └──────────────────────────────────┘
```

### Architecture: where tessera / Vertex lives

The simulation folder is only Python glue (Webots controllers, the
`mission_coordinator`, launch files). **Tessera enters as a compiled binary**,
so nothing here textually shows the engine:

1. `route_exploration.launch.py` starts `package="vertex_ros2",
   executable="vertex_node"`, **one per bot, four instances**, each with its
   own keypair. `vertex_ros2` *is* tessera's ROS package (this repo),
   colcon-built inside the container.
2. Crate chain inside each process:
   `vertex_node (vertex_ros2) → vertex_core → tashi-vertex →
   libtashi-vertex.so`, the last being the actual **Tashi Vertex consensus
   engine** (hashgraph gossip-about-gossip + virtual voting).
3. Each node binds a UDP socket (`vertex.bind_address`, 127.0.0.1:47611–47614
   from `fixtures/peers4.json`) and is peered with the other three
   (`vertex.peers`); `vertex_core/src/controller.rs` calls `Engine::start(...)`
   and the engine threads **gossip peer-to-peer over UDP** inside the container.
4. The bots communicate with each other **only** through that consensus:
   coordinator → `/robot_i/vertex/tx` → engine gossip + ordering →
   `/robot_i/vertex/event` on every bot, in the identical finalized order.
   There is no direct bot-to-bot ROS topic.

Two mechanisms prove it live: `nodes/verify_consensus_logs.py` (all four
per-node logs carry the identical ordered event-hash sequence) and the
launch_test's byte-identical `/vertex/event` assertion.

### Run it

```bash
# 0. one-time host setup: the follower controller needs websocket-client on the
#    python Webots uses (launch Webots from a terminal so it's the same python3):
python3 -m pip install --break-system-packages websocket-client

# 1. container side: rosbridge + 4x vertex_node + 4x mission_coordinator
#    (generates fixtures/peers4.json, colcon-builds vertex_ros2, then launches)
docker compose run --rm --service-ports sim

# 2. host side: open the ROS 2 world in native Webots and press Play
/Applications/Webots.app/Contents/MacOS/webots \
    vertex_ros2/test/simulation/worlds/routes_4bot_ros2.wbt
```

The followers connect to `ws://localhost:9090` (compose forwards it). Each
`mission_coordinator` brings its `vertex_node` to `Active` (via
`/vertex/transition`), then the fleet explores in parallel: consensus assigns
each bot an exclusive route, blocked bots re-claim without holding anyone up,
and the first arrival pulls everyone onto the winner route. Use keys `1`–`4`/`T`
to set which single route is open. (Override the bridge URL with
`WEBOTS_ROSBRIDGE_URL` if not on localhost.)

### Per-node consensus logs: proof the bots are Vertex nodes

Every `mission_coordinator` writes `simulation/logs/robot_<i>_consensus.log`
(host-visible via the bind mount) recording, per node: every **consensus event
delivered** (`EVENT #n hash=… records=[…]`), the resulting shared **STATE**,
every **TX** it submitted, and every **DECIDE** (role/target/drive) it took.
Verify all four nodes share one Vertex consensus:

```bash
python3 vertex_ros2/test/simulation/nodes/verify_consensus_logs.py
# -> CONSENSUS VERIFIED: identical ordered event stream on every node
```

When you change barriers (keys `1`–`4`/`T`/`0`), the Supervisor broadcasts a
**world-change**; coordinators relay it into Vertex as `unblock_all`, clearing
stale block knowledge so the fleet re-explores immediately. Converging bots
enter the winner lane **staggered by a consensus-derived rank** (collision
avoidance with zero extra messages).

### Node / topic map (per robot)

| Node | Where | Pub | Sub |
|------|-------|-----|-----|
| `waypoint_follower` | Mac (Webots) | `/robot_i/pose`, `/robot_i/barrier` | `/robot_i/drive` |
| `mission_coordinator` | container | `/robot_i/vertex/tx`, `/robot_i/drive`, `/robot_i/mission_state` | `/robot_i/vertex/event`, `/robot_i/pose`, `/robot_i/barrier` |
| `vertex_node` | container | `/robot_i/vertex/event` | `/robot_i/vertex/tx` |

### Automated test (headless, CI-ready, no Webots)

`route_exploration.launch_test.py` asserts the scenario against **real 4-node
Vertex consensus**, using a headless `mock_robot` in place of Webots (which has
no arm64 Linux build). Route R1 is physically blocked; the fleet explores in
parallel, discovers the block, and still gets every bot to the end.

```bash
docker compose run --rm sim simtest
```

Asserts (application layer of `../README.md` §7):
- **All reach the end**: every robot ends `arrived`, `phase == done`.
- **Blocking discovered**: `R1` appears in every robot's `blocked` set.
- **Route exclusivity**: no snapshot ever shows two bots assigned one route.
- **Byte-identical consensus**: all 4 `/vertex/event` streams match exactly.
- **Clean shutdown**: every process exits 0 / `-SIGINT` / `-SIGTERM`.

### What's verified

- **Consensus decision logic**: `nodes/mission_fsm.py`, 13 unit tests
  (`python3 nodes/test_mission_fsm.py`). The same ordered log yields identical
  assignments and state on every robot.
- **Automated integration**: `simtest` passes in the container: 4× real
  `vertex_node` + coordinators + mock robots run the parallel rules to
  completion, then survive a SIGKILLed explorer (lease recovery, f=1) and a
  mid-mission deactivate/activate cycle (no `Inactive` leakage). The
  randomized mission soak is `simsoak` (see `../README.md` §8).
- **Worlds + physics + devices**: both worlds load/run in Webots R2025a.
- **Native follower + bridge client**: loads in Webots on the Mac
  (`websocket-client`), idles cleanly with no rosbridge, retries to connect.
- **Live physical run**: the two-terminal run above, exercised headless via
  the autotest fixture in `route_supervisor.py`: launch Webots with
  `WEBOTS_AUTOTEST=turn3` (open R2, then flip to R1-only mid-mission) or
  `WEBOTS_AUTOTEST=open<k>` (exclusive-open route k). The supervisor logs
  position telemetry, flags any pair of cars closer than 0.24 m, and prints
  `SUCCESS` once all four cars reach the goal area. turn3 completes with zero
  collisions and byte-identical per-node consensus logs.

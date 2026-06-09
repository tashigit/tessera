# vertex_ros2

The ROS 2 node (`vertex_node`) for the Vertex consensus integration. It maps the
tested [`vertex_core`](../vertex_core) controller onto the ROS-facing contract.
Built by **colcon** (`ament_cargo`) inside a ROS 2 workspace — a bare `cargo build`
outside colcon will not resolve `rclrs`/the generated message crates.

## ROS-facing contract (§3.3)

| Resource | Direction | Type |
|---|---|---|
| `/vertex/tx` | sub | `vertex_ros2_msgs/msg/VertexTransaction` |
| `/vertex/event` | pub | `vertex_ros2_msgs/msg/VertexEvent` |
| `/vertex/sync_point` | pub | `vertex_ros2_msgs/msg/VertexSyncPoint` |
| `/vertex/status` | service | `vertex_ros2_msgs/srv/VertexStatus` |
| `/diagnostics` | pub | `diagnostic_msgs/msg/DiagnosticArray` |
| `/vertex/transition` | service | `vertex_ros2_msgs/srv/VertexTransition` (lifecycle control) |
| `/vertex/lifecycle/state` | pub (latched) | `std_msgs/msg/String` |

## Lifecycle (the rclrs gap, §8.4)

`rclrs` ships no `LifecycleNode`. **Re-checked 2026-06-09** (TAS-76 scope item 2)
against `ros2-rust/ros2_rust` `main` (post-v0.7.0): `rclrs/src` still has no
`lifecycle`/`LifecycleNode` module — it provides nodes, services, parameters,
timers, and (newly) actions, but no managed-node support. **Decision: keep the
`/vertex/transition` service fallback** (§8.4 fallback #2); revisit when upstream
lands lifecycle nodes.

`vertex_node` therefore runs the canonical managed-node state machine
(`vertex_core::lifecycle`, the same one a real `LifecycleNode` would drive) and
exposes it through the `/vertex/transition` service (verbs `configure`/`activate`/
`deactivate`/`cleanup`/`shutdown`), publishing the primary state on the latched
`/vertex/lifecycle/state` topic.

> This is a **sibling control plane**, not the `ros2 lifecycle` CLI. The deviation
> is intentional and documented; migrating to a native `LifecycleNode` later is a
> change localized to this crate (the engine path is unaffected). When it lands
> upstream, the migration is a dependency bump plus swapping the service handlers
> for the native transition callbacks — the state machine is unchanged.

## Build

```console
# in a ROS 2 Jazzy workspace with ros2-rust set up (colcon-cargo + colcon-ros-cargo):
#   pip install git+https://github.com/colcon/colcon-cargo.git \
#               git+https://github.com/colcon/colcon-ros-cargo.git
cd ~/ros2_ws/src && ln -s /path/to/tessera/vertex_ros2_msgs . \
                 && ln -s /path/to/tessera/vertex_ros2 . \
                 && ln -s /path/to/tessera/vertex_core .
cd ~/ros2_ws && colcon build --packages-up-to vertex_ros2
source install/setup.bash
```

## Run & bring-up

```console
# 1. start the node (begins in `unconfigured`)
ros2 run vertex_ros2 vertex_node --ros-args \
  -p vertex.bind_address:=127.0.0.1:9000 \
  -p vertex.secret_key_path:=/etc/vertex/node.key \
  -p "vertex.peers:=['<pubkey>@127.0.0.1:9001','<pubkey>@127.0.0.1:9002']"

# 2. drive the lifecycle
ros2 service call /vertex/transition vertex_ros2_msgs/srv/VertexTransition "{transition: configure}"
ros2 service call /vertex/transition vertex_ros2_msgs/srv/VertexTransition "{transition: activate}"

# 3. submit / observe
ros2 topic pub --once /vertex/tx vertex_ros2_msgs/msg/VertexTransaction "{payload: [1,2,3]}"
ros2 topic echo /vertex/event
ros2 service call /vertex/status vertex_ros2_msgs/srv/VertexStatus
```

## Parameters (§6, §4.6)

| Parameter | Type | Default | Purpose |
|---|---|---|---|
| `vertex.bind_address` | string | *(required)* | Literal `IP:port` to bind (no DNS) |
| `vertex.secret_key_path` | string | `""` | File holding the base58 secret key (**recommended**, §5.1) |
| `vertex.secret_key_base58` | string | `""` | Inline base58 secret key (fallback) |
| `vertex.peers` | string[] | `[]` | `<base58_pubkey>@<ip:port>` specs |
| `vertex.joining_running_session` | bool | `false` | Rejoin an in-flight session (§4.4) |
| `bridge.tx_channel_capacity` | int | 1024 | ROS→Vertex bound; over-bound ⇒ `tx_rejected_total` |
| `bridge.event_channel_capacity` | int | 4096 | Vertex→ROS bound |
| `diagnostics.period_s` | double | 1.0 | `/diagnostics` publish period |
| `options.*` | various | Vertex defaults | One per `tashi_vertex::Options` setter (heartbeat, ack latencies, epoch sizing, hole-punching, …) |

## Note on the rclrs adapter

All logic the test suite verifies lives in `vertex_core`. This crate is the thin
`rclrs` adapter: it targets the rclrs/Jazzy API and is compiled by colcon. The
`rclrs` symbol/QoS/callback specifics may need minor adjustment to the exact
distribution you pin; the contract (topics, types, parameters, lifecycle verbs)
is fixed.

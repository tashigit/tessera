# Tessera — Vertex for ROS 2

Integrate [Tashi **Vertex**](../tashi-vertex-rs/) — a hashgraph-based BFT consensus
engine — into ROS 2. Any ROS 2 node can submit opaque bytes on `/vertex/tx` and
receive a **totally-ordered, cryptographically-final** stream of those bytes
(grouped into Vertex events) on `/vertex/event`.

This is the v0.1 implementation of the design in
[`../Docs/ROS2_Vertex_Integration_Design.md`](../Docs/ROS2_Vertex_Integration_Design.md)
(Linear ticket **TAS-69**). Section references below (e.g. *§4.7*) point into that
document.

> The project is named *tessera* (a single tile in a mosaic): each consensus
> event is one ordered tile; together they form the agreed-upon history.

---

## Why the split into two Rust crates

`tashi-vertex`'s FFI handles (`Engine`, `Context`, `Socket`, …) wrap `NonNull` and
are therefore `!Send`, and the developer-facing API is Rust (design G5). Meanwhile
`rclrs` and the generated message crates only exist inside a colcon/ROS 2
workspace. To keep the **correctness-critical logic testable without a ROS 2
install**, the workspace is two Rust crates plus the message package:

| Package | Build system | What it is | Verified by |
|---|---|---|---|
| **`vertex_ros2_msgs`** | `ament_cmake` | `.msg`/`.srv` contract (§4.5) | colcon |
| **`vertex_core`** | plain `cargo` | All ROS-agnostic logic: lifecycle state machine, the Tokio↔channel bridge, the engine task, config parsing, Vertex→record conversion | **`cargo test` (runs here)** |
| **`vertex_ros2`** | `ament_cargo` | The thin `rclrs` node (`vertex_node`): pub/sub/services/params/diagnostics that map `vertex_core` onto ROS | colcon (ROS 2 Jazzy) |

```
tessera/
├── vertex_ros2_msgs/   # VertexTransaction/Event/SyncPoint + VertexStatus/VertexTransition srv
├── vertex_core/        # tested core (no ROS dependency)
│   ├── src/{lifecycle,bridge,engine_task,convert,config,status,controller}.rs
│   ├── build.rs        # bakes an rpath to libtashi-vertex so test binaries load it
│   └── tests/single_node.rs
└── vertex_ros2/        # rclrs node, built by colcon
    └── src/{main,node,params}.rs
```

---

## What is verified locally vs. what needs ROS 2

**Verified here, no ROS 2 required** (`cd vertex_core && cargo test`):

- **`multi_node`** — the headline guarantee (design G4 / §7): **three real
  in-process Vertex engines** networked over localhost UDP, txs submitted from
  every peer, and all three `/vertex/event` streams asserted **byte-for-byte
  identical**. This is the consensus-ordering acceptance criterion the v0.1
  single-node test could not prove.
- **`single_node`** — full engine path against the **real `libtashi-vertex`**:
  submit → round-trip on the event channel, every tx delivered exactly once
  (delivery integrity / no double-free, §7), counters correct, clean teardown.
- **`lifecycle_behavior`** — the event stream closes when the node leaves
  `Active` (no publishes when not Active, §7); and a `deactivate → activate`
  cycle on the same bind address succeeds without a process restart.
- Lifecycle state-machine transitions/guards (§4.4), time conversion (§4.5),
  status counters (§5.2), config & peer-spec parsing, every `Options` setter (§6).

```console
$ cd vertex_core && cargo test
test result: ok. 18 passed; 0 failed   # unit
test result: ok. 2 passed; 0 failed    # lifecycle_behavior
test result: ok. 1 passed; 0 failed    # multi_node (3 real engines, byte-for-byte)
test result: ok. 1 passed; 0 failed    # single_node (real engine round-trip)
```

The **ROS-level** equivalents (3 `vertex_node` processes, no-publish-in-Inactive,
10-min soak) live in `vertex_ros2/test/` as `launch_test`s for CI on Jazzy.

**Requires a ROS 2 Jazzy workspace** (built by colcon, *not* by the local `cargo`):
the `vertex_ros2` node crate (`rclrs` + generated messages). See
[`vertex_ros2/README.md`](vertex_ros2/README.md) for the colcon build and a
bring-up walkthrough.

---

## Threading model (the hot path) — §4.3, §4.7

`recv_message()` in `tashi-vertex` is **not cancellation-safe**: its future
registers a pointer to itself with the C library on first poll, so dropping it
mid-flight (as `tokio::select!` does to the losing branch) makes a later callback
write into freed memory — a segfault. The design's single-`select!` sketch (§4.7)
does not survive contact with this FFI.

`vertex_core` therefore runs the engine on **one dedicated thread** (the handles
are `!Send`) with **two cooperative tasks** on a `LocalSet` sharing an
`Rc<Engine>`:

- a **recv loop** that calls `recv_message().await` in a plain loop and is *never*
  cancelled mid-flight (it exits between messages via a stop flag, or when the
  outbound channel closes — Vertex's ≤500 ms heartbeat guarantees it wakes);
- a **send loop** that `select!`s only over cancellation-safe futures.

Ordering is still exact: one recv loop forwards events in `recv_message` order,
the bounded channel is FIFO, and `Event::transactions()` iterates by index.

---

## Open gaps (upstream `tashi-vertex`)

These can't be fully closed inside this repo; each is small (design §9).

1. **`whitened_signature()` segfaults.** `tv_event_get_whitened_signature`
   dereferences a null pointer for the events we observe (the upstream `pingback`
   example never calls it). We cannot guard a segfault, so v0.1 leaves
   `VertexEvent.whitened_signature` **empty** and tracks the fix upstream. Re-enable
   the one commented line in `vertex_core/src/convert.rs` once fixed.
2. **`Transaction` Drop leaks** (§9.1) — we allocate immediately before send, so we
   never drop an unsent buffer.
3. **No `Engine::stop`** (§9.3) — teardown drops the `Context` on the engine thread
   after the recv loop exits; a `deactivate → activate` cycle re-creates the engine
   from scratch (verified by `lifecycle_behavior`). Because `recv_message` can't be
   cancelled, an engine that has *stalled* (lost quorum, no more messages) can never
   observe the stop flag, so `deactivate`/`shutdown` waits a bounded
   `ENGINE_STOP_TIMEOUT` (5 s) and then **detaches** the thread rather than hang
   (reaped at process exit). A healthy engine stops well within the bound. A real
   `Engine::stop` upstream would make this deterministic.
4. **`SyncPoint` has no accessors** (§9.2) — `VertexSyncPoint.payload` is empty.
5. **Event timestamp unit** (§9.5) — pinned to **nanoseconds** in
   `convert::nanos_to_time`, the single place to change if upstream differs.

## Semantics worth knowing

- Vertex guarantees a **consistent total order across all peers**, *not* that
  consensus order equals submission order. The single-node test asserts delivery
  integrity (the multiset); cross-peer order agreement (G4) is the multi-node
  system test's job (§5.3 layer 3).
- Vertex payloads are **opaque**. The inbound `VertexTransaction.created_at`/`tag`
  are advisory and ignored in v0.1; outbound per-transaction `created_at`/`tag`
  are not recoverable and are left at defaults.

## License

Apache-2.0.

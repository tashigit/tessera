# Tessera: Vertex for ROS 2

Integrate [Tashi **Vertex**](https://github.com/tashigit/tashi-vertex-rs), a
hashgraph-based BFT consensus engine, into ROS 2. Any ROS 2 node can submit opaque bytes on `/vertex/tx` and
receive a **totally-ordered, cryptographically-final** stream of those bytes
(grouped into Vertex events) on `/vertex/event`.

This implements the ROS 2 + Vertex integration design. Section references
below (e.g. *§4.7*) point into the design document that guided the
implementation.

> The project is named *tessera* (a single tile in a mosaic): each consensus
> event is one ordered tile; together they form the agreed-upon history.

**Documentation:** a four-volume tutorial series lives in [`docs/`](docs/)
(servable with GitHub Pages): concepts and protocol design, building on the
`vertex_fleet` API, crate-level embedding, and porting a broker-based fleet.
Consumers start at [`CONSUMING.md`](CONSUMING.md). Two worked examples ship in
the repo: the [route-exploration simulation](vertex_ros2/test/simulation/) and
the [arena-exploration port](vertex_ros2/test/simulation_arena/).

---

## Workspace layout: why the core and the node are separate crates

`tashi-vertex`'s FFI handles (`Engine`, `Context`, `Socket`, …) wrap `NonNull` and
are therefore `!Send`, and the developer-facing API is Rust (design G5). Meanwhile
`rclrs` and the generated message crates only exist inside a colcon/ROS 2
workspace. To keep the **correctness-critical logic testable without a ROS 2
install**, the Rust side is split into a plain-`cargo` core and a thin
colcon-built node. With the message contract and the Python application
library on top, the workspace is four packages:

| Package | Build system | What it is | Verified by |
|---|---|---|---|
| **`vertex_ros2_msgs`** | `ament_cmake` | `.msg`/`.srv` contract (§4.5) | colcon |
| **`vertex_core`** | plain `cargo` | All ROS-agnostic logic: lifecycle state machine, the Tokio↔channel bridge, the engine task, config parsing, Vertex→record conversion | **`cargo test` (runs here)** |
| **`vertex_ros2`** | `ament_cargo` | The thin `rclrs` node (`vertex_node`): pub/sub/services/params/diagnostics that map `vertex_core` onto ROS | colcon (ROS 2 Jazzy) |
| **`vertex_fleet`** | `ament_python` | The application library: the `ReplicatedState` fold, the `VertexAgent` base node, and the ledger example | python unit tests + the `ledger_demo` launch_test |

```
tessera/
├── vertex_ros2_msgs/   # VertexTransaction/Event/SyncPoint + VertexStatus/VertexTransition srv
├── vertex_core/        # tested core (no ROS dependency)
│   ├── src/{lifecycle,bridge,engine_task,convert,config,status,controller}.rs
│   ├── build.rs        # bakes an rpath to libtashi-vertex so test binaries load it
│   └── tests/{single_node,multi_node,lifecycle_behavior,tx_saturation}.rs
├── vertex_ros2/        # rclrs node, built by colcon
│   └── src/{main,node,params}.rs
└── vertex_fleet/       # consumer library (ReplicatedState + VertexAgent) + examples
```

---

## What is verified locally vs. what needs ROS 2

**Verified here, no ROS 2 required** (`cd vertex_core && cargo test`):

- **`multi_node`**, the headline guarantee (design G4 / §7): **three real
  in-process Vertex engines** networked over localhost UDP, txs submitted from
  every peer, and all three `/vertex/event` streams asserted **byte-for-byte
  identical**. This is the consensus-ordering acceptance criterion the v0.1
  single-node test could not prove.
- **`single_node`**: full engine path against the **real `libtashi-vertex`**:
  submit → round-trip on the event channel, every tx delivered exactly once
  (delivery integrity / no double-free, §7), counters correct, clean teardown.
- **`tx_saturation`**: floods a deliberately tiny inbound channel with 10× its
  capacity in a tight loop and asserts `tx_submitted_total + tx_rejected_total`
  equals the number submitted (backpressure accounting is leak-free, §7).
- **`lifecycle_behavior`**: the event stream closes when the node leaves
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
test result: ok. 1 passed; 0 failed    # tx_saturation (backpressure accounting)
```

The **ROS-level** equivalents (3 `vertex_node` processes, no-publish-in-Inactive,
10-min soak) live in `vertex_ros2/test/` as `launch_test`s for CI on Jazzy
(`docker/` + `docker-compose.yml` run them on any machine, incl. Apple Silicon).
The application layer on top is covered by the two simulation suites
(`vertex_ros2/test/simulation*/`), which CI also runs headless on every push.

**Miri** (design §7 "Miri'd in CI"): Miri cannot execute the `libtashi-vertex`
C FFI, so it runs over the **FFI-free modules only** (`lifecycle`, `convert`):
`cargo +nightly miri test --lib lifecycle` / `--lib convert`, wired into CI. The
property Miri was meant to back (no double-send/double-free of tx buffers) is
instead guaranteed **statically**: `Transaction` is move-only and `mem::forget`'d
on send, so it cannot be double-freed or sent twice from safe Rust.

**Requires a ROS 2 Jazzy workspace** (built by colcon, *not* by the local `cargo`):
the `vertex_ros2` node crate (`rclrs` + generated messages). See
[`vertex_ros2/README.md`](vertex_ros2/README.md) for the colcon build and a
bring-up walkthrough.

---

## Threading model, the hot path (§4.3, §4.7)

`recv_message()` in `tashi-vertex` is **not cancellation-safe**: its future
registers a pointer to itself with the C library on first poll, so dropping it
mid-flight (as `tokio::select!` does to the losing branch) makes a later callback
write into freed memory and segfault. The design's single-`select!` sketch (§4.7)
does not survive contact with this FFI.

`vertex_core` therefore runs the engine on **one dedicated thread** (the handles
are `!Send`) with **two cooperative tasks** on a `LocalSet` sharing an
`Rc<Engine>`:

- a **recv loop** that calls `recv_message().await` in a plain loop and is *never*
  cancelled mid-flight. It exits between messages via a stop flag, or when the
  outbound channel closes, and Vertex's ≤500 ms heartbeat guarantees it wakes;
- a **send loop** that `select!`s only over cancellation-safe futures.

Ordering is still exact: one recv loop forwards events in `recv_message` order,
the bounded channel is FIFO, and `Event::transactions()` iterates by index.

---

## Open gaps (upstream `tashi-vertex`)

These can't be fully closed inside this repo; each is small (design §9).

1. **`whitened_signature()`: resolved against v0.14.0.** An earlier
   (pre-0.14.0) build segfaulted in `tv_event_get_whitened_signature`; v0.14.0
   returns the fixed-length signature reliably for every event.
   `VertexEvent.whitened_signature` is now **populated**
   (`vertex_core/src/convert.rs`); `single_node` asserts it is non-empty and
   `multi_node` diffs it byte-for-byte across peers (it is consensus-intrinsic).
2. **`Transaction` Drop leaks** (§9.1): we allocate immediately before
   send, so we never drop an unsent buffer. A `Drop` calling `tv_free`
   is not a safe fix, because transaction buffers are not `tv_free`-compatible.
   The real fix needs a dedicated `tv_transaction_free` upstream.
3. **No `Engine::stop`** (§9.3): teardown drops the `Context` on the engine thread
   after the recv loop exits; a `deactivate → activate` cycle re-creates the engine
   from scratch (verified by `lifecycle_behavior`). Because `recv_message` can't be
   cancelled, an engine that has *stalled* (lost quorum, no more messages) can never
   observe the stop flag, so `deactivate`/`shutdown` waits a bounded
   `ENGINE_STOP_TIMEOUT` (5 s) and then **detaches** the thread rather than hang
   (reaped at process exit). A healthy engine stops well within the bound. A real
   `Engine::stop` upstream would make this deterministic.
4. **`SyncPoint` has no accessors** (§9.2): `VertexSyncPoint.payload` is empty.
5. **Event timestamp unit** (§9.5): pinned to **nanoseconds** in
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

# vertex_core

The ROS-agnostic core of the Vertex / ROS 2 integration. Plain `cargo` crate — no
ROS dependency — so the correctness-critical logic is testable directly against
the real `libtashi-vertex`.

```console
cargo test
```

## Modules

| Module | Responsibility |
|---|---|
| `lifecycle` | Managed-node state machine (`Unconfigured`/`Inactive`/`Active`/`Finalized`) + transition guards (§4.4) |
| `config` | Typed config; maps parameters → `Options`/`Peers`/`KeySecret` (§6) |
| `bridge` | The bounded Tokio mpsc channels between ROS and the engine thread (§4.3) |
| `engine_task` | The two cooperative tasks owning `Engine` — the cancellation-safe receive design (§4.7) |
| `convert` | `tashi_vertex::Event` → plain records mirroring `vertex_ros2_msgs`; time conversion (§4.5) |
| `status` | Lock-free `ArcSwap` status snapshot + counters (§5.2) |
| `controller` | Drives the lifecycle callbacks; owns the engine thread; `submit()` |

## `build.rs`

`tashi-vertex`'s build script only adds an `rpath` to *its own* test binaries, so
downstream test binaries can't find `libtashi-vertex.dylib`/`.so` at runtime. Our
`build.rs` locates the dylib under the shared target dir and bakes an absolute
`rpath` (plus relative fallbacks). Without it, tests fail to dynamically link.

## Key design notes

- **`recv_message` is not cancellation-safe** — see the module docs in
  `engine_task.rs` and the workspace README. This is why there is no single
  `select!` loop.
- **`Engine`/`Context` are `!Send`** — the engine lives on one dedicated thread;
  only `Send` data crosses the channel boundary.
- **`whitened_signature()` segfaults upstream** — left empty in v0.1 (see workspace
  README "Open gaps").

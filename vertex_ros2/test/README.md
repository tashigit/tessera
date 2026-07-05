# vertex_ros2 system tests

ROS-level acceptance tests (design §7). They launch real
`vertex_node` processes and require **ROS 2 (Jazzy baseline)** + a colcon-built
workspace — they do **not** run in a plain `cargo test` sandbox.

> The same consensus-ordering guarantee these tests assert at the ROS level is
> also verified, with no ROS dependency, by `vertex_core/tests/multi_node.rs`
> (three real in-process engines, byte-for-byte event-stream diff). That core
> test is the fast, always-run proof; these are the end-to-end equivalents.

## Files

| File | Acceptance criterion |
|---|---|
| `system_three_peers.launch_test.py` | No publishes while `Inactive`; all 3 peers' `/vertex/event` streams byte-for-byte identical; lifecycle `configure→activate→deactivate→activate` without crash |
| `soak.launch_test.py` | No unbounded memory growth under load (RSS bound after a 60 s warm-up) |
| `gen_test_keys.sh` | Generates `fixtures/peers.json` (3 keypairs + bind addresses) using the tashi-vertex-rs `key-generate` example |

## Run

```console
# 1. one-time: generate peer keys (point VERTEX_RS at the tashi-vertex-rs checkout)
./test/gen_test_keys.sh

# 2. the multi-node test (part of `colcon test`)
colcon test --packages-select vertex_ros2
#   or directly:
launch_test src/vertex_ros2/test/system_three_peers.launch_test.py

# 3. the soak test (long; explicit opt-in)
pip install psutil
SOAK_SECONDS=600 launch_test src/vertex_ros2/test/soak.launch_test.py
```

## Notes

- The three nodes are remapped into `/v0`, `/v1`, `/v2` namespaces so they
  coexist on one host (the contract topics are otherwise absolute, §3.3).
- **Miri** (the design's suggested check for "no double-send/double-free") cannot
  execute over the FFI-linked engine, so that criterion is instead covered by:
  the move-only `Transaction` type (compile-time), the `multi_node.rs` round-trip
  (no double-free under real load), and running the soak under ASan/Valgrind in
  CI if deeper coverage is wanted.

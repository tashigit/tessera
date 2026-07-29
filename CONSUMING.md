# Using the ROS 2 + Vertex integration in your project

This guide gets an external ROS 2 team from zero to a working consensus
fleet: the `vertex_node` mesh, plus your first application agent built on the
`vertex_fleet` library. It is the consumer view. Contributor docs live in the
per-package READMEs.

What you get:

| Package | What it is |
|---|---|
| `vertex_ros2_msgs` | the wire contract: `VertexTransaction`, `VertexEvent`, the `VertexTransition`/`VertexStatus` services |
| `vertex_ros2` | the `vertex_node` binary wrapping the Tashi Vertex consensus engine. One per agent. Opaque bytes in on `/vertex/tx`, a byte-identical totally-ordered stream out on `/vertex/event`, on every peer |
| `vertex_fleet` | the application library: `VertexAgent` (lifecycle bring-up, single-mutation-path fold, epoch-stamped proposals) and `ReplicatedState` (deterministic fold with epoch/reset semantics), plus a runnable ledger example |
| `vertex_core` | the same engine integration as a plain Rust crate, for non-ROS embeddings |

## 1. Prerequisites

- ROS 2 Jazzy (or use the Docker harness below and skip a native install)
- Rust (stable) and CMake, for building the engine bindings
- Git access to `tashigit/tessera` and `tashigit/tashi-vertex-rs` (both
  private, so an org membership or a read-scoped PAT)

## 2. Workspace

The two repositories must sit side by side (`tashi-vertex-rs` is a path
dependency). One step with vcs:

```bash
mkdir -p vertex_ws/src && cd vertex_ws
curl -fsSL https://raw.githubusercontent.com/tashigit/tessera/main/tessera.repos -o tessera.repos
vcs import src < tessera.repos
```

Your own packages go in `src/` next to them.

The repos file on `main` tracks `main`. For a reproducible build, replace the
two `version:` fields with a release: the tessera tag, and the
`tashi-vertex-rs` commit recorded in that release's notes (see
`RELEASING.md`). Releases also ship a prebuilt Jazzy install-space tarball if
you want to skip building entirely.

## 3. Build

Native (a sourced Jazzy environment):

```bash
colcon build --packages-up-to vertex_ros2 vertex_fleet \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Or the ready-made Docker harness (nothing on the host but Docker):

```bash
cd src/tessera
docker compose build
docker compose run --rm build
```

Two build notes:

- The engine bindings download the pinned `libtashi-vertex` release archive
  during the build. To build against a local `tashi-vertex-c` checkout, set
  `TASHI_VERTEX_LOCAL_DIR` to it.
- The installed `vertex_node` carries an rpath to `libtashi-vertex` inside
  the workspace build tree (plus `$ORIGIN`-relative fallbacks), so it runs
  with no environment setup as long as the workspace stays where it was
  built. If you relocate a built workspace, either rebuild or fall back to
  exporting the library directory:

```bash
export LD_LIBRARY_PATH="$(dirname "$(find build install -name 'libtashi-vertex.so' | head -n1)"):$LD_LIBRARY_PATH"
```

## 4. First mesh

Every agent needs an Ed25519 keypair and a UDP bind address. Generate
identities with the helper (adapt `-n` to your fleet size):

```bash
bash src/tessera/vertex_ros2/test/gen_test_keys.sh    # writes peers.json (3 peers)
```

Then launch one `vertex_node` per agent, each remapped into its own
namespace and peered with the others. The canonical launch pattern is
`vertex_ros2/test/simulation/route_exploration.launch.py`; the short form:

```python
Node(package="vertex_ros2", executable="vertex_node", name=f"vertex{i}",
     remappings=[(f"/vertex/{t}", f"/agent_{i}/vertex/{t}")
                 for t in ("tx", "event", "sync_point", "status", "transition")]
               + [("/vertex/lifecycle/state", f"/agent_{i}/vertex/lifecycle/state")],
     parameters=[{
         "vertex.bind_address": me["addr"],
         "vertex.secret_key_path": key_path,   # a 0600 file holding me["secret"]
         "vertex.peers": [f"{p['public']}@{p['addr']}" for p in others],
         "options.heartbeat_us": 50000,
     }])
```

Always use `secret_key_path`, never `secret_key_base58`: once a value is
declared as a ROS 2 parameter it is readable by any DDS participant on the
mesh via `ros2 param get`/`ros2 param dump`, so inlining the private key
there hands it to anyone on the network. Write the key to a file the node
user owns exclusively (`chmod 600`) and point `secret_key_path` at it —
`vertex_node` rejects the file outright if group/other can read or write it.

Size the fleet for the faults you must survive: Vertex is BFT with the
standard quorum, so `n = 4` is the smallest fleet that tolerates one crashed
agent (`n >= 3f + 1`).

## 5. First agent

Build on `vertex_fleet`. An application is one pure state class and one node
class:

```python
from vertex_fleet import ReplicatedState, VertexAgent, spin_agent

class CounterState(ReplicatedState):
    def wipe(self):
        self.total = 0
    def apply_record(self, rec):          # pure and idempotent
        if rec.get("op") == "add":
            self.total += int(rec.get("n", 0))

class CounterAgent(VertexAgent):
    def __init__(self):
        super().__init__("counter_agent", CounterState())
    def tick(self):                       # called once the engine is Active
        if self.state.total < 100:
            self.propose({"op": "add", "n": 1})

def main():
    spin_agent(CounterAgent)
```

The library enforces the structural rules that keep a fleet convergent: the
state mutates only in the `/vertex/event` fold, proposals are epoch-stamped
and take effect only once finalized, and `reset` records restart the whole
fleet at a fresh epoch through consensus.

The complete worked example is the replicated ledger:
`vertex_fleet/vertex_fleet/examples/ledger_agent.py`, with its launch and
assertions in `vertex_fleet/test/ledger_demo.launch_test.py`. The full-scale
consumer is the route-exploration simulation
(`vertex_ros2/test/simulation/`): a four-robot fleet built on the same two
base classes. Run the ledger against real consensus in the harness:

```bash
docker compose run --rm test          # includes the ledger integration test
```

## 6. Testing your application

Copy the two test shapes the repo uses everywhere:

- unit test your `ReplicatedState`: feed the same record log to several
  instances and assert identical state (`vertex_fleet/test/test_state.py`)
- integration test with real engines under `launch_testing` and assert the
  byte-identical `/vertex/event` streams across your agents
  (`vertex_fleet/test/ledger_demo.launch_test.py`)

## 7. Further reading

- Application-level tutorial (protocol design, worked robot-fleet example):
  the interactive guide that accompanies this repo, and
  `vertex_ros2/test/simulation/README.md`
- Engine contract reference (topics, parameters, lifecycle):
  `vertex_ros2/README.md`
- Embedding without ROS (Rust crate, custom middleware adapters):
  `vertex_core/README.md`

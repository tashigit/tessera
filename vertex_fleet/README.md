# vertex_fleet

The application library for building consensus-coordinated ROS 2 agents on
the Tashi Vertex integration. Consumers start at `CONSUMING.md` in the
repository root.

| Module | Responsibility |
|---|---|
| `vertex_fleet.state` | `ReplicatedState`: the deterministic fold with epoch and `reset` semantics, plus the canonical `encode`/`decode` for opaque payload records. Pure Python, imports without ROS |
| `vertex_fleet.agent` | `VertexAgent`: base node wiring an agent to its `vertex_node` (lifecycle bring-up, single-mutation-path event fold, epoch-stamped `propose`), and the `spin_agent` main helper |
| `vertex_fleet.examples.ledger_state` / `ledger_agent` | the minimal worked application: a replicated append-only ledger. Copy this pair (pure state module, thin node module) as the starting shape for a new app |

Two consumers ship in this repository: the ledger example (minimal) and the
route-exploration simulation (`vertex_ros2/test/simulation/`), whose
`mission_fsm.py` / `mission_coordinator.py` build the full robot-fleet
scenario on this library. Read the ledger first, the simulation second.

Tests:

```bash
python3 test/test_state.py         # pure fold semantics, any host, no ROS
docker compose run --rm test       # includes test/ledger_demo.launch_test.py:
                                   # the ledger example on 3 real vertex_nodes
```

The structural rules the library enforces (and the reasoning behind them) are
documented in the two integration tutorials and in
`vertex_ros2/test/simulation/README.md`.

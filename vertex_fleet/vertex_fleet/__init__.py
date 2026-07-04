"""vertex_fleet: build consensus-coordinated ROS 2 agents on the Tashi Vertex
integration (vertex_ros2).

    from vertex_fleet import ReplicatedState, VertexAgent, spin_agent

The state layer (ReplicatedState, encode, decode) is pure Python and imports
without ROS, so application state machines stay unit-testable on any host.
VertexAgent and spin_agent load lazily and require rclpy.

See vertex_fleet.examples.ledger_agent for a complete minimal consumer and
CONSUMING.md at the repository root for the quickstart.
"""

from .state import ReplicatedState, decode, encode

__all__ = ["ReplicatedState", "VertexAgent", "spin_agent", "decode", "encode"]


def __getattr__(name):
    # Lazy: keep the pure state layer importable on hosts without ROS.
    if name in ("VertexAgent", "spin_agent"):
        from . import agent
        return getattr(agent, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

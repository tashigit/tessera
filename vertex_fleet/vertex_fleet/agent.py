"""VertexAgent: the base node for a consensus-coordinated agent.

Wires one agent to its vertex_node (the Tashi Vertex consensus engine) and
enforces the structural rules that make fleet coordination correct:

  * the agent's ReplicatedState mutates in EXACTLY ONE place, the
    /vertex/event callback, so shared state can never diverge from the log
  * proposals go out epoch-stamped through ``propose()`` and take effect only
    when they come back finalized (propose, then wait: never assume your own
    proposal succeeded)
  * the agent brings its own vertex_node to Active through the
    /vertex/transition lifecycle service, retrying until the engine runs

Topics resolve relative to the node's namespace, so the standard deployment
is one namespace per agent (/agent_0/vertex/tx, /agent_0/vertex/event, ...)
with the vertex_node contract topics remapped into it. See
vertex_ros2/test/simulation/route_exploration.launch.py for the launch
pattern and vertex_fleet/examples/ledger_agent.py for a complete consumer.

Subclass, pass a ReplicatedState to ``__init__``, and override:

    tick()               periodic hook, called only while the engine is
                         Active; read self.state, call self.propose(...)
    on_event(msg)        called after each consensus event is folded, with
                         the raw VertexEvent (hash, timestamps, transactions)
    on_state_changed()   called after on_event; the state-level hook

A state whose construction needs ROS parameters cannot exist before the node
does. Pass ``state=None`` and assign ``self.state`` right after
``super().__init__(...)``, before spinning; no callback runs until then.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from vertex_ros2_msgs.msg import VertexEvent, VertexTransaction
from vertex_ros2_msgs.srv import VertexTransition

from .state import ReplicatedState, decode, encode


class VertexAgent(Node):
    def __init__(self, node_name: str, state: ReplicatedState | None,
                 tick_period_sec: float = 0.2):
        super().__init__(node_name)
        self.state = state          # may be assigned by the subclass pre-spin
        self._event_count = 0

        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._tx_pub = self.create_publisher(
            VertexTransaction, "vertex/tx", reliable)
        self.create_subscription(
            VertexEvent, "vertex/event", self._on_event, reliable)

        self._lifecycle = "init"
        self._transition_pending = False
        self._transition_cli = self.create_client(
            VertexTransition, "vertex/transition")

        self.create_timer(tick_period_sec, self._on_timer)

    # ---- public API ----

    @property
    def engine_running(self) -> bool:
        """True once this agent's vertex_node reached Active."""
        return self._lifecycle == "running"

    @property
    def events_folded(self) -> int:
        """Consensus events applied so far (diagnostics)."""
        return self._event_count

    def propose(self, record: dict) -> None:
        """Submit a record to consensus, stamped with the current epoch. It
        has NO local effect: it changes ``self.state`` only if and when it
        returns finalized on /vertex/event, identically on every agent."""
        tx = VertexTransaction()
        tx.payload = list(encode({**record, "epoch": self.state.epoch}))
        self._tx_pub.publish(tx)

    def propose_reset(self, epoch: int) -> None:
        """Restart the whole fleet's state at a fresh epoch, through
        consensus. The first reset in the finalized order wins."""
        tx = VertexTransaction()
        tx.payload = list(encode({"op": "reset", "epoch": int(epoch)}))
        self._tx_pub.publish(tx)

    # ---- subclass hooks ----

    def tick(self) -> None:
        """Periodic hook, called only while the engine is Active."""

    def on_event(self, msg) -> None:
        """Called after each consensus event is folded, with the raw
        VertexEvent (hash, consensus_at, transactions). For logging and
        telemetry; state mutation already happened."""

    def on_state_changed(self) -> None:
        """Called after every consensus event has been folded into state."""

    # ---- internals ----

    def _on_event(self, msg: VertexEvent) -> None:
        # THE single mutation path for self.state.
        for tx in msg.transactions:
            self.state.apply(decode(tx.payload))
        self._event_count += 1
        self.on_event(msg)
        self.on_state_changed()

    def _on_timer(self) -> None:
        if not self._bringup():
            return
        self.tick()

    def _bringup(self) -> bool:
        """Drive this agent's vertex_node to Active (configure -> activate),
        retrying until the transition service is up and both verbs succeed."""
        if self._lifecycle == "running":
            return True
        if self._transition_pending or not self._transition_cli.service_is_ready():
            return False
        verb, next_state = (("configure", "inactive")
                            if self._lifecycle == "init"
                            else ("activate", "running"))
        self._transition_pending = True
        req = VertexTransition.Request()
        req.transition = verb

        def _done(fut):
            self._transition_pending = False
            res = fut.result()
            if res is not None and res.success:
                self._lifecycle = next_state
            else:
                self.get_logger().warn(
                    f"{verb} rejected: {getattr(res, 'message', 'timeout')}")

        self._transition_cli.call_async(req).add_done_callback(_done)
        return False


def spin_agent(agent_factory) -> None:
    """Boilerplate main: init rclpy, construct the agent, spin, tear down
    cleanly on SIGINT (the same shutdown shape the tessera test harness
    expects from launch_test processes)."""
    rclpy.init()
    node = agent_factory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

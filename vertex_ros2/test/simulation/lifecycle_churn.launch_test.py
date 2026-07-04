#!/usr/bin/env python3
# Lifecycle-churn test for the route-exploration scenario (N4): deactivate and
# reactivate one robot's vertex_node via /vertex/transition while the rest of
# the fleet keeps moving and finalizing consensus traffic.
#
# The churn target is the FIRST robot to arrive: it has completed its physical
# task and needs no further consensus decisions, so churning it cannot wedge
# the mission (the plan's "keep the churned robot active across the decisions
# it must act on" — it has none left). While it is Inactive the test injects
# no-op transactions through another node, forcing the live nodes to finalize
# events, and asserts the churned node publishes NOTHING on its /vertex/event.
#
# Asserts:
#   * deactivate and activate both succeed mid-mission;
#   * ZERO messages on the churned robot's /vertex/event while Inactive, while
#     the other nodes demonstrably finalize events in the same window;
#   * the mission still completes: every robot ends up arrived (the churned
#     robot arrived before the churn) and the never-churned robots agree on
#     the winner with byte-identical event streams;
#   * clean shutdown for every process.
#
# Not asserted: that the churned node re-receives events finalized while it
# was Inactive. Rejoining a running session is upstream scope; the
# churned robot needs none of those events because it is already done.
#
# Run in the Jazzy container:  docker compose run --rm sim simtest
# Prerequisite: fixtures/peers4.json (fixtures/gen_peers4.sh).

import json
import os
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vertex_ros2_msgs.msg import VertexEvent, VertexTransaction
from vertex_ros2_msgs.srv import VertexTransition

HERE = os.path.dirname(__file__)
PEERS_PATH = os.path.join(HERE, "fixtures", "peers4.json")
COORDINATOR = os.path.join(HERE, "nodes", "mission_coordinator.py")
MOCK_ROBOT = os.path.join(HERE, "nodes", "mock_robot.py")
ROUTES = ["R1", "R2", "R3", "R4"]
HOME_LANE_Y = [2.25, 0.75, -0.75, -2.25]
BLOCKED = ["R1"]                # bot 0 discovers the block and re-joins


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers4.sh first.")
    with open(PEERS_PATH) as f:
        return json.load(f)[:4]


@pytest.mark.launch_test
def generate_test_description():
    peers = _load_peers()
    actions = []
    for i, me in enumerate(peers):
        ns = f"/robot_{i}"
        others = [p for j, p in enumerate(peers) if j != i]
        peer_specs = [f"{p['public']}@{p['addr']}" for p in others]
        actions.append(launch_ros.actions.Node(
            package="vertex_ros2", executable="vertex_node", name=f"vertex{i}",
            remappings=[(f"/vertex/{t}", f"{ns}/vertex/{t}")
                        for t in ("tx", "event", "sync_point", "status", "transition")]
                      + [("/vertex/lifecycle/state", f"{ns}/vertex/lifecycle/state")],
            parameters=[{
                "vertex.bind_address": me["addr"],
                "vertex.secret_key_base58": me["secret"],
                "vertex.peers": peer_specs,
                "options.heartbeat_us": 50000,
            }],
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=COORDINATOR, name="mission_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "routes": ROUTES, "goal_x": 3.6,
                         "num_bots": 4, "claim_interval_sec": 0.5,
                         "random_routes": False}],
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=MOCK_ROBOT, name="mock_robot", namespace=ns,
            parameters=[{"home_lane_y": HOME_LANE_Y[i],
                         "blocked_routes": BLOCKED}],
            output="screen",
        ))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestLifecycleChurn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("lifecycle_churn_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _transition(self, i, verb, timeout=10.0):
        cli = self.node.create_client(VertexTransition,
                                      f"/robot_{i}/vertex/transition")
        self.assertTrue(cli.wait_for_service(timeout_sec=timeout),
                        f"robot_{i} transition service")
        req = VertexTransition.Request()
        req.transition = verb
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        self.assertIsNotNone(fut.result(), f"robot_{i} {verb} timed out")
        return fut.result()

    def test_churn_mid_mission(self):
        latest = {i: None for i in range(4)}
        events = {i: [] for i in range(4)}

        subs = [self.node.create_subscription(
            String, f"/robot_{i}/mission_state",
            (lambda i: lambda m: latest.__setitem__(i, json.loads(m.data)))(i),
            10) for i in range(4)]
        # Deep queues: the byte-identical comparison below is only meaningful
        # if THIS observer never drops an event during bursts. A shallow depth
        # here once made CI fail with "streams differ" when the real cause was
        # the test's own subscription overflowing and misaligning the prefixes.
        subs += [self.node.create_subscription(
            VertexEvent, f"/robot_{i}/vertex/event",
            (lambda i: lambda m: events[i].append(
                (bytes(m.hash),
                 tuple(bytes(t.payload) for t in m.transactions))))(i),
            1000) for i in range(4)]

        # Phase 1: wait for the first arrival — that robot is the churn target.
        deadline = time.time() + 60.0
        target = None
        while time.time() < deadline and target is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            for i in range(4):
                st = latest[i]
                if st and i in st.get("arrived", []):
                    target = i
                    break
        self.assertIsNotNone(target, f"nobody arrived: {latest}")
        others = [i for i in range(4) if i != target]

        # Phase 2: deactivate the target's vertex_node mid-mission.
        res = self._transition(target, "deactivate")
        self.assertTrue(res.success, f"deactivate: {res.message}")
        target_events_before = len(events[target])
        others_events_before = {i: len(events[i]) for i in others}

        # Force live consensus traffic during the Inactive window: no-op
        # records finalize on the active nodes, so silence on the churned
        # node's /vertex/event is meaningful, not just an idle network.
        # Rate-limited to ~10 noops/s: spin_once returns immediately while
        # events are flowing, so publishing once per loop iteration floods
        # thousands of records on a fast runner and buries the observer.
        # A few dozen finalized noops prove liveness just as well.
        pub = self.node.create_publisher(
            VertexTransaction, f"/robot_{others[0]}/vertex/tx", 10)
        end = time.time() + 3.0
        seq = 0
        last_pub = 0.0
        while time.time() < end:
            if time.time() - last_pub >= 0.1:
                last_pub = time.time()
                msg = VertexTransaction()
                msg.payload = list(json.dumps(
                    {"op": "noop", "seq": seq, "epoch": 0}).encode())
                pub.publish(msg)
                seq += 1
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertEqual(len(events[target]), target_events_before,
                         "churned node published on /vertex/event while Inactive")
        grew = sum(1 for i in others
                   if len(events[i]) > others_events_before[i])
        self.assertGreaterEqual(
            grew, len(others),
            f"active nodes did not finalize during the churn window: "
            f"{ {i: len(events[i]) - others_events_before[i] for i in others} }")

        # Phase 3: reactivate — must succeed while the session is running.
        res = self._transition(target, "activate")
        self.assertTrue(res.success, f"activate: {res.message}")

        # Phase 4: the mission completes regardless of the churn. The churned
        # robot arrived before deactivation; every other robot must also end
        # up arrived, with one agreed winner and byte-identical streams.
        deadline = time.time() + 90.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(latest[i] and i in latest[i].get("arrived", [])
                   for i in range(4)):
                break

        winners = set()
        for i in range(4):
            st = latest[i]
            self.assertIn(i, st.get("arrived", []),
                          f"robot_{i} never arrived: {st}")
            if i != target:
                winners.add(st.get("winner_route"))
                self.assertIsNotNone(st.get("winner_route"), f"robot_{i}: {st}")
        self.assertEqual(len(winners), 1,
                         f"live robots disagree on the winner: {winners}")

        min_len = min(len(events[i]) for i in others)
        self.assertGreater(min_len, 0)
        for idx in range(min_len):
            e0 = events[others[0]][idx]
            for i in others[1:]:
                self.assertEqual(e0, events[i][idx],
                                 f"event #{idx} differs between live nodes")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

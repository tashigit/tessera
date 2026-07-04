#!/usr/bin/env python3
# Automated integration test for the consensus-coordinated route-exploration
# scenario (../README.md §3/§7), headless — no Webots.
#
# Launches, per robot i (0..3):
#   * vertex_node          the crate under test (real Vertex consensus)
#   * mission_coordinator  the replicated FSM
#   * mock_robot           stand-in for waypoint_follower + physics
#
# PARALLEL model: all bots claim routes concurrently; consensus assigns them
# exclusively (no two bots on one route). R1 is physically blocked, R2-R4 open:
# bot0 (home R1) discovers the block and re-joins; the first arrival fixes the
# winner and everyone converges onto it. Asserts (design §7, app layer):
#   * All 4 bots reach the end (phase done) and agree on the same winner route.
#   * R1 is discovered and recorded blocked on every bot.
#   * Route exclusivity: no snapshot ever shows two bots assigned one route.
#   * The 4 vertex_nodes deliver byte-identical /vertex/event streams.
#
# Run in the Jazzy container:  docker compose run --rm sim simtest
# (or:  launch_test vertex_ros2/test/simulation/route_exploration.launch_test.py)
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
from vertex_ros2_msgs.msg import VertexEvent

HERE = os.path.dirname(__file__)
PEERS_PATH = os.path.join(HERE, "fixtures", "peers4.json")
COORDINATOR = os.path.join(HERE, "nodes", "mission_coordinator.py")
MOCK_ROBOT = os.path.join(HERE, "nodes", "mock_robot.py")
ROUTES = ["R1", "R2", "R3", "R4"]
HOME_LANE_Y = [2.25, 0.75, -0.75, -2.25]
BLOCKED = ["R1"]                # R1 is physically blocked; the rest are open


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
                         "random_routes": False}],   # deterministic tie-breaks
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=MOCK_ROBOT, name="mock_robot", namespace=ns,
            parameters=[{"home_lane_y": HOME_LANE_Y[i], "blocked_routes": BLOCKED}],
            output="screen",
        ))

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestRouteExploration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("route_exploration_test")
        cls.n = 4

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_parallel_exploration_all_reach_end(self):
        latest = {i: None for i in range(self.n)}
        exclusivity_violations = []               # snapshots with a route shared
        events = {i: [] for i in range(self.n)}   # per-bot /vertex/event stream

        def make_cb(i):
            def cb(msg):
                st = json.loads(msg.data)
                latest[i] = st
                assigned = st.get("assigned", {})
                if len(set(assigned.values())) != len(assigned):
                    exclusivity_violations.append(assigned)
            return cb

        def make_ev_cb(i):
            def cb(msg):
                events[i].append((bytes(msg.hash),
                                  tuple(bytes(t.payload) for t in msg.transactions)))
            return cb

        subs = [self.node.create_subscription(
            String, f"/robot_{i}/mission_state", make_cb(i), 10) for i in range(self.n)]
        subs += [self.node.create_subscription(
            VertexEvent, f"/robot_{i}/vertex/event", make_ev_cb(i), 1000)
            for i in range(self.n)]

        # Run until every bot has reached the end (phase DONE on all), R1 blocked.
        deadline = time.time() + 120.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(latest[i] and latest[i].get("phase") == "done"
                   for i in range(self.n)):
                break

        winners = set()
        for i in range(self.n):
            st = latest[i]
            self.assertIsNotNone(st, f"robot_{i} never published mission_state")
            # every bot reached the end (rule 8)
            self.assertEqual(sorted(st.get("arrived", [])), [0, 1, 2, 3],
                             f"robot_{i} state shows not everyone arrived: {st}")
            self.assertEqual(st.get("phase"), "done", f"robot_{i}: {st}")
            # the physically-blocked route was discovered and reported (rule 5)
            self.assertIn("R1", st.get("blocked", []), f"robot_{i}: {st}")
            # a proven-open path was found and everyone agrees on it (rule 4)
            self.assertIsNotNone(st.get("winner_route"), f"robot_{i}: {st}")
            winners.add(st.get("winner_route"))
        self.assertEqual(len(winners), 1, f"bots disagree on the winning path: {winners}")
        self.assertNotIn("R1", winners, "winner should be an open route, not R1")

        # Route exclusivity: at no observed instant were two bots assigned the
        # same route (parallel exploration, consensus-arbitrated).
        self.assertEqual(exclusivity_violations, [],
                         f"route shared by two bots: {exclusivity_violations[:3]}")

        # Vertex consensus itself: the 4 vertex_nodes must deliver a BYTE-FOR-BYTE
        # identical, identically-ordered /vertex/event stream (the Tashi total-order
        # guarantee — the foundation the whole scenario relies on).
        min_len = min(len(events[i]) for i in range(self.n))
        self.assertGreater(min_len, 0, "no consensus events were delivered")
        for idx in range(min_len):
            e0 = events[0][idx]
            for i in range(1, self.n):
                self.assertEqual(e0, events[i][idx],
                                 f"vertex event #{idx} differs: bot0={e0} bot{i}={events[i][idx]}")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

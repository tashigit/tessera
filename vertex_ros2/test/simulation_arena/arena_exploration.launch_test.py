#!/usr/bin/env python3
# Automated integration test for the consensus-coordinated ARENA-EXPLORATION
# scenario (the second simulation; see README.md), headless — no Webots.
#
# Launches, per robot i (0..4):
#   * vertex_node        the crate under test (real Vertex consensus, 5-peer mesh)
#   * arena_coordinator  the replicated FSM on the vertex_fleet API
#   * mock_pioneer       stand-in for the pioneer_explorer Webots controller
#
# Scenario: 8 sectors over the arena, all reachable. Robot 4's sensor streams
# die at t=6s and recover at t=26s: the fleet must fold its not-ok beacon into
# an unhealthy verdict (claims released and refused), reject the detection it
# reports while untrusted, readmit it on recovery, and still sweep every
# sector. Robot 1 reports a detection while healthy: accepted everywhere.
#
# Asserts (application layer):
#   * Full coverage: every sector explored, phase done on all 5 bots.
#   * Claim exclusivity: no snapshot shows one bot holding two sectors.
#   * Health verdicts: the unhealthy episode is observed on every bot, and
#     the final state has robot 4 readmitted (no vote/tally protocol needed).
#   * Detection gating: exactly the healthy robot's detection is accepted,
#     identically on every bot.
#   * The 5 vertex_nodes deliver byte-identical /vertex/event streams.
#
# Run in the Jazzy container:  docker compose run --rm sim simtest
# (or:  launch_test vertex_ros2/test/simulation_arena/arena_exploration.launch_test.py)
# Prerequisite: fixtures/peers5.json (fixtures/gen_peers5.sh).

import json
import os
import signal
import tempfile
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
PEERS_PATH = os.path.join(HERE, "fixtures", "peers5.json")
COORDINATOR = os.path.join(HERE, "nodes", "arena_coordinator.py")
MOCK_PIONEER = os.path.join(HERE, "nodes", "mock_pioneer.py")

N = 5
GRID = {"grid_nx": 4, "grid_ny": 2, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 10.0, "cell_h": 7.5}                      # 8 sectors
ALL_SECTORS = [f"S{k:02d}" for k in range(8)]
START_Y = [-12.0, -9.0, -6.0, -3.0, 0.0]                    # staging column x=-24

FAIL_AT, RECOVER_AT = 6.0, 45.0        # robot 4's sensor-stream outage window
HEALTHY_DET = {"id": 1, "label": "deer", "x": -14.6, "y": 9.6, "at_sec": 2.0}
UNHEALTHY_DET = {"id": 1, "label": "deer", "x": 11.8, "y": 5.0, "at_sec": 20.0}


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers5.sh first.")
    with open(PEERS_PATH) as f:
        return json.load(f)[:N]


def _secret_key_file(secret, tmpdir, name):
    # vertex.secret_key_path over vertex.secret_key_base58: the base58 form is a
    # normal ROS 2 parameter, so once declared the private key is readable by any
    # DDS participant via `ros2 param get`/`ros2 param dump`.
    # The file form keeps the parameter store holding only a path.
    path = os.path.join(tmpdir, f"{name}.key")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(secret)
    return path


@pytest.mark.launch_test
def generate_test_description():
    peers = _load_peers()
    keydir = tempfile.mkdtemp(prefix="vertex_arena_exploration_test_keys_")
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
                "vertex.secret_key_path": _secret_key_file(me["secret"], keydir, f"robot{i}"),
                "vertex.peers": peer_specs,
                "options.heartbeat_us": 50000,
            }],
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=COORDINATOR, name="arena_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "num_bots": N, **GRID,
                         "claim_interval_sec": 0.5,
                         "health_interval_sec": 1.0,
                         "stream_timeout_sec": 3.0,
                         "suspect_after_sec": 15.0}],
            output="screen",
        ))
        mock_params = {"start_x": -24.0, "start_y": START_Y[i], **GRID}
        if i == 4:
            mock_params.update(fail_streams_after_sec=FAIL_AT,
                               recover_streams_after_sec=RECOVER_AT,
                               detections_json=json.dumps([UNHEALTHY_DET]))
        if i == 1:
            mock_params.update(detections_json=json.dumps([HEALTHY_DET]))
        actions.append(launch_ros.actions.Node(
            executable=MOCK_PIONEER, name="mock_pioneer", namespace=ns,
            parameters=[mock_params],
            output="screen",
        ))

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestArenaExploration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("arena_exploration_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_fleet_sweeps_arena_with_health_gating(self):
        latest = {i: None for i in range(N)}
        saw_unhealthy4 = {i: False for i in range(N)}   # episode observed
        exclusivity_violations = []                     # one bot, two sectors
        events = {i: [] for i in range(N)}              # per-bot event stream

        def make_cb(i):
            def cb(msg):
                st = json.loads(msg.data)
                latest[i] = st
                if 4 in st.get("unhealthy", []):
                    saw_unhealthy4[i] = True
                holders = list(st.get("claimed", {}).values())
                if len(set(holders)) != len(holders):
                    exclusivity_violations.append(st["claimed"])
            return cb

        def make_ev_cb(i):
            def cb(msg):
                events[i].append((bytes(msg.hash),
                                  tuple(bytes(t.payload) for t in msg.transactions)))
            return cb

        subs = [self.node.create_subscription(
            String, f"/robot_{i}/mission_state", make_cb(i), 10) for i in range(N)]
        subs += [self.node.create_subscription(
            VertexEvent, f"/robot_{i}/vertex/event", make_ev_cb(i), 1000)
            for i in range(N)]

        # Run until, on every bot: coverage is done, robot 4 has been
        # readmitted (post-recovery beacon folded), and the accepted
        # detection has landed. Beacons keep folding after done, so
        # readmission needs no extra machinery.
        def settled():
            for i in range(N):
                st = latest[i]
                if not st or st.get("phase") != "done":
                    return False
                if st.get("unhealthy"):
                    return False
                if len(st.get("detections", [])) != 1:
                    return False
            return True

        deadline = time.time() + 150.0
        while time.time() < deadline and not settled():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        for i in range(N):
            st = latest[i]
            self.assertIsNotNone(st, f"robot_{i} never published mission_state")
            # full coverage: the whole arena swept
            self.assertEqual(sorted(st.get("explored", [])), ALL_SECTORS,
                             f"robot_{i} incomplete coverage: {st}")
            self.assertEqual(st.get("phase"), "done", f"robot_{i}: {st}")
            self.assertEqual(st.get("unreachable", []), [], f"robot_{i}: {st}")
            # the unhealthy episode was observed, and recovery readmitted bot 4
            self.assertTrue(saw_unhealthy4[i],
                            f"robot_{i} never saw robot 4 marked unhealthy")
            self.assertEqual(st.get("unhealthy", []), [],
                             f"robot_{i}: robot 4 was not readmitted: {st}")
            # detection gating: the healthy report accepted, the unhealthy
            # one rejected — identically everywhere
            self.assertEqual(st.get("detections"), [[1, 1, "deer"]],
                             f"robot_{i} detections diverge: {st}")

        # claim exclusivity (one sector per bot at any observed instant)
        self.assertEqual(exclusivity_violations, [],
                         f"a bot held two sectors: {exclusivity_violations[:3]}")

        # Vertex consensus itself: the 5 vertex_nodes must deliver a
        # BYTE-FOR-BYTE identical, identically-ordered /vertex/event stream.
        min_len = min(len(events[i]) for i in range(N))
        self.assertGreater(min_len, 0, "no consensus events were delivered")
        for idx in range(min_len):
            e0 = events[0][idx]
            for i in range(1, N):
                self.assertEqual(e0, events[i][idx],
                                 f"vertex event #{idx} differs: bot0={e0} "
                                 f"bot{i}={events[i][idx]}")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

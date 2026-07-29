#!/usr/bin/env python3
# Integration test for vertex_fleet, and the packaging acceptance test: a
# CONSUMER-shaped application (the ledger example, installed as a normal
# ament_python entry point) coordinating through three REAL vertex_node
# consensus peers.
#
# Three agents each contribute 5 entries by proposing `append` records. The
# assertions are the guarantees the library sells:
#   * every agent ends with the same 15 entries in the SAME order
#   * every agent contributed exactly its own 5 (idempotent re-proposals
#     collapse to one fold each)
#   * all three /vertex/event streams are byte-identical
#   * clean shutdown
#
# Run in the Jazzy container:  docker compose run --rm test
# Prerequisite: vertex_ros2/test/fixtures/peers.json (gen_test_keys.sh).

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
PEERS_PATH = os.path.join(HERE, "..", "..", "vertex_ros2", "test",
                          "fixtures", "peers.json")
N = 3
COUNT = 5           # entries each agent contributes


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run vertex_ros2/test/gen_test_keys.sh.")
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
    keydir = tempfile.mkdtemp(prefix="vertex_ledger_demo_keys_")
    actions = []
    for i, me in enumerate(peers):
        ns = f"/agent_{i}"
        others = [p for j, p in enumerate(peers) if j != i]
        actions.append(launch_ros.actions.Node(
            package="vertex_ros2", executable="vertex_node", name=f"vertex{i}",
            remappings=[(f"/vertex/{t}", f"{ns}/vertex/{t}")
                        for t in ("tx", "event", "sync_point", "status", "transition")]
                      + [("/vertex/lifecycle/state", f"{ns}/vertex/lifecycle/state")],
            parameters=[{
                "vertex.bind_address": me["addr"],
                "vertex.secret_key_path": _secret_key_file(me["secret"], keydir, f"agent{i}"),
                "vertex.peers": [f"{p['public']}@{p['addr']}" for p in others],
                "options.heartbeat_us": 50000,
            }],
            output="screen",
        ))
        # The consumer application, launched the way a consumer launches it:
        # as an installed executable of the vertex_fleet package.
        actions.append(launch_ros.actions.Node(
            package="vertex_fleet", executable="ledger_agent",
            name="ledger_agent", namespace=ns,
            parameters=[{"agent_id": i, "count": COUNT}],
            output="screen",
        ))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestLedgerDemo(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("ledger_demo_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_agents_agree_on_one_ledger(self):
        latest = {i: None for i in range(N)}
        events = {i: [] for i in range(N)}

        subs = [self.node.create_subscription(
            String, f"/agent_{i}/ledger",
            (lambda i: lambda m: latest.__setitem__(i, json.loads(m.data)))(i),
            10) for i in range(N)]
        subs += [self.node.create_subscription(
            VertexEvent, f"/agent_{i}/vertex/event",
            (lambda i: lambda m: events[i].append(
                (bytes(m.hash),
                 tuple(bytes(t.payload) for t in m.transactions))))(i),
            1000) for i in range(N)]

        deadline = time.time() + 90.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(latest[i] and latest[i]["done"]
                   and len(latest[i]["entries"]) == N * COUNT
                   for i in range(N)):
                break

        # Completion + agreement: same entries, same ORDER, on every agent.
        for i in range(N):
            st = latest[i]
            self.assertIsNotNone(st, f"agent_{i} never published its ledger")
            self.assertEqual(len(st["entries"]), N * COUNT, f"agent_{i}: {st}")
            self.assertEqual(st["entries"], latest[0]["entries"],
                             f"agent_{i} ordered ledger differs from agent_0")
            # idempotency: exactly COUNT entries per contributor, seqs 0..COUNT-1
            for a in range(N):
                seqs = sorted(s for agent, s in st["entries"] if agent == a)
                self.assertEqual(seqs, list(range(COUNT)),
                                 f"agent_{i} sees bad seqs for contributor {a}")

        # The transport-level guarantee underneath: byte-identical streams.
        min_len = min(len(events[i]) for i in range(N))
        self.assertGreater(min_len, 0, "no consensus events observed")
        for idx in range(min_len):
            for i in range(1, N):
                self.assertEqual(events[0][idx], events[i][idx],
                                 f"event #{idx} differs agent_0 vs agent_{i}")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

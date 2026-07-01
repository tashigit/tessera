#!/usr/bin/env python3
# 10-minute soak test for vertex_ros2 (TAS-76 / design §7: "No unbounded memory
# growth under 10-minute load").
#
# Launches a single `vertex_node`, activates it, publishes transactions at a
# steady rate for SOAK_SECONDS, sampling the process RSS once per second. Fails
# if RSS grows by more than MAX_RSS_GROWTH_MB after the first minute (the
# warm-up window).
#
# Long-running; not part of the default `colcon test`. Run explicitly:
#     SOAK_SECONDS=600 launch_test src/vertex_ros2/test/soak.launch_test.py
#
# Requires ROS 2 (Jazzy) + psutil. Reuses test/fixtures/peers.json (first entry).

import json
import os
import signal
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import psutil
import pytest
import rclpy
from rclpy.node import Node

from vertex_ros2_msgs.msg import VertexTransaction
from vertex_ros2_msgs.srv import VertexTransition

HERE = os.path.dirname(__file__)
PEERS_PATH = os.path.join(HERE, "fixtures", "peers.json")

SOAK_SECONDS = int(os.environ.get("SOAK_SECONDS", "600"))
WARMUP_SECONDS = 60
MAX_RSS_GROWTH_MB = 50
PUBLISH_HZ = 50


@pytest.mark.launch_test
def generate_test_description():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run test/gen_test_keys.sh first.")
    with open(PEERS_PATH) as f:
        me = json.load(f)[0]
    node = launch_ros.actions.Node(
        package="vertex_ros2",
        executable="vertex_node",
        name="vertex_soak",
        parameters=[
            {
                "vertex.bind_address": me["addr"],
                "vertex.secret_key_base58": me["secret"],
                # vertex.peers omitted: launch_ros can't type an empty-list param
                # (it becomes an empty tuple). The node defaults it to an empty
                # array, which is exactly the solo session we want here.
                "options.heartbeat_us": 50000,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]),
        {"vertex_proc": node},
    )


class TestSoak(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("vertex_soak_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _transition(self, verb, timeout=10.0):
        cli = self.node.create_client(VertexTransition, "/vertex/transition")
        self.assertTrue(cli.wait_for_service(timeout_sec=timeout))
        req = VertexTransition.Request()
        req.transition = verb
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        self.assertIsNotNone(fut.result())
        return fut.result()

    def test_no_unbounded_rss_growth(self, proc_info, vertex_proc):
        self.assertTrue(self._transition("configure").success)
        self.assertTrue(self._transition("activate").success)

        pid = proc_info[vertex_proc].pid
        proc = psutil.Process(pid)
        pub = self.node.create_publisher(VertexTransaction, "/vertex/tx", 10)

        baseline_mb = None
        period = 1.0 / PUBLISH_HZ
        start = time.time()
        seq = 0
        while time.time() - start < SOAK_SECONDS:
            msg = VertexTransaction()
            msg.payload = list(f"soak-{seq:09}".encode())
            pub.publish(msg)
            seq += 1
            rclpy.spin_once(self.node, timeout_sec=period)

            elapsed = time.time() - start
            if baseline_mb is None and elapsed >= WARMUP_SECONDS:
                baseline_mb = proc.memory_info().rss / 1e6
            if baseline_mb is not None:
                cur_mb = proc.memory_info().rss / 1e6
                self.assertLess(
                    cur_mb - baseline_mb,
                    MAX_RSS_GROWTH_MB,
                    f"RSS grew {cur_mb - baseline_mb:.1f} MB after warm-up "
                    f"(baseline {baseline_mb:.1f} MB, {seq} txs sent)",
                )
        self.assertIsNotNone(baseline_mb, "soak shorter than warm-up window")


@launch_testing.post_shutdown_test()
class TestSoakShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # The harness stops the node with SIGINT (exit code -2 in Python's
        # negative-signal convention), which is a clean teardown, not a crash.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

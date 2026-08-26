#!/usr/bin/env python3
# Phase B of the air/ground simulation: a BYZANTINE PEER, headless.
#
# Vertex tolerates f = 1 of n = 4 at the consensus layer, but that guarantee is
# narrower than people usually read it. Consensus makes every peer agree on
# what was SAID and in what order. It does not make what was said TRUE. A drone
# with a working engine and valid signatures can propose whatever it likes, and
# every honest peer will faithfully fold the lie.
#
# So the application needs its own rule, and this scenario is the demonstration
# that it works. `drone_1` runs with --conduct phantom-hazards: it reports a
# crater in every cell of every block it surveys. All fabricated.
#
# The corroboration rule says a hazard needs two DISTINCT witnesses before it
# condemns anything. Nobody ever agrees with drone_1, because the craters are
# not there, so:
#
#   * every phantom stays provisional: deferred, never condemned
#   * the bots sweep the whole map anyway, so a liar cannot deny service
#   * the honest drone's survey work still lands normally
#
# The mirror case (a drone reporting a real pit as clear) is covered by the
# main airground.launch_test: there the bot that drives into the hole is itself
# the second witness. Between the two, a single lying drone can neither invent
# an obstacle nor hide one.
#
# Run in the Jazzy container:  docker compose run --rm sim simtest
# Prerequisites: fixtures/peers_airground.json and a built air_agent.

import json
import os
import shutil
import signal
import tempfile
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
PEERS_PATH = os.path.join(HERE, "fixtures", "peers_airground.json")
COORDINATOR = os.path.join(HERE, "nodes", "ground_coordinator.py")
MOCK_PIONEER = os.path.join(HERE, "nodes", "mock_pioneer.py")
MOCK_AIRFRAME = os.path.join(HERE, "nodes", "mock_airframe.py")
AIR_AGENT = os.path.join(HERE, "air_agent", "target", "debug", "air_agent")
LOGDIR = os.path.join(HERE, "logs", "lying_drone")

GRID = {"grid_nx": 4, "grid_ny": 3, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 10.0, "cell_h": 10.0, "block_w": 2, "block_h": 1}
ALL_SECTORS = [f"S{k:02d}" for k in range(12)]

BOTS = ["bot_0", "bot_1"]
DRONES = ["drone_0", "drone_1"]
AGENTS = BOTS + DRONES
LIAR = "drone_1"

BOT_START = {"bot_0": (-24.0, -12.0), "bot_1": (-24.0, -6.0)}
DRONE_START = {"drone_0": "-20,-15", "drone_1": "16,9"}
LINK_PORT = {"drone_0": 48635, "drone_1": 48636}


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers_airground.sh first.")
    with open(PEERS_PATH) as f:
        return {p["name"]: p for p in json.load(f)}


def _secret_key_file(secret, tmpdir, name):
    path = os.path.join(tmpdir, f"{name}.key")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(secret)
    return path


def _peer_specs(peers, me):
    return [f"{p['public']}@{p['addr']}" for n, p in peers.items() if n != me]


@pytest.mark.launch_test
def generate_test_description():
    peers = _load_peers()
    if not os.path.exists(AIR_AGENT):
        pytest.skip(f"{AIR_AGENT} missing — run (cd air_agent && cargo build) first.")

    shutil.rmtree(LOGDIR, ignore_errors=True)
    os.makedirs(LOGDIR, exist_ok=True)
    keydir = tempfile.mkdtemp(prefix="vertex_lying_drone_test_keys_")
    actions = []

    for me in BOTS:
        ns = f"/{me}"
        p = peers[me]
        actions.append(launch_ros.actions.Node(
            package="vertex_ros2", executable="vertex_node", name=f"vertex_{me}",
            remappings=[(f"/vertex/{t}", f"{ns}/vertex/{t}")
                        for t in ("tx", "event", "sync_point", "status", "transition")]
                      + [("/vertex/lifecycle/state", f"{ns}/vertex/lifecycle/state")],
            parameters=[{
                "vertex.bind_address": p["addr"],
                "vertex.secret_key_path": _secret_key_file(p["secret"], keydir, me),
                "vertex.peers": _peer_specs(peers, me),
                "options.heartbeat_us": 50000,
            }],
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=COORDINATOR, name="ground_coordinator", namespace=ns,
            parameters=[{"agent_id": me, "agents": AGENTS, **GRID,
                         "claim_interval_sec": 0.5,
                         "cover_radius": 2.0,
                         "suspect_after_sec": 30.0,
                         "stall_sec": 8.0}],
            output="screen",
        ))
        sx, sy = BOT_START[me]
        actions.append(launch_ros.actions.Node(
            executable=MOCK_PIONEER, name="mock_pioneer", namespace=ns,
            # No craters anywhere: every sector really is reachable, so every
            # hazard drone_1 reports is a fabrication.
            parameters=[{"agent_id": me, "start_x": sx, "start_y": sy, **GRID,
                         "pits_json": "[]"}],
            output="screen",
        ))

    for me in DRONES:
        p = peers[me]
        cmd = [AIR_AGENT,
               "--id", me,
               "--bind", p["addr"],
               "--key", p["secret"],
               "--link", f"127.0.0.1:{LINK_PORT[me]}",
               "--log", os.path.join(LOGDIR, f"{me}.log")]
        if me == LIAR:
            cmd += ["--conduct", "phantom-hazards"]
        for spec in _peer_specs(peers, me):
            cmd += ["--peer", spec]
        actions.append(launch.actions.ExecuteProcess(
            cmd=cmd, name=f"air_agent_{me}", output="screen"))
        actions.append(launch.actions.ExecuteProcess(
            cmd=["python3", MOCK_AIRFRAME,
                 "--link", f"127.0.0.1:{LINK_PORT[me]}",
                 # `--start=` not `--start `: on Python 3.12 argparse rejects a
                 # value that looks like an option, and "-20,-15" does.
                 f"--start={DRONE_START[me]}"],
            name=f"mock_airframe_{me}", output="screen"))

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestLyingDrone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("lying_drone_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_a_liar_cannot_deny_service(self):
        latest = {b: None for b in BOTS}
        condemned_ever = []        # any sector condemned at any instant

        def make_cb(b):
            def cb(msg):
                st = json.loads(msg.data)
                latest[b] = st
                for s in st.get("unreachable", []):
                    condemned_ever.append((b, s))
            return cb

        subs = [self.node.create_subscription(
            String, f"/{b}/mission_state", make_cb(b), 10) for b in BOTS]

        def settled():
            return all(latest[b] and latest[b].get("phase") == "done" for b in BOTS)

        deadline = time.time() + 240.0
        while time.time() < deadline and not settled():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        for b in BOTS:
            st = latest[b]
            self.assertIsNotNone(st, f"{b} never published mission_state")

            # The whole point: one Byzantine peer shouting about craters that
            # do not exist must not cost the fleet a single sector.
            self.assertEqual(sorted(st.get("explored", [])), ALL_SECTORS,
                             f"{b}: the liar denied service, coverage is short: {st}")
            self.assertEqual(st.get("unreachable", []), [],
                             f"{b}: a fabricated hazard condemned real ground: {st}")
            self.assertEqual(st.get("confirmed_hazards", []), [],
                             f"{b}: an uncorroborated hazard was confirmed: {st}")
            self.assertEqual(st.get("phase"), "done", f"{b} did not finish: {st}")

            # The lies are all present in the log and all still provisional:
            # folded faithfully, believed by nobody.
            reports = st.get("hazard_reports", {})
            self.assertTrue(reports, f"{b}: the liar's hazards never reached the log")
            for cell, witnesses in reports.items():
                self.assertEqual(
                    witnesses, [LIAR],
                    f"{b}: {cell} has witnesses other than the liar: {witnesses}")

            # And the honest drone's work was unaffected.
            self.assertEqual(sorted(st.get("surveyed_blocks", [])),
                             [f"B{k:02d}" for k in range(6)],
                             f"{b}: survey did not complete: {st}")

        self.assertEqual(condemned_ever, [],
                         f"ground was condemned on one witness: {condemned_ever[:3]}")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

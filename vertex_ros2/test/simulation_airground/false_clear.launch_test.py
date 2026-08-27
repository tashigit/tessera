#!/usr/bin/env python3
# Phase B, the other half: a BYZANTINE PEER THAT HIDES A HAZARD, headless.
#
# `lying_drone.launch_test` covers a drone inventing craters that are not
# there, and shows a liar cannot deny service. This covers the mirror case,
# which is the more dangerous one: a drone that flies over a real crater and
# reports the block CLEAR.
#
# Inventing a hazard is cheap to defend against, because nobody corroborates a
# fiction. Hiding one is not, because the fleet has no reason to doubt a clear
# report and the ground tier is blind to exactly this kind of obstacle. So the
# defence cannot be "detect the lie". It has to be that the truth arrives
# anyway, from a different direction:
#
#   * `drone_1` runs with --conduct false-clear. It surveys its blocks and
#     reports every one of them clean, including the block holding S05, where
#     its ranger really did see a crater.
#   * No hazard is ever recorded for S05, so the bots treat it as ordinary
#     ground and one of them claims it.
#   * That bot drives in, cannot cross, and reports it. The crater enters the
#     shared map through the ground tier instead of the air tier.
#
# The cost of the lie is real and is worth being precise about: one robot's
# time, and a sector that took a physical attempt to condemn instead of a
# 12 m flyover. What the lie cannot do is corrupt the shared map or stall the
# mission. Every honest peer still converges on the same state.
#
# Asserts:
#   * The liar really did suppress the hazard: no drone ever witnesses S05.
#   * The truth lands anyway: S05 is witnessed by the ground tier and
#     condemned, or covered, but never silently left as unexamined ground.
#   * Coverage completes and the fleet agrees.
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
LOGDIR = os.path.join(HERE, "logs", "false_clear")

GRID = {"grid_nx": 4, "grid_ny": 3, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 10.0, "cell_h": 10.0, "block_w": 2, "block_h": 1}
ALL_SECTORS = [f"S{k:02d}" for k in range(12)]
PIT = "S05"                    # a real crater, in a block the liar surveys
REACHABLE = [s for s in ALL_SECTORS if s != PIT]

BOTS = ["bot_0", "bot_1"]
DRONES = ["drone_0", "drone_1"]
AGENTS = BOTS + DRONES
LIAR = "drone_1"

BOT_START = {"bot_0": (-24.0, -12.0), "bot_1": (-24.0, -6.0)}
# The liar starts nearest the block containing S05 (B02 = S04, S05) so it is
# the one that surveys it and therefore the one with something to hide.
DRONE_START = {"drone_0": "18,12", "drone_1": "-14,0"}
LINK_PORT = {"drone_0": 48637, "drone_1": 48638}


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
    keydir = tempfile.mkdtemp(prefix="vertex_false_clear_test_keys_")
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
                         "stall_sec": 8.0,
                         "max_attempts": 1}],
            output="screen",
        ))
        sx, sy = BOT_START[me]
        actions.append(launch_ros.actions.Node(
            executable=MOCK_PIONEER, name="mock_pioneer", namespace=ns,
            # The crater is REALLY THERE. That is the whole point: the world
            # has a hazard the air tier chose not to report.
            parameters=[{"agent_id": me, "start_x": sx, "start_y": sy, **GRID,
                         "pits_json": json.dumps([PIT]),
                         "pit_rim": 4.0}],
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
            cmd += ["--conduct", "false-clear"]
        for spec in _peer_specs(peers, me):
            cmd += ["--peer", spec]
        actions.append(launch.actions.ExecuteProcess(
            cmd=cmd, name=f"air_agent_{me}", output="screen"))
        actions.append(launch.actions.ExecuteProcess(
            cmd=["python3", MOCK_AIRFRAME,
                 "--link", f"127.0.0.1:{LINK_PORT[me]}",
                 f"--start={DRONE_START[me]}",
                 # Both airframes fly over the real crater and both rangers
                 # see it. Only the liar's agent throws the reading away.
                 "--pits", PIT],
            name=f"mock_airframe_{me}", output="screen"))

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


class TestFalseClear(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("false_clear_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_a_hidden_hazard_still_reaches_the_map(self):
        latest = {b: None for b in BOTS}
        air_witnessed_pit = []      # any drone ever witnessing the real crater

        def make_cb(b):
            def cb(msg):
                st = json.loads(msg.data)
                latest[b] = st
                for w in (st.get("hazard_reports", {}) or {}).get(PIT, []):
                    if w in DRONES:
                        air_witnessed_pit.append(w)
            return cb

        subs = [self.node.create_subscription(
            String, f"/{b}/mission_state", make_cb(b), 10) for b in BOTS]

        def settled():
            for b in BOTS:
                st = latest[b]
                if not st:
                    return False
                # every sector either covered or condemned
                done = set(st.get("explored", [])) | set(st.get("unreachable", []))
                if set(ALL_SECTORS) - done:
                    return False
            return True

        deadline = time.time() + 240.0
        while time.time() < deadline and not settled():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        for b in BOTS:
            st = latest[b]
            self.assertIsNotNone(st, f"{b} never published mission_state")

            # The blocks were all surveyed, the liar's included. It did the
            # work, it just did not report what it found.
            self.assertEqual(sorted(st.get("surveyed_blocks", [])),
                             [f"B{k:02d}" for k in range(6)],
                             f"{b}: survey did not complete: {st}")

            # The lie landed: no drone ever flagged the real crater, even
            # though both airframes flew over it and both rangers saw it.
            witnesses = (st.get("hazard_reports", {}) or {}).get(PIT, [])
            self.assertFalse([w for w in witnesses if w in DRONES],
                             f"{b}: a drone reported {PIT}, so the false-clear "
                             f"conduct did not actually suppress it: {witnesses}")

            # And the truth arrived anyway, from the ground. The crater is
            # either condemned by the bots that could not cross it, or at the
            # very least witnessed by one of them. What must NOT happen is the
            # crater passing silently as ordinary swept ground.
            self.assertTrue(
                [w for w in witnesses if w in BOTS],
                f"{b}: the hidden crater at {PIT} was never witnessed by the "
                f"ground tier either, so the lie went entirely unchallenged: {st}")

            # No sector left unexamined.
            done = set(st.get("explored", [])) | set(st.get("unreachable", []))
            self.assertEqual(sorted(done), ALL_SECTORS,
                             f"{b}: sectors left unresolved: {sorted(set(ALL_SECTORS) - done)}")

            # Everything the bots could physically reach, they swept.
            for s in REACHABLE:
                self.assertIn(s, st.get("explored", []),
                              f"{b}: {s} is reachable but was not swept: {st}")

        # Both bots agree on the crater's fate. The lie cost a robot's time,
        # not the fleet's agreement.
        verdicts = [(sorted(latest[b].get("unreachable", [])),
                     sorted(latest[b].get("confirmed_hazards", []))) for b in BOTS]
        self.assertEqual(len(set(map(str, verdicts))), 1,
                         f"the bots disagree about the hidden crater: {verdicts}")

        self.assertFalse(air_witnessed_pit,
                         f"a drone witnessed {PIT} at some point: {air_witnessed_pit}")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

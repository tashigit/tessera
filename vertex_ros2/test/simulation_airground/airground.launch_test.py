#!/usr/bin/env python3
# Automated integration test for the AIR/GROUND SURVEY-AND-SWEEP scenario
# (the third simulation; see README.md), headless — no Webots.
#
# This is the one that proves the point. The committee is genuinely mixed:
#
#   bot_0, bot_1     vertex_node + ground_coordinator + mock_pioneer
#                    (tessera: ROS 2 nodes, the Python fold)
#   drone_0, drone_1 air_agent + mock_airframe
#                    (native Rust binaries linking tashi-vertex directly;
#                     no rclpy, no DDS, no message packages, no rosbridge)
#
# Four peers, one hashgraph, one ordered log. The unit tests already show the
# two folds agree on the same records, and fixtures/conformance.json pins them
# together. What only this test can show is that the WIRE interoperates: that
# a record a Rust drone puts on the mesh is decoded and folded correctly by a
# Python bot, and the reverse.
#
# Scenario: 12 sectors in 6 survey blocks. S05 is a real crater.
#   * Nothing on the ground is claimable until a drone surveys it, so the
#     bots idle at first. That is the cross-tier dependency, not a stall.
#   * Whichever drone surveys B02 sees S05 read long on its downward ranger
#     and reports `hazard`. One witness, so the sector is deferred, not
#     condemned: the bots sweep everything else first.
#   * A bot eventually claims S05, drives to the crater rim, cannot get to
#     the centre, and reports `abandon` + `corroborate`. Two distinct
#     witnesses, so the sector is condemned and the mission can finish.
#
# Asserts:
#   * Cross-tier gating: no sector is ever claimed before it was surveyed.
#   * Coverage: the 11 reachable sectors explored, S05 unreachable, phase done.
#   * Corroboration: S05 confirmed by two DISTINCT agents, one air one ground.
#   * Mixed-fleet agreement: the drones' folded state (read from their own
#     journals, written by the Rust fold) equals the bots' (published on
#     mission_state by the Python fold).
#   * The 2 vertex_nodes deliver byte-identical /vertex/event streams.
#
# Run in the Jazzy container:  docker compose run --rm sim simtest
# Prerequisites: fixtures/peers_airground.json (fixtures/gen_peers_airground.sh)
#                and a built air_agent (cd air_agent && cargo build).

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
from vertex_ros2_msgs.msg import VertexEvent

HERE = os.path.dirname(os.path.abspath(__file__))
PEERS_PATH = os.path.join(HERE, "fixtures", "peers_airground.json")
COORDINATOR = os.path.join(HERE, "nodes", "ground_coordinator.py")
MOCK_PIONEER = os.path.join(HERE, "nodes", "mock_pioneer.py")
MOCK_AIRFRAME = os.path.join(HERE, "nodes", "mock_airframe.py")
AIR_AGENT = os.path.join(HERE, "air_agent", "target", "debug", "air_agent")
LOGDIR = os.path.join(HERE, "logs", "launch_test")

GRID = {"grid_nx": 4, "grid_ny": 3, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 10.0, "cell_h": 10.0, "block_w": 2, "block_h": 1}
ALL_SECTORS = [f"S{k:02d}" for k in range(12)]
PIT = "S05"                       # the one real crater in the world
REACHABLE = [s for s in ALL_SECTORS if s != PIT]

BOTS = ["bot_0", "bot_1"]
DRONES = ["drone_0", "drone_1"]
AGENTS = BOTS + DRONES

# Ground staging, west side, clear of the crater.
BOT_START = {"bot_0": (-24.0, -12.0), "bot_1": (-24.0, -6.0)}
# Air staging, spread so the two drones do not both race for B00.
DRONE_START = {"drone_0": "-20,-15", "drone_1": "16,9"}
LINK_PORT = {"drone_0": 48633, "drone_1": 48634}


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers_airground.sh first.")
    with open(PEERS_PATH) as f:
        peers = json.load(f)
    return {p["name"]: p for p in peers}


def _secret_key_file(secret, tmpdir, name):
    # vertex.secret_key_path over vertex.secret_key_base58: the base58 form is a
    # normal ROS 2 parameter, so once declared the private key is readable by any
    # DDS participant via `ros2 param get`/`ros2 param dump`.
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

    # Fresh journals: the drones append, and a previous run's state would be
    # read back as this run's result.
    shutil.rmtree(LOGDIR, ignore_errors=True)
    os.makedirs(LOGDIR, exist_ok=True)

    keydir = tempfile.mkdtemp(prefix="vertex_airground_test_keys_")
    actions = []

    # ---- ground tier: tessera ----
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
                         "health_interval_sec": 1.0,
                         "stream_timeout_sec": 3.0,
                         "suspect_after_sec": 30.0,
                         # short enough that the crater is corroborated while
                         # the test is still running
                         "stall_sec": 8.0}],
            output="screen",
        ))
        sx, sy = BOT_START[me]
        actions.append(launch_ros.actions.Node(
            executable=MOCK_PIONEER, name="mock_pioneer", namespace=ns,
            parameters=[{"agent_id": me, "start_x": sx, "start_y": sy, **GRID,
                         "pits_json": json.dumps([PIT]),
                         # must exceed cover_radius, or the robot would credit
                         # the sector despite never reaching the centre
                         "pit_rim": 4.0}],
            output="screen",
        ))

    # ---- air tier: no ROS at all ----
    for me in DRONES:
        p = peers[me]
        cmd = [AIR_AGENT,
               "--id", me,
               "--bind", p["addr"],
               "--key", p["secret"],
               "--link", f"127.0.0.1:{LINK_PORT[me]}",
               "--log", os.path.join(LOGDIR, f"{me}.log")]
        for spec in _peer_specs(peers, me):
            cmd += ["--peer", spec]
        actions.append(launch.actions.ExecuteProcess(
            cmd=cmd, name=f"air_agent_{me}", output="screen"))
        actions.append(launch.actions.ExecuteProcess(
            cmd=["python3", MOCK_AIRFRAME,
                 "--link", f"127.0.0.1:{LINK_PORT[me]}",
                 # `--start=` not `--start `: on Python 3.12 argparse rejects a
                 # value that looks like an option, and "-20,-15" does.
                 f"--start={DRONE_START[me]}",
                 "--pits", PIT],
            name=f"mock_airframe_{me}", output="screen"))

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {}


def _drone_state(name):
    """The last folded state a drone journalled, straight from the Rust fold."""
    path = os.path.join(LOGDIR, f"{name}.log")
    if not os.path.exists(path):
        return None
    last = None
    with open(path) as f:
        for line in f:
            if line.startswith("STATE "):
                last = line[len("STATE "):].strip()
    try:
        return json.loads(last) if last else None
    except ValueError:
        return None


class TestAirGround(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("airground_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_mixed_fleet_surveys_then_sweeps(self):
        latest = {b: None for b in BOTS}
        events = {b: [] for b in BOTS}
        gate_violations = []       # a sector claimed before it was surveyed
        exclusivity_violations = []

        def make_cb(b):
            def cb(msg):
                st = json.loads(msg.data)
                latest[b] = st
                surveyed = set(st.get("surveyed", []))
                for sector in st.get("claimed", {}):
                    if sector not in surveyed:
                        gate_violations.append((b, sector))
                holders = list(st.get("claimed", {}).values())
                if len(set(holders)) != len(holders):
                    exclusivity_violations.append(st["claimed"])
            return cb

        def make_ev_cb(b):
            def cb(msg):
                events[b].append((bytes(msg.hash),
                                  tuple(bytes(t.payload) for t in msg.transactions)))
            return cb

        subs = [self.node.create_subscription(
            String, f"/{b}/mission_state", make_cb(b), 10) for b in BOTS]
        subs += [self.node.create_subscription(
            VertexEvent, f"/{b}/vertex/event", make_ev_cb(b), 1000) for b in BOTS]

        def settled():
            for b in BOTS:
                st = latest[b]
                if not st or st.get("phase") != "done":
                    return False
            return all(_drone_state(d) is not None for d in DRONES)

        deadline = time.time() + 240.0
        while time.time() < deadline and not settled():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # ---- the ground tier's view ----
        for b in BOTS:
            st = latest[b]
            self.assertIsNotNone(st, f"{b} never published mission_state")
            self.assertEqual(st.get("phase"), "done", f"{b} did not finish: {st}")
            self.assertEqual(sorted(st.get("surveyed_blocks", [])),
                             [f"B{k:02d}" for k in range(6)],
                             f"{b}: the air tier did not survey every block: {st}")
            self.assertEqual(sorted(st.get("explored", [])), REACHABLE,
                             f"{b}: incomplete coverage: {st}")
            self.assertEqual(st.get("unreachable", []), [PIT],
                             f"{b}: the crater was not condemned: {st}")

            # Corroboration: two DISTINCT witnesses, and crucially one from
            # each tier. A drone saw the hole from above; a bot could not get
            # through it from the ground.
            witnesses = st.get("hazard_reports", {}).get(PIT, [])
            self.assertEqual(len(set(witnesses)), 2,
                             f"{b}: {PIT} needs two distinct witnesses, got {witnesses}")
            self.assertTrue(any(w in DRONES for w in witnesses),
                            f"{b}: no drone witnessed {PIT}: {witnesses}")
            self.assertTrue(any(w in BOTS for w in witnesses),
                            f"{b}: no bot corroborated {PIT}: {witnesses}")
            self.assertEqual(st.get("confirmed_hazards", []), [PIT], f"{b}: {st}")

        # ---- the cross-tier gate ----
        self.assertEqual(gate_violations, [],
                         f"a sector was claimed before the air tier surveyed it: "
                         f"{gate_violations[:3]}")
        self.assertEqual(exclusivity_violations, [],
                         f"an agent held two sectors: {exclusivity_violations[:3]}")

        # ---- THE headline: the Rust fold agrees with the Python fold ----
        # Not merely "both implementations pass the same fixture" (the unit
        # tests cover that) but "records crossed the wire between a Rust peer
        # and a Python peer and both derived the same state from them".
        ground = latest[BOTS[0]]
        for d in DRONES:
            air = _drone_state(d)
            self.assertIsNotNone(air, f"{d} journalled no folded state")
            for field in ("surveyed_blocks", "surveyed", "explored",
                          "unreachable", "confirmed_hazards", "phase",
                          "explored_by", "hazard_reports"):
                self.assertEqual(
                    air.get(field), ground.get(field),
                    f"{d} (Rust fold) and {BOTS[0]} (Python fold) disagree on "
                    f"{field}:\n  air={air.get(field)}\n  ground={ground.get(field)}")

        # ---- Vertex itself: identical ordered streams ----
        min_len = min(len(events[b]) for b in BOTS)
        self.assertGreater(min_len, 0, "no consensus events were delivered")
        for idx in range(min_len):
            e0 = events[BOTS[0]][idx]
            for b in BOTS[1:]:
                self.assertEqual(e0, events[b][idx],
                                 f"vertex event #{idx} differs between bots")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

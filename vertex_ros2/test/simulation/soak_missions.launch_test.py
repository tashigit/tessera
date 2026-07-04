#!/usr/bin/env python3
# Mission soak for the route-exploration scenario (N5): randomized
# back-to-back missions against real 4-node Vertex consensus, headless.
#
# Each mission uses a random blocked-route set (0 to 3 routes blocked, always
# at least one open so every mission can terminate). When all four robots
# report done, the test re-randomizes the mocks' blocked sets and publishes a
# fresh epoch on /reset; the coordinators relay it into consensus and the
# fleet wipes and re-explores. Repeats until SOAK_SECONDS elapse.
#
# Asserts, per mission:
#   * termination: every robot reaches phase done within the mission deadline;
#   * agreement: one winner on all robots, and it is not a blocked route;
#   * epoch: every robot follows the reset into the new epoch.
# And across the run:
#   * RSS of each of the four vertex_node processes grows less than
#     MAX_RSS_GROWTH_MB after the first-mission warm-up;
#   * all four /vertex/event streams end byte-identical;
#   * clean shutdown.
#
# Long-running; not part of the default suite. Run explicitly:
#     SOAK_SECONDS=300 docker compose run --rm sim simsoak
# Prerequisite: fixtures/peers4.json (fixtures/gen_peers4.sh).

import json
import os
import random
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
from std_msgs.msg import Int32, String
from vertex_ros2_msgs.msg import VertexEvent

HERE = os.path.dirname(__file__)
PEERS_PATH = os.path.join(HERE, "fixtures", "peers4.json")
COORDINATOR = os.path.join(HERE, "nodes", "mission_coordinator.py")
MOCK_ROBOT = os.path.join(HERE, "nodes", "mock_robot.py")
ROUTES = ["R1", "R2", "R3", "R4"]
HOME_LANE_Y = [2.25, 0.75, -0.75, -2.25]

SOAK_SECONDS = int(os.environ.get("SOAK_SECONDS", "120"))
MISSION_DEADLINE = 90.0
MAX_RSS_GROWTH_MB = 50
SEED = int(os.environ.get("SOAK_SEED", str(int(time.time()))))


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers4.sh first.")
    with open(PEERS_PATH) as f:
        return json.load(f)[:4]


@pytest.mark.launch_test
def generate_test_description():
    peers = _load_peers()
    actions = []
    vertex_actions = []
    for i, me in enumerate(peers):
        ns = f"/robot_{i}"
        others = [p for j, p in enumerate(peers) if j != i]
        peer_specs = [f"{p['public']}@{p['addr']}" for p in others]
        vertex = launch_ros.actions.Node(
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
        )
        vertex_actions.append(vertex)
        actions.append(vertex)
        actions.append(launch_ros.actions.Node(
            executable=COORDINATOR, name="mission_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "routes": ROUTES, "goal_x": 3.6,
                         "num_bots": 4, "claim_interval_sec": 0.5}],
            output="screen",
        ))
        actions.append(launch_ros.actions.Node(
            executable=MOCK_ROBOT, name="mock_robot", namespace=ns,
            parameters=[{"home_lane_y": HOME_LANE_Y[i],
                         "blocked_routes": ["R1"]}],   # mission 0's world
            output="screen",
        ))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {"vertex_actions": vertex_actions}


class TestSoakMissions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("soak_missions_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_randomized_missions_no_leak(self, proc_info, vertex_actions):
        rng = random.Random(SEED)
        print(f"[soak] SOAK_SECONDS={SOAK_SECONDS} SOAK_SEED={SEED}")

        latest = {i: None for i in range(4)}
        events = {i: [] for i in range(4)}
        subs = [self.node.create_subscription(
            String, f"/robot_{i}/mission_state",
            (lambda i: lambda m: latest.__setitem__(i, json.loads(m.data)))(i),
            10) for i in range(4)]
        subs += [self.node.create_subscription(
            VertexEvent, f"/robot_{i}/vertex/event",
            (lambda i: lambda m: events[i].append(
                (bytes(m.hash),
                 tuple(bytes(t.payload) for t in m.transactions))))(i),
            1000) for i in range(4)]
        reset_pub = self.node.create_publisher(Int32, "/reset", 10)
        blocked_pubs = [self.node.create_publisher(
            String, f"/robot_{i}/blocked_routes", 10) for i in range(4)]

        procs = [psutil.Process(proc_info[a].pid) for a in vertex_actions]
        baseline_mb = None

        start = time.time()
        epoch, mission, blocked = 0, 0, ["R1"]
        while True:
            # --- run one mission to completion ---
            deadline = time.time() + MISSION_DEADLINE
            done = False
            while time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if all(latest[i] and latest[i].get("phase") == "done"
                       and latest[i].get("epoch") == epoch for i in range(4)):
                    done = True
                    break
            self.assertTrue(
                done, f"mission {mission} (epoch {epoch}, blocked {blocked}) "
                      f"did not terminate: {latest}")

            winners = {latest[i].get("winner_route") for i in range(4)}
            self.assertEqual(len(winners), 1,
                             f"mission {mission}: winner disagreement {winners}")
            winner = winners.pop()
            self.assertIsNotNone(winner, f"mission {mission}: no winner")
            self.assertNotIn(winner, blocked,
                             f"mission {mission}: blocked route won: {winner}")
            print(f"[soak] mission {mission} done: blocked={blocked} "
                  f"winner={winner} t={time.time() - start:.0f}s")

            # --- RSS bound (baseline after the first full mission) ---
            rss = [p.memory_info().rss / 1e6 for p in procs]
            if baseline_mb is None:
                baseline_mb = rss
            for k, (cur, base) in enumerate(zip(rss, baseline_mb)):
                self.assertLess(
                    cur - base, MAX_RSS_GROWTH_MB,
                    f"vertex{k} RSS grew {cur - base:.1f} MB after "
                    f"{mission + 1} missions (baseline {base:.1f} MB)")

            if time.time() - start >= SOAK_SECONDS:
                break

            # --- next mission: random world, fresh consensus epoch ---
            mission += 1
            epoch += 1
            blocked = rng.sample(ROUTES, rng.randint(0, 3))
            for pub in blocked_pubs:
                pub.publish(String(data=json.dumps(blocked)))
            settle = time.time() + 0.5      # mocks must see the new world
            while time.time() < settle:     # before the fleet restarts
                rclpy.spin_once(self.node, timeout_sec=0.05)
            reset_pub.publish(Int32(data=epoch))

        self.assertGreaterEqual(mission + 1, 2,
                                "soak too short to run at least two missions")

        # all four streams must end byte-identical (common prefix)
        min_len = min(len(events[i]) for i in range(4))
        self.assertGreater(min_len, 0)
        for idx in range(min_len):
            e0 = events[0][idx]
            for i in range(1, 4):
                self.assertEqual(e0, events[i][idx], f"event #{idx} differs")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM],
        )

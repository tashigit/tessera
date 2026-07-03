#!/usr/bin/env python3
# Fault-injection test for the route-exploration scenario (N3): crash an
# assigned explorer mid-probe and prove the fleet still completes, exercising
# the f=1 tolerance of the 4-peer mesh plus the protocol's lease/timeout.
#
# Scenario: R1, R2, R4 are blocked and R3 is the only open route. Bot 2 (home
# lane R3) deterministically claims R3, freezes mid-lane without reporting
# (freeze_at_x), and is then SIGKILLed: vertex_node, coordinator, and mock all
# die. Without the lease the mission deadlocks: R3 is neither blocked nor
# free, so nobody can claim it. With the lease the survivors propose a
# consensus `timeout`, R3 is released, a survivor claims it, arrives, and the
# rest converge.
#
# Asserts:
#   * a survivor's state shows bot 2's R3 assignment released after the lease;
#   * winner_route == R3 on every survivor and all three survivors arrive;
#   * route exclusivity holds in every observed snapshot;
#   * the three surviving /vertex/event streams stay byte-identical
#     (3 of 4 nodes finalize: n=4 tolerates f=1);
#   * survivors shut down cleanly (the victim's -SIGKILL is intentional).
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
from vertex_ros2_msgs.msg import VertexEvent

HERE = os.path.dirname(__file__)
PEERS_PATH = os.path.join(HERE, "fixtures", "peers4.json")
COORDINATOR = os.path.join(HERE, "nodes", "mission_coordinator.py")
MOCK_ROBOT = os.path.join(HERE, "nodes", "mock_robot.py")
ROUTES = ["R1", "R2", "R3", "R4"]
HOME_LANE_Y = [2.25, 0.75, -0.75, -2.25]
BLOCKED = ["R1", "R2", "R4"]    # R3 is the only way through
VICTIM = 2                      # home lane R3: claims the open route, then dies
LEASE_SEC = 8.0
SURVIVORS = [i for i in range(4) if i != VICTIM]


def _load_peers():
    if not os.path.exists(PEERS_PATH):
        pytest.skip(f"{PEERS_PATH} missing — run fixtures/gen_peers4.sh first.")
    with open(PEERS_PATH) as f:
        return json.load(f)[:4]


@pytest.mark.launch_test
def generate_test_description():
    peers = _load_peers()
    actions = []
    victim_actions = []
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
        coordinator = launch_ros.actions.Node(
            executable=COORDINATOR, name="mission_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "routes": ROUTES, "goal_x": 3.6,
                         "num_bots": 4, "claim_interval_sec": 0.5,
                         "random_routes": False, "lease_sec": LEASE_SEC}],
            output="screen",
        )
        mock = launch_ros.actions.Node(
            executable=MOCK_ROBOT, name="mock_robot", namespace=ns,
            parameters=[{"home_lane_y": HOME_LANE_Y[i],
                         "blocked_routes": BLOCKED}
                        | ({"freeze_at_x": -1.0} if i == VICTIM else {})],
            output="screen",
        )
        actions += [vertex, coordinator, mock]
        if i == VICTIM:
            victim_actions = [vertex, coordinator, mock]

    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions), {"victim_actions": victim_actions}


class TestFaultInjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("fault_injection_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_lease_recovers_from_crashed_explorer(self, proc_info, victim_actions):
        latest = {i: None for i in range(4)}
        exclusivity_violations = []
        events = {i: [] for i in SURVIVORS}

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
            String, f"/robot_{i}/mission_state", make_cb(i), 10) for i in range(4)]
        subs += [self.node.create_subscription(
            VertexEvent, f"/robot_{i}/vertex/event", make_ev_cb(i), 50)
            for i in SURVIVORS]

        # Phase 1: wait until consensus has assigned R3 to the victim.
        deadline = time.time() + 60.0
        assigned_to_victim = False
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            st = latest[SURVIVORS[0]]
            if st and st.get("assigned", {}).get(str(VICTIM)) == "R3":
                assigned_to_victim = True
                break
        self.assertTrue(assigned_to_victim,
                        f"victim never got R3: {latest[SURVIVORS[0]]}")

        # Phase 2: crash the whole victim robot (engine, coordinator, body).
        # It froze at x=-1.0 so it has reported neither blocked nor arrived:
        # R3 is held by a dead robot.
        for action in victim_actions:
            os.kill(proc_info[action].pid, signal.SIGKILL)

        # Phase 3: survivors must lease-timeout R3, re-claim it, and complete.
        deadline = time.time() + 120.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all(latest[i] and i in latest[i].get("arrived", [])
                   for i in SURVIVORS):
                break

        winners = set()
        for i in SURVIVORS:
            st = latest[i]
            self.assertIsNotNone(st, f"robot_{i} never published mission_state")
            self.assertIn(i, st.get("arrived", []),
                          f"survivor robot_{i} never arrived: {st}")
            self.assertEqual(st.get("winner_route"), "R3", f"robot_{i}: {st}")
            winners.add(st.get("winner_route"))
            # the dead robot's assignment must be gone (lease released it)
            self.assertNotEqual(st.get("assigned", {}).get(str(VICTIM)), "R3",
                                f"robot_{i} still shows the dead bot on R3: {st}")
        self.assertEqual(len(winners), 1)
        self.assertEqual(exclusivity_violations, [],
                         f"route shared by two bots: {exclusivity_violations[:3]}")

        # 3 of 4 nodes must still finalize one byte-identical order (f=1).
        min_len = min(len(events[i]) for i in SURVIVORS)
        self.assertGreater(min_len, 0, "survivors delivered no consensus events")
        for idx in range(min_len):
            e0 = events[SURVIVORS[0]][idx]
            for i in SURVIVORS[1:]:
                self.assertEqual(e0, events[i][idx],
                                 f"survivor event #{idx} differs")

        for s in subs:
            self.node.destroy_subscription(s)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Survivors exit via the harness's SIGINT/SIGTERM. The victim's three
        # processes were SIGKILLed BY the test — that is the fault being
        # injected, not a crash to flag.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -signal.SIGINT, -signal.SIGTERM,
                                  -signal.SIGKILL],
        )

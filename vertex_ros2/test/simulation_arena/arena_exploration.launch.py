#!/usr/bin/env python3
"""Container-side launch for the arena-exploration simulation (the second
simulation: five Pioneer 3-AT robots sweep the pioneer arena; see README.md).

Same native-Webots + Docker-ROS split as route exploration: Webots runs
NATIVELY on the Mac and drives worlds/pioneer_arena.wbt, whose
pioneer_explorer controllers connect to rosbridge; this launch runs everything
else inside the Jazzy container:

  * rosbridge_server (WebSocket, port 9090) — the Mac-side pioneer_explorer
    controllers exchange /robot_i/pose, /telemetry, /detection, /goto here.
  * per robot i (0..4):
      - vertex_node        /vertex/* remapped to /robot_i/vertex/*
      - arena_coordinator  the replicated FSM, in namespace /robot_i

Keys for vertex_node come from fixtures/peers5.json (fixtures/gen_peers5.sh).
Launched by `docker compose run --rm --service-ports sim simarena`.
"""

import json
import os

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
PEERS_PATH = os.path.join(HERE, "fixtures", "peers5.json")
COORDINATOR = os.path.join(HERE, "nodes", "arena_coordinator.py")

N = 5
# 20 sectors over the interesting part of the 50x50 arena (walls at +-25)
GRID = {"grid_nx": 5, "grid_ny": 4, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 8.0, "cell_h": 7.5}


def _load_peers():
    with open(PEERS_PATH) as f:
        peers = json.load(f)
    assert len(peers) >= N, "need 5 peers — run fixtures/gen_peers5.sh"
    return peers[:N]


def generate_launch_description():
    peers = _load_peers()
    actions = [
        # rosbridge: exposes the ROS graph to the native Webots controllers.
        Node(package="rosbridge_server", executable="rosbridge_websocket",
             name="rosbridge", parameters=[{"port": 9090}], output="screen"),
    ]

    for i, me in enumerate(peers):
        ns = f"/robot_{i}"
        others = [p for j, p in enumerate(peers) if j != i]
        peer_specs = [f"{p['public']}@{p['addr']}" for p in others]

        # vertex_node — contract topics are absolute /vertex/*, so remap explicitly.
        actions.append(Node(
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

        # arena_coordinator — relative topics resolve under /robot_i;
        # pose/telemetry/detection come from the Mac controller via rosbridge,
        # goto goes back the same way. Physical driving is slower than the
        # mock, so give physical outcomes more slack than the launch_test.
        actions.append(Node(
            executable=COORDINATOR, name="arena_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "num_bots": N, **GRID,
                         "claim_interval_sec": 1.5,
                         "cover_radius": 2.0,
                         "stream_timeout_sec": 5.0,
                         "suspect_after_sec": 30.0,
                         # long enough to cross a sector around obstacles,
                         # short enough that a crater-guarded centre (e.g.
                         # S09, on a pit rim) cycles to `unreachable` while
                         # the viewer is still watching
                         "stall_sec": 35.0}],
            output="screen",
        ))

    return LaunchDescription(actions)

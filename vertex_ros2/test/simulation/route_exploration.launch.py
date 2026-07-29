#!/usr/bin/env python3
"""Container-side launch for the route-exploration simulation (native-Webots +
Docker-ROS split — see ../README.md §8 and worlds/README.md).

Webots runs NATIVELY on the Mac (it has no arm64 Linux build); this launch runs
everything else inside the Jazzy Docker container:

  * rosbridge_server (WebSocket, port 9090) — the Mac-side waypoint_follower
    controllers connect here to exchange /robot_i/pose, /barrier, /drive.
  * per robot i (0..3):
      - vertex_node          the crate under test; /vertex/* remapped to /robot_i/vertex/*
      - mission_coordinator  the replicated FSM, in namespace /robot_i

Keys for vertex_node come from fixtures/peers4.json (fixtures/gen_peers4.sh).
Launched by `docker compose run --rm --service-ports sim` (entrypoint verb `sim`).
"""

import json
import os
import tempfile

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
PEERS_PATH = os.path.join(HERE, "fixtures", "peers4.json")
COORDINATOR = os.path.join(HERE, "nodes", "mission_coordinator.py")
ROUTES = ["R1", "R2", "R3", "R4"]


def _load_peers():
    with open(PEERS_PATH) as f:
        peers = json.load(f)
    assert len(peers) >= 4, "need 4 peers — run fixtures/gen_peers4.sh"
    return peers[:4]


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


def generate_launch_description():
    peers = _load_peers()
    keydir = tempfile.mkdtemp(prefix="vertex_route_exploration_keys_")
    actions = [
        # rosbridge: exposes the ROS graph to the native Webots followers.
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
                "vertex.secret_key_path": _secret_key_file(me["secret"], keydir, f"robot{i}"),
                "vertex.peers": peer_specs,
                "options.heartbeat_us": 50000,
            }],
            output="screen",
        ))

        # mission_coordinator — relative topics resolve under /robot_i; pose/barrier
        # come from the Mac follower via rosbridge, drive goes back the same way.
        actions.append(Node(
            executable=COORDINATOR, name="mission_coordinator", namespace=ns,
            parameters=[{"robot_id": i, "routes": ROUTES, "goal_x": 3.6,
                         "num_bots": 4, "claim_interval_sec": 1.5}],
            output="screen",
        ))

    return LaunchDescription(actions)

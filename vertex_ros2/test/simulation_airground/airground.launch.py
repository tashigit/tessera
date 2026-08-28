#!/usr/bin/env python3
"""Container-side launch for the air/ground simulation (the third simulation:
two Pioneer 3-AT sweepers and two DJI Mavic 2 PRO surveyors clear the arena
together; see README.md).

Same native-Webots + Docker-ROS split as the first two simulations, with one
addition. Webots runs NATIVELY on the Mac and drives
worlds/airground_arena.wbt; this launch runs everything else in the Jazzy
container. What is new is that only HALF the fleet is a ROS citizen:

  ground tier (tessera)
    rosbridge_server      WebSocket on 9090; the Mac-side pioneer_sweeper
                          controllers exchange /bot_i/pose, /telemetry, /goto
    vertex_node x2        /vertex/* remapped into /bot_i/
    ground_coordinator x2 the replicated fold, in namespace /bot_i

  air tier (no ROS at all)
    air_agent x2          native Rust binaries linking tashi-vertex directly.
                          They reach their Mac-side mavic_surveyor controllers
                          over a plain JSON-lines TCP socket, not rosbridge,
                          and reach the rest of the fleet only through Vertex.

All four are peers in one committee and fold one ordered log. Keys come from
fixtures/peers_airground.json (fixtures/gen_peers_airground.sh).

Launched by `docker compose run --rm --service-ports sim simairground`.
"""

import json
import os
import tempfile

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
PEERS_PATH = os.path.join(HERE, "fixtures", "peers_airground.json")
COORDINATOR = os.path.join(HERE, "nodes", "ground_coordinator.py")
AIR_AGENT = os.path.join(HERE, "air_agent", "target", "debug", "air_agent")
LOGDIR = os.path.join(HERE, "logs")

BOTS = ["bot_0", "bot_1"]
DRONES = ["drone_0", "drone_1"]
AGENTS = BOTS + DRONES

# 12 sectors over the interesting part of the 50x50 arena (walls at +-25),
# in 6 survey blocks of 2 sectors each. Blocks outnumber the drones, so
# survey claims actually contend.
GRID = {"grid_nx": 4, "grid_ny": 3, "grid_min_x": -20.0, "grid_min_y": -15.0,
        "cell_w": 10.0, "cell_h": 10.0, "block_w": 2, "block_h": 1}

# The Mac-side mavic_surveyor controllers dial these. Ports are forwarded by
# the compose `sim` service alongside rosbridge's 9090.
LINK_PORT = {"drone_0": 48633, "drone_1": 48634}


def _load_peers():
    with open(PEERS_PATH) as f:
        peers = {p["name"]: p for p in json.load(f)}
    missing = [a for a in AGENTS if a not in peers]
    assert not missing, f"peers file is missing {missing} — run fixtures/gen_peers_airground.sh"
    return peers


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


def _peer_specs(peers, me):
    return [f"{p['public']}@{p['addr']}" for n, p in peers.items() if n != me]


def generate_launch_description():
    peers = _load_peers()
    keydir = tempfile.mkdtemp(prefix="vertex_airground_keys_")
    os.makedirs(LOGDIR, exist_ok=True)

    actions = [
        # rosbridge: exposes the ROS graph to the native Webots sweepers.
        # The drones do not use it.
        Node(package="rosbridge_server", executable="rosbridge_websocket",
             name="rosbridge", parameters=[{"port": 9090}], output="screen"),
    ]

    # ---- ground tier ----
    for me in BOTS:
        ns = f"/{me}"
        p = peers[me]
        actions.append(Node(
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
        # Physical driving is slower than the mock, so physical outcomes get
        # more slack here than in the launch_test. stall_sec has to be long
        # enough to cross a sector around trees and rocks, but short enough
        # that a crater-guarded centre is corroborated while you are watching.
        actions.append(Node(
            executable=COORDINATOR, name="ground_coordinator", namespace=ns,
            parameters=[{"agent_id": me, "agents": AGENTS, **GRID,
                         "claim_interval_sec": 1.5,
                         "cover_radius": 2.5,
                         "stream_timeout_sec": 5.0,
                         "suspect_after_sec": 30.0,
                         "stall_sec": 35.0}],
            output="screen",
        ))

    # ---- air tier ----
    for me in DRONES:
        p = peers[me]
        cmd = [AIR_AGENT,
               "--id", me,
               "--bind", p["addr"],
               "--key", p["secret"],
               "--link", f"0.0.0.0:{LINK_PORT[me]}",
               "--log", os.path.join(LOGDIR, f"{me}_airground.log")]
        for spec in _peer_specs(peers, me):
            cmd += ["--peer", spec]
        actions.append(ExecuteProcess(cmd=cmd, name=f"air_agent_{me}",
                                      output="screen"))

    return LaunchDescription(actions)

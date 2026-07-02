#!/usr/bin/env python3
"""waypoint_follower — each ExplorerBot's Webots controller, run NATIVELY on the
Mac (Webots has no arm64 Linux build, so it stays on the host and uses the GPU).

It can't import rclpy (no ROS on the host), so it bridges to the ROS 2 graph in
the Docker container over rosbridge's WebSocket protocol using the pure-Python
`websocket-client` library. It self-namespaces by robot name:

  publishes (advertise)         subscribes
    /<name>/pose   PointStamped   /<name>/drive  String   (route id | STAGING | STOP)
    /<name>/barrier Bool

`barrier` is reported by STALL (no progress toward the current waypoint), not the
distance sensor — a robot pushing against a raised barrier stops advancing. This
is immune to chassis-pitch/bounce sensor artifacts. See ../../README.md.

Container side: rosbridge_server (:9090) + vertex_node + mission_coordinator
(route_exploration.launch.py). Override the URL with WEBOTS_ROSBRIDGE_URL.
Geometry mirrors config/routes.yaml.
"""

import json
import math
import os
import threading

import websocket  # websocket-client (pip install websocket-client)

from controller import Robot  # Webots

# --- geometry (mirror config/routes.yaml) ---
ROUTE_LANE_Y = {"R1": 2.25, "R2": 0.75, "R3": -0.75, "R4": -2.25}
HOME_LANE_Y = {"robot_0": 2.25, "robot_1": 0.75, "robot_2": -0.75, "robot_3": -2.25}
STAGING_X, ENTER_X, EXIT_X, GOAL_X = -4.0, -3.2, 3.0, 3.6
# Transfer corridors: vertical lane-change columns kept clear of parked bots
# (parked west at x=STAGING_X, east at x=GOAL_PARK_X), so a moving bot never
# drives through a stationary one. All lane changes happen in these columns.
TRANSFER_W_X = -3.5          # west column (between staging spots and lane mouths)
TRANSFER_E_X, GOAL_PARK_X = 4.0, 4.5   # east column and goal-side parking

# --- control ---
WHEEL_RADIUS, HALF_AXLE, MAX_WHEEL = 0.05, 0.11, 28.0
V_MAX, KP_ANG, WP_TOL = 0.8, 5.0, 0.25
STALL_WINDOW, STALL_EPS = 2.0, 0.04     # no progress for this long (s) => blocked
PROX_DIST, PROX_CONFIRM = 0.40, 3       # hold if something is this close ahead
                                         # (debounced over consecutive readings)
STANDOFF_SEC = 3.5                      # held this long while maneuvering ->
BACKOFF_SEC = 1.0                       # reverse briefly, then pause by bot id
                                         # (deterministic head-on deadlock breaker)
WHEELS = ("front left wheel motor", "front right wheel motor",
          "rear left wheel motor", "rear right wheel motor")


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def snap_row(y):
    """Nearest lane row — travel along rows only, never along a divider line."""
    return min(ROUTE_LANE_Y.values(), key=lambda ly: abs(ly - y))


class RosbridgeClient:
    """Minimal rosbridge v2 WebSocket client (advertise/publish/subscribe)."""

    def __init__(self, url, name):
        self.pose_topic = f"/{name}/pose"
        self.barrier_topic = f"/{name}/barrier"
        self.drive_topic = f"/{name}/drive"
        self.drive = None
        self.connected = False
        self._app = websocket.WebSocketApp(
            url, on_open=self._on_open, on_message=self._on_message,
            on_close=self._on_close, on_error=lambda ws, e: None)
        threading.Thread(
            target=self._app.run_forever,
            kwargs={"reconnect": 3, "ping_interval": 10}, daemon=True).start()

    def _on_open(self, ws):
        ws.send(json.dumps({"op": "advertise", "topic": self.pose_topic,
                            "type": "geometry_msgs/PointStamped"}))
        ws.send(json.dumps({"op": "advertise", "topic": self.barrier_topic,
                            "type": "std_msgs/Bool"}))
        ws.send(json.dumps({"op": "subscribe", "topic": self.drive_topic,
                            "type": "std_msgs/String"}))
        self.connected = True
        print(f"[waypoint_follower] bridged to {self.drive_topic[:-6]}* via rosbridge")

    def _on_close(self, ws, *_):
        self.connected = False

    def _on_message(self, ws, message):
        try:
            d = json.loads(message)
        except ValueError:
            return
        if d.get("op") == "publish" and d.get("topic") == self.drive_topic:
            self.drive = d["msg"]["data"]

    def _send(self, topic, msg):
        if not self.connected:
            return
        try:
            self._app.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))
        except Exception:
            self.connected = False

    def publish_pose(self, x, y):
        self._send(self.pose_topic, {
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "map"},
            "point": {"x": x, "y": y, "z": 0.0}})

    def publish_barrier(self, blocked):
        self._send(self.barrier_topic, {"data": bool(blocked)})


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    name = robot.getName()
    home_y = HOME_LANE_Y.get(name, 0.0)

    motors = [robot.getDevice(w) for w in WHEELS]  # FL, FR, RL, RR
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    gps = robot.getDevice("gps"); gps.enable(ts)
    imu = robot.getDevice("imu"); imu.enable(ts)
    ds = robot.getDevice("front_ds"); ds.enable(ts)   # proximity guard

    def drive(lv, rv):                              # skid steer: left pair / right pair
        scale = max(1.0, abs(lv) / MAX_WHEEL, abs(rv) / MAX_WHEEL)
        motors[0].setVelocity(lv / scale); motors[2].setVelocity(lv / scale)
        motors[1].setVelocity(rv / scale); motors[3].setVelocity(rv / scale)

    url = os.environ.get("WEBOTS_ROSBRIDGE_URL", "ws://localhost:9090")
    bridge = RosbridgeClient(url, name)

    def build_waypoints(c, cur_y):
        """Path for command c from the current row. Lane changes always go via
        the transfer columns; never diagonally through lanes or parked bots."""
        row = snap_row(cur_y)
        if c in ROUTE_LANE_Y:
            y = ROUTE_LANE_Y[c]
            return [(TRANSFER_W_X, row), (TRANSFER_W_X, y),
                    (ENTER_X, y), (EXIT_X, y), (GOAL_X + 0.2, y)]
        if c == "STAGING":
            return [(TRANSFER_W_X, row), (TRANSFER_W_X, home_y),
                    (STAGING_X, home_y)]
        if c == "STOP":
            return [(TRANSFER_E_X, row), (TRANSFER_E_X, home_y),
                    (GOAL_PARK_X, home_y)]
        return []

    cmd, waypoints, idx = "STAGING", [(STAGING_X, home_y)], 0
    prox_streak = 0
    standoff_since, backoff_until, pause_until = None, 0.0, 0.0
    last_rebuild = -10.0
    bot_idx = int(name[-1]) if name and name[-1].isdigit() else 0

    while robot.step(ts) != -1:
        now = robot.getTime()
        gx, gy, _ = gps.getValues()
        yaw = imu.getRollPitchYaw()[2]
        bridge.publish_pose(gx, gy)

        # Proximity guard (physical no-collision net for the parallel model):
        # anything solid directly ahead within PROX_DIST -> hold until it clears.
        # Debounced so a transient pitch-induced floor hit can't stop the car.
        dist = ds.getValue() / 1000.0
        prox_streak = prox_streak + 1 if 0.0 < dist < PROX_DIST else 0
        prox_hold = prox_streak >= PROX_CONFIRM

        # react to a new drive command. Lane changes always go via the transfer
        # columns (never diagonally through other bots' parking spots).
        if bridge.drive is not None and bridge.drive != cmd:
            cmd = bridge.drive
            waypoints = build_waypoints(cmd, gy)
            idx, best_x, t_progress = 0, -1e9, now

        pursuing = cmd in ROUTE_LANE_Y
        if idx >= len(waypoints):
            drive(0.0, 0.0)
            bridge.publish_barrier(False)
            continue

        wx, wy = waypoints[idx]

        # West-column discipline: the vertical (row-change) leg at TRANSFER_W_X
        # is only safe truly WEST of the divider ends (x=-3.0). If we drifted
        # back east while chasing it (standoff backoffs push us into a lane),
        # the straight line to the leg clips a divider tip and we wedge on the
        # wall forever — rebuild the path from where we actually are instead.
        if wx == TRANSFER_W_X and abs(wy - gy) > 0.5 \
                and gx > TRANSFER_W_X + 0.45 and now - last_rebuild > 2.0:
            last_rebuild = now
            waypoints = build_waypoints(cmd, gy)
            idx, best_x, t_progress = 0, -1e9, now
            wx, wy = waypoints[idx] if waypoints else (gx, gy)
            print(f"[waypoint_follower:{name}] drifted off the west column — "
                  "rebuilding path")

        # Wrong-lane self-correction: with several cars funneling into one lane
        # mouth, a cornering overshoot can drop us into the WRONG lane. If we are
        # lane-driving (waypoint east) but off the target row, back out and
        # re-enter properly — never wedge on a barrier that isn't even ours.
        target_row = ROUTE_LANE_Y.get(cmd)
        if (target_row is not None and wx > gx + 0.05 and gx > ENTER_X - 0.2
                and abs(gy - target_row) > 0.5):
            waypoints = build_waypoints(cmd, gy)
            idx, best_x, t_progress = 0, -1e9, now
            print(f"[waypoint_follower:{name}] wrong lane (y={gy:.2f}, "
                  f"want {target_row}) — backing out and re-entering")
            wx, wy = waypoints[idx]

        # Blocked detection: only while committed EAST inside the corridor, ON
        # the target row, AND actually pushing east (waypoint ahead) — there, a
        # real barrier on OUR route is the only thing that stops progress. Never
        # while maneuvering at the junction, backing out (waypoint west), or in
        # the wrong lane: those produced false `blocked` reports for the open
        # winner route (fleet churn).
        pushing_east = (pursuing and gx > ENTER_X + 0.1 and wx > gx + 0.05
                        and target_row is not None
                        and abs(gy - target_row) < 0.4)
        if not pushing_east:
            best_x, t_progress = gx, now             # maneuvering: never "blocked"
        elif gx > best_x + STALL_EPS:
            best_x, t_progress = gx, now             # made eastward progress
        bridge.publish_barrier(pushing_east and (now - t_progress) > STALL_WINDOW)

        d = math.hypot(wx - gx, wy - gy)
        # a west-column waypoint only counts as reached when we are genuinely IN
        # the column (west of the divider ends) — WP_TOL slack must not unlock
        # the vertical leg while we are still inside a lane mouth
        if d < WP_TOL and (wx != TRANSFER_W_X or gx < TRANSFER_W_X + 0.2):
            idx += 1
            continue
        # Standoff breaker: two cars nose-to-nose while maneuvering (not lane
        # driving) can hold each other forever. After STANDOFF_SEC, reverse
        # briefly and then pause for a per-bot time — ids differ, so exactly one
        # yields and the deadlock breaks deterministically.
        if now < backoff_until:
            drive(-7.0, -7.0)
            continue
        if now < pause_until:
            drive(0.0, 0.0)
            continue
        if prox_hold and not pushing_east:
            if standoff_since is None:
                standoff_since = now
            elif now - standoff_since > STANDOFF_SEC:
                backoff_until = now + BACKOFF_SEC
                pause_until = backoff_until + 0.5 + bot_idx * 1.2
                standoff_since = None
                print(f"[waypoint_follower:{name}] standoff — backing off")
                continue
        else:
            standoff_since = None

        err = wrap(math.atan2(wy - gy, wx - gx) - yaw)
        if prox_hold:
            # Something (a barrier / another car) directly ahead: never advance
            # into it, but KEEP TURNING toward the current waypoint. If the
            # waypoint is behind (e.g. returning from a barrier), the car spins
            # away, the sensor clears, and it drives off — a full stop here
            # would freeze the car at the barrier forever. In a convoy the
            # waypoint is straight ahead (err~0), so this is just holding.
            v, w = 0.0, KP_ANG * err
        else:
            v = V_MAX * max(0.0, math.cos(err))
            # slow down in the junction / transfer zone (west of the lane mouths
            # and east of the goal line): tighter turns, no cornering overshoot
            # into a neighbouring lane
            if gx < ENTER_X + 0.2 or gx > GOAL_X:
                v = min(v, 0.45)
            w = KP_ANG * err
        lv = (v - w * HALF_AXLE) / WHEEL_RADIUS
        rv = (v + w * HALF_AXLE) / WHEEL_RADIUS
        drive(lv, rv)

    node_shutdown = getattr(bridge, "_app", None)
    if node_shutdown:
        node_shutdown.close()


if __name__ == "__main__":
    main()

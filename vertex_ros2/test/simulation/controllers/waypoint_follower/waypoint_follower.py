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

Bot-to-bot collision avoidance runs on a sim-local pose radio (the proto's
Emitter/Receiver): every car broadcasts its pose each step and reacts to the
other three with deterministic rules (convoy gap, yield by id, keep-right
passing, dodge around parked cars, emergency stop bubble) — see assess_peers.

Container side: rosbridge_server (:9090) + vertex_node + mission_coordinator
(route_exploration.launch.py). Override the URL with WEBOTS_ROSBRIDGE_URL.
Geometry mirrors config/routes.yaml.
"""

import collections
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
BARRIER_X = 1.5              # barrier plane inside every lane (config/routes.yaml)
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

# --- peer awareness (bot-to-bot collision avoidance) ---
# Every bot broadcasts its pose on a sim-local radio (Emitter/Receiver in the
# proto) every step, so the guard below can see peers the single forward ray
# cannot (crossing traffic in the transfer columns, parked bots, angled
# approaches). Sim-local means no rosbridge round-trip: poses stay fresh and
# velocity estimates stay correct at any simulation speed, and the WebSocket
# is not flooded. All rules are deterministic: ids break every tie.
PEER_STALE = 1.0                        # ignore peer poses older than this (sim s)
AVOID_R = 1.1                           # react to a peer within this range ahead
AVOID_CONE = math.radians(95)           # "ahead" = within this bearing of my nose
HARD_R = 0.44                           # emergency stop bubble (centre distance)
HARD_CONE = math.radians(85)
FOLLOW_GAP = 0.65                       # convoy: hold this far behind same-dir peer
STATIONARY = 0.06                       # peer speed below this = parked / stopped
SWERVE = 0.35                           # keep-right lateral shift when passing head-on
WHEELS = ("front left wheel motor", "front right wheel motor",
          "rear left wheel motor", "rear right wheel motor")


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def snap_row(y):
    """Nearest lane row — travel along rows only, never along a divider line."""
    return min(ROUTE_LANE_Y.values(), key=lambda ly: abs(ly - y))


def clear_ground(x, y):
    """True when (x, y) is drivable open ground: inside the arena walls and not
    inside a divider band (dividers span x in [-3, 3] at y = -1.5 / 0 / 1.5).
    The margins cover half the car's footprint, so a car CENTRED here does not
    scrape a wall or beach itself on a divider tip."""
    if not (-4.7 < x < 4.7 and -2.75 < y < 2.75):
        return False
    if -3.2 < x < 3.2:
        for dy in (-1.5, 0.0, 1.5):
            if abs(y - dy) < 0.35:
                return False
    return True


def dodge_heading(gx, gy, px, py, wx, wy):
    """Absolute heading that skirts a stopped bot at (px, py) on the way to
    waypoint (wx, wy): roughly perpendicular to the line to the bot, on the
    side where the waypoint lies. The whole near path (not just its endpoint)
    must be drivable, or a dodge could beach the car on a divider tip. Falls
    back to the other side; None when neither side is clear."""
    to_peer = math.atan2(py - gy, px - gx)
    to_wp = math.atan2(wy - gy, wx - gx)
    side = 1.0 if wrap(to_wp - to_peer) >= 0.0 else -1.0
    for s in (side, -side):
        h = to_peer + s * (math.pi / 2.0 + 0.35)
        if all(clear_ground(gx + d * math.cos(h), gy + d * math.sin(h))
               for d in (0.35, 0.7)):
            return h
    return None


def assess_peers(gx, gy, yaw, wx, wy, my_idx, peers, in_corridor, heading_east):
    """React to the nearest other bot ahead. `peers` is [(idx, x, y, vx, vy)].

    Returns (hold, slow, swerve, dodge):
      hold    stop and wait: convoy gap, yielding to crossing/oncoming traffic,
              queueing behind a stopped explorer in a lane, or the emergency
              bubble (a peer directly ahead inside HARD_R)
      slow    cap speed while a nearby conflict resolves
      swerve  shade the aim point to the right (head-on lane passing; both
              bots shift right, so they clear each other inside one lane)
      dodge   absolute heading to steer around a stopped bot, or None

    Deterministic tie-break: on crossing or open-ground head-on conflicts the
    LOWER id has right of way; the higher id holds until the way is clear.
    """
    nearest = None
    for idx, px, py, vx, vy in peers:
        dist = math.hypot(px - gx, py - gy)
        if dist > AVOID_R:
            continue
        if abs(wrap(math.atan2(py - gy, px - gx) - yaw)) > AVOID_CONE:
            continue
        if nearest is None or dist < nearest[0]:
            nearest = (dist, idx, px, py, vx, vy)
    if nearest is None:
        return False, False, False, None

    dist, idx, px, py, vx, vy = nearest
    hold = slow = swerve = False
    dodge = None
    if math.hypot(vx, vy) > STATIONARY:
        hdiff = abs(wrap(math.atan2(vy, vx) - yaw))
        if hdiff < math.radians(60):            # same direction: convoy spacing
            hold = dist < FOLLOW_GAP
        elif hdiff > math.radians(120):         # head-on
            if in_corridor:
                swerve = slow = True            # lane passing: keep right
            elif my_idx > idx:
                hold = True                     # open ground: higher id gives way
            else:
                dodge = dodge_heading(gx, gy, px, py, wx, wy)
                hold, slow = dodge is None, True
        else:                                   # crossing: lower id has right of way
            if my_idx > idx:
                hold = True
            else:
                slow = True
    else:                                       # peer is parked / stopped
        if not in_corridor:
            dodge = dodge_heading(gx, gy, px, py, wx, wy)
            hold, slow = dodge is None, True    # drive around it (or wait if boxed in)
        elif heading_east:
            hold = True                         # queue behind it in the lane
        else:
            swerve = slow = True                # returning: squeeze past on the right

    # emergency bubble: never advance onto a peer directly ahead. While dodging
    # the bot we are skirting stays close by design, so only a genuinely tight
    # distance stops the dodge.
    if dist < (0.33 if dodge is not None else HARD_R) \
            and abs(wrap(math.atan2(py - gy, px - gx) - yaw)) < HARD_CONE:
        hold = True
    return hold, slow, swerve, dodge


class PeerRadio:
    """Sim-local pose sharing over the proto's Emitter/Receiver pair. Each bot
    broadcasts {id, x, y, t} every step; the receive side keeps a short history
    per peer for a sim-time velocity estimate. Degrades to 'no peers' when the
    devices are missing (older world files)."""

    def __init__(self, robot, ts, my_idx):
        self.my_idx = my_idx
        self.tx = robot.getDevice("peer_tx")
        self.rx = robot.getDevice("peer_rx")
        if self.rx is not None:
            self.rx.enable(ts)
        self._hist = {}          # peer idx -> deque of (sim_t, x, y)

    def step(self, now, gx, gy):
        """Broadcast my pose and drain received peer poses (sim time `now`)."""
        if self.tx is not None:
            self.tx.send(json.dumps(
                {"i": self.my_idx, "x": round(gx, 3), "y": round(gy, 3),
                 "t": round(now, 3)}))
        if self.rx is None:
            return
        while self.rx.getQueueLength() > 0:
            try:
                d = json.loads(self.rx.getString())
                if d["i"] != self.my_idx:
                    hist = self._hist.setdefault(
                        d["i"], collections.deque(maxlen=16))
                    hist.append((float(d["t"]), float(d["x"]), float(d["y"])))
            except (ValueError, KeyError, TypeError):
                pass
            self.rx.nextPacket()

    def peer_states(self, now):
        """Fresh peers with velocity over the history window (all sim time):
        [(idx, x, y, vx, vy)]."""
        out = []
        for idx, hist in self._hist.items():
            t1, x1, y1 = hist[-1]
            if now - t1 > PEER_STALE:
                continue
            t0, x0, y0 = hist[0]
            vx = vy = 0.0
            if t1 - t0 > 0.05:
                vx, vy = (x1 - x0) / (t1 - t0), (y1 - y0) / (t1 - t0)
            out.append((idx, x1, y1, vx, vy))
        return out


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

    def build_waypoints(c, cur_x, cur_y):
        """Path for command c from the current pose. Lane changes always go via
        the transfer columns; never diagonally through lanes or parked bots."""
        row = snap_row(cur_y)
        if c in ROUTE_LANE_Y:
            y = ROUTE_LANE_Y[c]
            if cur_x > BARRIER_X + 0.3:
                # Already east of the barrier plane (e.g. a converger whose
                # winner lane was re-blocked BEHIND it by a barrier flip):
                # the west column is unreachable, so reach the target row via
                # the east column. Crossing the goal line on the target row is
                # a legitimate arrival, and the coordinator detects it there.
                return [(TRANSFER_E_X, row), (TRANSFER_E_X, y),
                        (GOAL_X + 0.2, y)]
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
    radio = PeerRadio(robot, ts, bot_idx)
    peer_mode = ""                      # last printed avoidance mode (debug)
    step_no = 0
    last_barrier = None

    while robot.step(ts) != -1:
        step_no += 1
        now = robot.getTime()
        gx, gy, _ = gps.getValues()
        yaw = imu.getRollPitchYaw()[2]
        radio.step(now, gx, gy)
        # the coordinator only needs a coarse pose (arrival check), so throttle
        # the WebSocket feed; in fast mode an every-step feed floods rosbridge
        # and delays the drive commands behind a giant pose queue
        if step_no % 3 == 0:
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
            waypoints = build_waypoints(cmd, gx, gy)
            idx, best_x, t_progress = 0, -1e9, now

        pursuing = cmd in ROUTE_LANE_Y
        if idx >= len(waypoints):
            drive(0.0, 0.0)
            if last_barrier is not False or step_no % 15 == 0:
                last_barrier = False
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
            waypoints = build_waypoints(cmd, gx, gy)
            idx, best_x, t_progress = 0, -1e9, now
            wx, wy = waypoints[idx] if waypoints else (gx, gy)
            print(f"[waypoint_follower:{name}] drifted off the west column — "
                  "rebuilding path")

        # Wrong-lane self-correction: with several cars funneling into one lane
        # mouth, a cornering overshoot can drop us into the WRONG lane. If we are
        # lane-driving (waypoint east) but off the target row, back out and
        # re-enter properly — never wedge on a barrier that isn't even ours.
        # (Not while the waypoint is the east column: that IS the off-row
        # escape path for a bot trapped east of a raised barrier.)
        target_row = ROUTE_LANE_Y.get(cmd)
        if (target_row is not None and wx > gx + 0.05 and gx > ENTER_X - 0.2
                and wx < TRANSFER_E_X - 0.1 and abs(gy - target_row) > 0.5):
            waypoints = build_waypoints(cmd, gx, gy)
            idx, best_x, t_progress = 0, -1e9, now
            print(f"[waypoint_follower:{name}] wrong lane (y={gy:.2f}, "
                  f"want {target_row}) — backing out and re-entering")
            wx, wy = waypoints[idx]

        # Peer avoidance: react to the other bots' shared poses (see the rules
        # in assess_peers). Any active reaction also freezes the stall clock so
        # waiting for a peer is never misreported as a barrier `blocked`.
        in_corridor = -2.95 < gx < 2.95
        p_hold, p_slow, p_swerve, p_dodge = assess_peers(
            gx, gy, yaw, wx, wy, bot_idx, radio.peer_states(now),
            in_corridor, wx > gx)
        peer_active = p_hold or p_slow or p_swerve or p_dodge is not None
        mode = ("hold" if p_hold else "dodge" if p_dodge is not None
                else "swerve" if p_swerve else "slow" if p_slow else "")
        if mode != peer_mode:
            peer_mode = mode
            if mode:
                print(f"[waypoint_follower:{name}] peer-avoid: {mode}")

        # Blocked detection: only while committed EAST inside the corridor, ON
        # the target row, AND actually pushing east (waypoint ahead) — there, a
        # real barrier on OUR route is the only thing that stops progress. Never
        # while maneuvering at the junction, backing out (waypoint west), in
        # the wrong lane, or held up by another bot: those produced false
        # `blocked` reports for the open winner route (fleet churn).
        pushing_east = (pursuing and gx > ENTER_X + 0.1 and wx > gx + 0.05
                        and target_row is not None
                        and abs(gy - target_row) < 0.4)
        if not pushing_east or peer_active:
            best_x, t_progress = gx, now             # maneuvering: never "blocked"
        elif gx > best_x + STALL_EPS:
            best_x, t_progress = gx, now             # made eastward progress
        blocked_now = (pushing_east and not peer_active
                       and (now - t_progress) > STALL_WINDOW)
        if blocked_now != last_barrier or step_no % 15 == 0:
            last_barrier = blocked_now
            bridge.publish_barrier(blocked_now)

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
        if (prox_hold or p_hold) and not pushing_east:
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

        # steering aim: normally the waypoint; a dodge steers around a stopped
        # bot; a swerve shades the near-term aim to the right for lane passing
        aim_x, aim_y = wx, wy
        v_cap = V_MAX
        if p_dodge is not None:
            aim_x, aim_y = gx + math.cos(p_dodge), gy + math.sin(p_dodge)
            v_cap = 0.3
        elif p_swerve:
            dirn = math.atan2(wy - gy, wx - gx)
            sx = gx + math.cos(dirn) + SWERVE * math.sin(dirn)
            sy = gy + math.sin(dirn) - SWERVE * math.cos(dirn)
            if clear_ground(sx, sy):
                aim_x, aim_y = sx, sy
            v_cap = 0.4
        elif p_slow:
            v_cap = 0.35

        err = wrap(math.atan2(aim_y - gy, aim_x - gx) - yaw)
        if prox_hold or p_hold:
            # Something (a barrier / another car) directly ahead: never advance
            # into it, but KEEP TURNING toward the current aim. If the
            # waypoint is behind (e.g. returning from a barrier), the car spins
            # away, the sensor clears, and it drives off — a full stop here
            # would freeze the car at the barrier forever. In a convoy the
            # waypoint is straight ahead (err~0), so this is just holding.
            v, w = 0.0, KP_ANG * err
        else:
            v = v_cap * max(0.0, math.cos(err))
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

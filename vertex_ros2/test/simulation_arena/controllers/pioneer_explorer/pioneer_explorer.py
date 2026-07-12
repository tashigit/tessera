#!/usr/bin/env python3
"""pioneer_explorer — each Pioneer 3-AT's Webots controller for the
arena-exploration simulation (worlds/pioneer_arena.wbt), run NATIVELY on the
Mac (Webots has no arm64 Linux build, so it stays on the host and uses the GPU).

It can't import rclpy (no ROS on the host), so it bridges to the ROS 2 graph
in the Docker container over rosbridge's WebSocket protocol using the
pure-Python `websocket-client` library — the same split as waypoint_follower.
It self-namespaces by robot name:

  publishes (advertise)               subscribes
    /<name>/pose      PointStamped      /<name>/goto  String
    /<name>/telemetry String (JSON)          (sector id | HOLD | STOP)

`telemetry` carries per-sensor-stream ages (seconds since the lidar / camera
last delivered data). The arena_coordinator folds these into its consensus
health beacon — this is the live replacement for the MQTT stack's
telemetry_monitor_node, with the voting collapsed into the ordered-log fold.

Driving is deliberately simple and deterministic: steer toward the target
sector's centre, with lidar-reactive avoidance around trees, rocks, deer and
the other robots. Sectors whose centre cannot be reached (obstacle clusters,
pits) surface as a progress stall, which the coordinator turns into
consensus `abandon` / `unreachable` reports — physical truth in, shared
decisions out. There is no controller-to-controller channel: all cross-robot
coordination flows through Vertex in the container.

Grid geometry mirrors arena_exploration.launch.py (5x4 sectors, 8 x 7.5 m,
south-west corner at (-20, -15)).
"""

import json
import math
import os
import threading

import websocket  # websocket-client (pip install websocket-client)

from controller import Robot  # Webots

# --- geometry (mirror arena_exploration.launch.py GRID) ---
GRID_NX, GRID_NY = 5, 4
GRID_MIN_X, GRID_MIN_Y = -20.0, -15.0
CELL_W, CELL_H = 8.0, 7.5

# --- control (Pioneer 3-AT) ---
WHEELS = ("front left wheel", "front right wheel",
          "back left wheel", "back right wheel")
# The proto's motor limit is 6.4 rad/s; command just below it so a scaled
# command never float-equals the limit (Webots warns on every such step).
WHEEL_RADIUS, HALF_AXLE, MAX_WHEEL = 0.111, 0.2, 6.3
V_MAX, KP_ANG = 0.6, 3.0
W_MAX = 2.0                            # turn-rate cap: keeps the wheels off
                                       # their stops while turning, so the
                                       # robot arcs instead of pure-spinning
AVOID_NEAR, AVOID_SLOW = 1.0, 1.8      # lidar clearances (m): swerve / slow
AVOID_COMMIT = 1.5                     # hold a chosen swerve side this long,
                                       # so facing an obstacle never dithers
STUCK_WINDOW, STUCK_DIST = 6.0, 0.10   # driving but not displacing => wedged
BACKOFF_SEC = 2.0                      # wedged: reverse-and-turn to break free
TELEMETRY_PERIOD = 0.5                 # seconds between telemetry publishes


def sector_centers():
    centers = {}
    for iy in range(GRID_NY):
        for ix in range(GRID_NX):
            sid = f"S{iy * GRID_NX + ix:02d}"
            centers[sid] = (GRID_MIN_X + (ix + 0.5) * CELL_W,
                            GRID_MIN_Y + (iy + 0.5) * CELL_H)
    return centers


CENTERS = sector_centers()


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RosbridgeClient:
    """Minimal rosbridge v2 WebSocket client (advertise/publish/subscribe)."""

    def __init__(self, url, name):
        self.pose_topic = f"/{name}/pose"
        self.telemetry_topic = f"/{name}/telemetry"
        self.goto_topic = f"/{name}/goto"
        self.goto = None
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
        ws.send(json.dumps({"op": "advertise", "topic": self.telemetry_topic,
                            "type": "std_msgs/String"}))
        ws.send(json.dumps({"op": "subscribe", "topic": self.goto_topic,
                            "type": "std_msgs/String"}))
        self.connected = True
        print(f"[pioneer_explorer] bridged to {self.goto_topic[:-5]}* via rosbridge")

    def _on_close(self, ws, *_):
        self.connected = False

    def _on_message(self, ws, message):
        try:
            d = json.loads(message)
        except ValueError:
            return
        if d.get("op") == "publish" and d.get("topic") == self.goto_topic:
            self.goto = d["msg"]["data"]

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

    def publish_telemetry(self, scan_age, camera_age):
        self._send(self.telemetry_topic, {"data": json.dumps(
            {"scan_age": round(scan_age, 2), "camera_age": round(camera_age, 2)})})


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    name = robot.getName()

    motors = [robot.getDevice(w) for w in WHEELS]  # FL, FR, BL, BR
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    gps = robot.getDevice("gps"); gps.enable(ts)
    imu = robot.getDevice("inertial unit"); imu.enable(ts)
    lidar = robot.getDevice("Sick LMS 291"); lidar.enable(ts)
    camera = robot.getDevice("Webcam for Robotino 3")
    if camera is not None:
        camera.enable(ts * 8)          # low rate: presence, not vision

    def drive(lv, rv):                 # skid steer: left pair / right pair
        scale = max(1.0, abs(lv) / MAX_WHEEL, abs(rv) / MAX_WHEEL)
        motors[0].setVelocity(lv / scale); motors[2].setVelocity(lv / scale)
        motors[1].setVelocity(rv / scale); motors[3].setVelocity(rv / scale)

    url = os.environ.get("WEBOTS_ROSBRIDGE_URL", "ws://localhost:9090")
    bridge = RosbridgeClient(url, name)

    cmd = "HOLD"
    step_no = 0
    last_scan_t = last_image_t = last_telemetry = -1e9
    bot_idx = int(name[-1]) if name and name[-1].isdigit() else 0
    swerve_dir, swerve_until = 0.0, -1e9   # committed avoidance side
    anchor_x = anchor_y = None             # stuck detection anchor
    anchor_t = -1e9
    backoff_until = -1e9
    escapes = 0                            # alternate escape turn direction

    while robot.step(ts) != -1:
        step_no += 1
        now = robot.getTime()
        gx, gy, _ = gps.getValues()
        yaw = imu.getRollPitchYaw()[2]

        # sensor-stream freshness (the input to the consensus health beacon)
        ranges = lidar.getRangeImage()
        if ranges:
            last_scan_t = now
        if camera is not None and camera.getImage() is not None:
            last_image_t = now

        # throttle the WebSocket feed: a coarse pose is enough for the
        # coordinator's arrival check, and flooding rosbridge delays goto
        if step_no % 3 == 0:
            bridge.publish_pose(gx, gy)
        if now - last_telemetry >= TELEMETRY_PERIOD:
            last_telemetry = now
            bridge.publish_telemetry(now - last_scan_t, now - last_image_t)

        if bridge.goto is not None:
            cmd = bridge.goto

        target = CENTERS.get(cmd)
        if target is None:             # HOLD / STOP / not yet commanded
            drive(0.0, 0.0)
            anchor_x = None            # stuck detection only applies to driving
            continue

        # Wedged? Commanded to drive but not displacing (pressed against a
        # tree, a rock, a deer, another robot, or a pit rim the horizontal
        # lidar cannot see): reverse with a turn to break free. Escape turn
        # direction alternates per attempt and is seeded per bot, so two
        # mutually wedged robots pick different sides.
        if anchor_x is None or now < backoff_until:
            pass
        elif math.hypot(gx - anchor_x, gy - anchor_y) > STUCK_DIST:
            anchor_x, anchor_y, anchor_t = gx, gy, now
        elif now - anchor_t > STUCK_WINDOW:
            backoff_until = now + BACKOFF_SEC
            escapes += 1
            anchor_x, anchor_y, anchor_t = gx, gy, now
            print(f"[pioneer_explorer:{name}] wedged at "
                  f"({gx:.1f}, {gy:.1f}) — backing off")
        if anchor_x is None:
            anchor_x, anchor_y, anchor_t = gx, gy, now
        if now < backoff_until:
            side = 1.0 if (bot_idx + escapes) % 2 else -1.0
            drive(-3.5 - side * 1.5, -3.5 + side * 1.5)
            continue

        # steer toward the sector centre, lidar-reactive around obstacles
        err = wrap(math.atan2(target[1] - gy, target[0] - gx) - yaw)
        v = V_MAX * max(0.0, math.cos(err))
        w = max(-W_MAX, min(W_MAX, KP_ANG * err))
        if ranges:
            n = len(ranges)
            third = max(1, n // 3)
            right = min(ranges[:third])            # lidar scans right-to-left
            front = min(ranges[third:n - third])
            left = min(ranges[n - third:])
            if front < AVOID_NEAR:
                # Blocked ahead: turn toward the clearer side and crawl.
                # Commit to the chosen side for a while — re-picking every
                # step dithers left/right in front of a wide obstacle.
                if now > swerve_until:
                    swerve_dir = 1.5 if left > right else -1.5
                swerve_until = now + AVOID_COMMIT
                v, w = 0.1, swerve_dir
            elif front < AVOID_SLOW:
                v = min(v, 0.25)
            elif min(left, right) < 0.5:
                v = min(v, 0.35)       # hugging something: ease off

        lv = (v - w * HALF_AXLE) / WHEEL_RADIUS
        rv = (v + w * HALF_AXLE) / WHEEL_RADIUS
        drive(lv, rv)

    app = getattr(bridge, "_app", None)
    if app:
        app.close()


if __name__ == "__main__":
    main()

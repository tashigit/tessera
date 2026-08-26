#!/usr/bin/env python3
"""mock_airframe — stands in for a Webots Mavic 2 PRO and its controller.

The headless equivalent of controllers/mavic_surveyor/: same JSON-lines TCP
protocol, straight-line kinematics instead of PID-stabilised flight, and a
scripted pit map instead of a real downward ranger. It exists so the air tier
can be asserted in CI with no Webots, no GPU and no 3D, exactly as
mock_pioneer does for the ground tier in the arena simulation.

Nothing here talks to ROS or to Vertex. It only speaks to its air_agent over
the socket, which is the same boundary the real controller sits behind.

    python3 mock_airframe.py --link 127.0.0.1:48633 --start -20,-15 \
        [--pits S02,S05] [--battery-drain 0.004] [--fail-at 12 --recover-at 40]
"""

import argparse
import json
import math
import select
import socket
import sys
import time

# Grid geometry, mirroring airground_fsm / air_agent / the launch file.
NX, NY = 4, 3
MIN_X, MIN_Y = -20.0, -15.0
CELL_W, CELL_H = 10.0, 10.0

SURVEY_ALT = 12.0
SPEED = 9.0             # m/s, brisk enough to keep the launch_test short
TICK = 0.1              # seconds per integration step
PIT_EXTRA = 1.4         # a crater reads this much further than the altitude


def sector_at(x, y):
    ix = math.floor((x - MIN_X) / CELL_W)
    iy = math.floor((y - MIN_Y) / CELL_H)
    if ix < 0 or iy < 0 or ix >= NX or iy >= NY:
        return None
    return f"S{iy * NX + ix:02d}"


class LineSocket:
    """Newline-delimited JSON over a socket, polled rather than blocked on.

    Deliberately not `socket.makefile()`. A file object over a socket that has
    a timeout is documented as leaving its buffer inconsistent when the
    timeout fires, and in practice the very next read raises
    `OSError: cannot read from timed out object` for good. Since this loop has
    to keep integrating flight whether or not a command arrived, it polls with
    select and does its own buffering. The real Webots controller needs the
    same shape, because there Webots owns the main loop through robot.step().
    """

    def __init__(self, sock):
        self.sock = sock
        self.sock.setblocking(False)
        self.buf = b""
        self.closed = False

    def lines(self, timeout=0.0):
        """Every complete line available right now. Sets `closed` on EOF."""
        out = []
        r, _, _ = select.select([self.sock], [], [], timeout)
        if not r:
            return out
        try:
            chunk = self.sock.recv(65536)
        except BlockingIOError:
            return out
        except OSError:
            self.closed = True
            return out
        if not chunk:
            self.closed = True
            return out
        self.buf += chunk
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            if line.strip():
                out.append(line.decode("utf-8", "replace"))
        return out

    def send_json(self, obj):
        try:
            self.sock.sendall((json.dumps(obj) + "\n").encode())
            return True
        except OSError:
            self.closed = True
            return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--link", required=True, help="host:port of this drone's air_agent")
    ap.add_argument("--start", default="-20,-15", help="x,y spawn position")
    ap.add_argument("--pits", default="", help="comma-separated sectors that are craters")
    ap.add_argument("--battery-drain", type=float, default=0.0,
                    help="battery lost per second of flight (0 disables the rtb lease)")
    ap.add_argument("--recharge", type=float, default=0.25,
                    help="battery regained per second while landed")
    ap.add_argument("--fail-at", type=float, default=-1.0,
                    help="seconds after start when the sensor stream goes stale")
    ap.add_argument("--recover-at", type=float, default=-1.0,
                    help="seconds after start when it comes back")
    args = ap.parse_args()

    pits = {s for s in args.pits.split(",") if s}
    sx, sy = (float(v) for v in args.start.split(","))
    x, y, z = sx, sy, 0.0
    goal = None
    landing = False
    battery = 1.0

    host, port = args.link.rsplit(":", 1)
    # The agent listens; retry until it is up, since launch order is not
    # guaranteed and the agent binds only after joining the committee.
    sock = None
    for _ in range(200):
        try:
            sock = socket.create_connection((host, int(port)), timeout=5)
            break
        except OSError:
            time.sleep(0.1)
    if sock is None:
        print(f"[mock_airframe] could not reach {args.link}", file=sys.stderr)
        return 1
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    wire = LineSocket(sock)
    print(f"[mock_airframe] linked to air_agent at {args.link}", flush=True)

    started = time.time()

    while not wire.closed:
        now = time.time()
        elapsed = now - started

        # --- consume any commands that arrived, blocking at most one tick ---
        for line in wire.lines(timeout=TICK):
            try:
                cmd = json.loads(line)
            except ValueError:
                continue        # malformed: ignore, keep flying
            t = cmd.get("t")
            if t == "goto":
                goal = (cmd.get("x", x), cmd.get("y", y), cmd.get("z", SURVEY_ALT))
                landing = False
            elif t == "hold":
                goal, landing = None, False
            elif t == "land":
                goal, landing = None, True
        if wire.closed:
            break

        # --- integrate one step of flight ---
        if landing:
            z = max(0.0, z - SPEED * TICK)
            battery = min(1.0, battery + args.recharge * TICK)
        else:
            if goal is not None:
                gx, gy, gz = goal
                dx, dy, dz = gx - x, gy - y, gz - z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                step = SPEED * TICK
                if dist <= step or dist == 0.0:
                    x, y, z = gx, gy, gz
                else:
                    x += dx / dist * step
                    y += dy / dist * step
                    z += dz / dist * step
            battery = max(0.0, battery - args.battery_drain * TICK)

        # --- sensing ---
        cell = sector_at(x, y)
        clearance = z + (PIT_EXTRA if cell in pits else 0.0)

        # Scripted sensor-stream outage: the stream goes stale rather than the
        # process dying, so the drone reports itself unhealthy and the fold
        # releases its block. A hard crash is the `suspect` path instead.
        stale = (args.fail_at >= 0 and elapsed >= args.fail_at
                 and (args.recover_at < 0 or elapsed < args.recover_at))
        age = (elapsed - args.fail_at) if stale else 0.02

        wire.send_json({"t": "telemetry", "x": round(x, 3), "y": round(y, 3),
                        "z": round(z, 3), "clearance": round(clearance, 3),
                        "battery": round(battery, 4), "age": round(age, 3)})

    print("[mock_airframe] link closed", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)

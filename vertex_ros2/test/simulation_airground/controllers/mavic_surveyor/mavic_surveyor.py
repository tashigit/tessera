"""mavic_surveyor — each DJI Mavic 2 PRO's Webots controller for the
air/ground simulation (worlds/airground_arena.wbt), run NATIVELY on the Mac.

It is the airframe only. Every decision about WHERE to fly and WHAT to report
belongs to this drone's `air_agent`, the native Rust binary that holds the
Vertex membership and the replicated fold. This controller does two things:
flies where it is told, and says what it sees.

The link is newline-delimited JSON over a plain TCP socket, deliberately not
rosbridge. The Pioneers reach the ROS graph through rosbridge because they are
already ROS citizens; the air tier is not, and keeping its only outward
connection this dumb is what shows a drone needs nothing but the Vertex wire
protocol and the record schema to be a full member of the fleet.

  agent -> here     {"t":"goto","x":..,"y":..,"z":..} | {"t":"hold"} | {"t":"land"}
  here  -> agent    {"t":"telemetry","x":..,"y":..,"z":..,"clearance":..,
                     "battery":..,"age":..}

`clearance` is the whole reason the air tier exists. A Pioneer's Sick LMS 291
is horizontal and cannot see a hole, so the arena's craters are invisible from
the ground and a robot that drives into one is stuck for good. From up here a
pit reads as ground further away than the drone's own altitude, which is a
geometric signal rather than a vision problem, so it stays deterministic and
the assertions stay crisp.

Attitude control is the published Mavic 2 PRO law from Webots' own
`mavic2pro_patrol` sample, constants and all. It needs `basicTimeStep 8` and
linear/angular damping of 0.5, which the world sets.
"""

import json
import math
import os
import select
import socket
import sys

from controller import Robot  # Webots

# --- link ---
# One port per drone, matching airground.launch.py. Overridable for anyone
# running a differently-sized fleet.
DEFAULT_PORTS = {"drone_0": 48633, "drone_1": 48634}
AGENT_HOST = os.environ.get("AIR_AGENT_HOST", "127.0.0.1")

# --- flight envelope ---
SURVEY_ALT = 12.0       # metres; clears the arena's BigSassafras canopy
TARGET_PRECISION = 2.0  # metres, horizontal, before a waypoint counts reached
TELEMETRY_PERIOD = 0.1  # seconds between telemetry frames

# --- Mavic 2 PRO control constants (Webots' mavic2pro sample) ---
K_VERTICAL_THRUST = 68.5   # with this thrust the drone lifts
K_VERTICAL_OFFSET = 0.6    # vertical offset where it stabilises
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0
MAX_YAW_DISTURBANCE = 0.4
MAX_PITCH_DISTURBANCE = -1.0

# --- battery model ---
# Webots' battery field is not wired up here; a simple linear model is enough
# to exercise the `rtb` lease, and keeping it in software makes the timing
# reproducible between the live world and the headless mock.
FLIGHT_SECONDS = 900.0     # full charge to empty, airborne
RECHARGE_SECONDS = 60.0    # empty to full, landed


def clamp(v, lo, hi):
    return min(max(v, lo), hi)


class LineSocket:
    """Newline-delimited JSON over a socket, polled rather than blocked on.

    Webots owns the main loop through robot.step(), so this must never block.
    It also does its own buffering rather than using socket.makefile(): a file
    object over a socket with a timeout is documented as leaving its buffer
    inconsistent when the timeout fires, and in practice every later read
    then fails with "cannot read from timed out object".
    """

    def __init__(self, sock):
        self.sock = sock
        self.sock.setblocking(False)
        self.buf = b""
        self.closed = False

    def lines(self):
        out = []
        r, _, _ = select.select([self.sock], [], [], 0)
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
        except OSError:
            self.closed = True


class Surveyor(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.dt = int(self.getBasicTimeStep())
        self.name = self.getName()

        self.imu = self.getDevice("inertial unit")
        self.gps = self.getDevice("gps")
        self.gyro = self.getDevice("gyro")
        for d in (self.imu, self.gps, self.gyro):
            d.enable(self.dt)

        # The downward ranger added in the world's bodySlot. Absent if someone
        # loads a stock Mavic, in which case say so rather than silently
        # reporting flat ground everywhere.
        self.ranger = self.getDevice("ground_ranger")
        if self.ranger is not None:
            self.ranger.enable(self.dt)
        else:
            print(f"[{self.name}] WARNING: no ground_ranger device; pits will "
                  f"be invisible from the air", flush=True)

        self.camera = self.getDevice("camera")
        if self.camera is not None:
            self.camera.enable(self.dt)
        pitch_motor = self.getDevice("camera pitch")
        if pitch_motor is not None:
            pitch_motor.setPosition(1.5708)     # look straight down

        self.motors = [self.getDevice(n) for n in (
            "front left propeller", "front right propeller",
            "rear left propeller", "rear right propeller")]
        for m in self.motors:
            m.setPosition(float("inf"))
            m.setVelocity(1.0)

        self.goal = None            # (x, y, z) or None
        self.landing = False
        self.battery = 1.0
        self.last_tx = 0.0
        self.wire = None

    # ---- link ----
    def connect(self):
        port = int(os.environ.get(
            "AIR_AGENT_PORT", DEFAULT_PORTS.get(self.name, 48633)))
        # The agent binds its listener as it starts; Webots may well be up
        # first, so retry across steps rather than failing the controller.
        while self.step(self.dt) != -1:
            try:
                s = socket.create_connection((AGENT_HOST, port), timeout=2)
            except OSError:
                continue
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.wire = LineSocket(s)
            print(f"[{self.name}] linked to air_agent at {AGENT_HOST}:{port}",
                  flush=True)
            return True
        return False

    def read_commands(self):
        for line in self.wire.lines():
            try:
                cmd = json.loads(line)
            except ValueError:
                continue
            t = cmd.get("t")
            if t == "goto":
                self.goal = (cmd.get("x", 0.0), cmd.get("y", 0.0),
                             cmd.get("z", SURVEY_ALT))
                self.landing = False
            elif t == "hold":
                self.goal, self.landing = None, False
            elif t == "land":
                self.goal, self.landing = None, True

    # ---- flight ----
    def fly(self, pose, gyro_xy):
        """One step of the Mavic attitude/position law."""
        x, y, alt, roll, pitch, yaw = pose
        roll_accel, pitch_accel = gyro_xy

        if self.landing:
            target_alt = 0.0
            yaw_dist = pitch_dist = 0.0
        elif self.goal is not None:
            gx, gy, gz = self.goal
            target_alt = gz
            # Turn toward the waypoint, then pitch forward proportionally to
            # how well lined up we are: the sample's non-proportional,
            # decreasing function, which stops it running off on a bad heading.
            bearing = math.atan2(gy - y, gx - x)
            angle_left = (bearing - yaw + 2 * math.pi) % (2 * math.pi)
            if angle_left > math.pi:
                angle_left -= 2 * math.pi
            yaw_dist = MAX_YAW_DISTURBANCE * angle_left / (2 * math.pi)
            if math.hypot(gx - x, gy - y) < TARGET_PRECISION:
                pitch_dist = 0.0        # arrived: stop pushing, hold station
            else:
                pitch_dist = clamp(math.log10(abs(angle_left) + 1e-9),
                                   MAX_PITCH_DISTURBANCE, 0.1)
        else:
            target_alt = SURVEY_ALT
            yaw_dist = pitch_dist = 0.0

        roll_input = K_ROLL_P * clamp(roll, -1, 1) + roll_accel
        pitch_input = K_PITCH_P * clamp(pitch, -1, 1) + pitch_accel + pitch_dist
        yaw_input = yaw_dist
        d_alt = clamp(target_alt - alt + K_VERTICAL_OFFSET, -1, 1)
        vertical_input = K_VERTICAL_P * (d_alt ** 3.0)

        base = K_VERTICAL_THRUST + vertical_input
        fl = base - yaw_input + pitch_input - roll_input
        fr = base + yaw_input + pitch_input + roll_input
        rl = base + yaw_input - pitch_input - roll_input
        rr = base - yaw_input - pitch_input + roll_input
        self.motors[0].setVelocity(fl)
        self.motors[1].setVelocity(-fr)
        self.motors[2].setVelocity(-rl)
        self.motors[3].setVelocity(rr)

    def run(self):
        if not self.connect():
            return
        while self.step(self.dt) != -1:
            if self.wire.closed:
                print(f"[{self.name}] air_agent closed the link", flush=True)
                return
            self.read_commands()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, alt = self.gps.getValues()
            roll_accel, pitch_accel, _ = self.gyro.getValues()
            self.fly((x, y, alt, roll, pitch, yaw), (roll_accel, pitch_accel))

            now = self.getTime()
            step_s = self.dt / 1000.0
            if self.landing and alt < 0.5:
                self.battery = min(1.0, self.battery + step_s / RECHARGE_SECONDS)
            else:
                self.battery = max(0.0, self.battery - step_s / FLIGHT_SECONDS)

            if now - self.last_tx >= TELEMETRY_PERIOD:
                self.last_tx = now
                clearance = (self.ranger.getValue() if self.ranger is not None
                             else float("nan"))
                self.wire.send_json({
                    "t": "telemetry",
                    "x": round(x, 3), "y": round(y, 3), "z": round(alt, 3),
                    "clearance": round(clearance, 3),
                    "battery": round(self.battery, 4),
                    # Sensors are read every step, so freshness is one step.
                    # The agent folds this into its health beacon exactly as
                    # the ground tier folds its robots' stream ages.
                    "age": round(step_s, 3),
                })


if __name__ == "__main__":
    Surveyor().run()

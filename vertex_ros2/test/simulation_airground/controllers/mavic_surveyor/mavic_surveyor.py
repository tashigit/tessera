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
`mavic2pro_patrol` sample, constants unchanged. The POSITION loop on top is
not from the sample, which is a patrol demo rather than a position controller
and flies away when driven from a survey plan; see the note above `KP_POS`.

That sample also asks for `basicTimeStep 8`, which this world deliberately
does not use: measured here, a finer step takes real time from 0.90x to
0.014x because contact solving against the arena's sixteen procedural pit
meshes explodes below 32 ms. The world runs at 32 with damping 0.5 instead.
See the note in `worlds/airground_arena.wbt`.
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
STATUS_PERIOD = 5.0     # seconds between console status lines

# --- attitude loop: Webots' mavic2pro sample, unchanged ---
K_VERTICAL_THRUST = 68.5   # with this thrust the drone lifts
K_VERTICAL_OFFSET = 0.6    # vertical offset where it stabilises
K_VERTICAL_P = 3.0
K_ROLL_P = 50.0
K_PITCH_P = 30.0

# --- position loop: not from the sample ---
# The sample is a patrol demo, not a position controller: it pitches forward
# whenever it is roughly aligned with the next waypoint, with no position or
# velocity feedback, and never has to settle anywhere. Driven from a survey
# plan it simply flies away, which it did here, to 375 m outside the arena.
# What follows is a normal cascaded loop: position error and velocity in the
# body frame produce an attitude setpoint, which the sample's attitude loop
# then tracks.
KP_POS = 0.08              # tilt per metre of position error
KD_POS = 0.30              # tilt per m/s of closing speed (the brake)
MAX_TILT = 0.12            # rad, hard cap on commanded tilt
SP_SLEW = 0.12             # rad/s: never STEP the attitude setpoint (see fly)
K_YAW_P, K_YAW_D = 1.2, 0.3
YAW_AUTH = 0.25            # rad/s of yaw authority
GATE_FROM, GATE_SPAN = 6.0, 4.0   # ramp the position loop in with altitude

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
        # `name` is a read-only property on Webots' Robot, so keep ours
        # under a different attribute.
        self.robot_name = self.getName()

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
            print(f"[{self.robot_name}] WARNING: no ground_ranger device; pits will "
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

        self.yaw0 = None            # heading held for the whole flight
        self.pitch_sp = 0.0          # rate-limited attitude setpoints
        self.roll_sp = 0.0
        self.goal = None            # (x, y, z) or None
        self.landing = False
        self.battery = 1.0
        self.last_tx = 0.0
        self.last_status = 0.0
        self.wire = None

    # ---- link ----
    def connect(self):
        port = int(os.environ.get(
            "AIR_AGENT_PORT", DEFAULT_PORTS.get(self.robot_name, 48633)))
        # The agent binds its listener as it starts; Webots may well be up
        # first, so retry across steps rather than failing the controller.
        while self.step(self.dt) != -1:
            try:
                s = socket.create_connection((AGENT_HOST, port), timeout=2)
            except OSError:
                continue
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.wire = LineSocket(s)
            print(f"[{self.robot_name}] linked to air_agent at {AGENT_HOST}:{port}",
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
    def fly(self, pose, rates):
        """One step of the cascaded position/attitude loop.

        Position error and velocity are resolved into the body frame and become
        a tilt setpoint; the sample's attitude loop tracks that setpoint. Three
        details matter and each one cost a crash to find:

        * the setpoint is RATE-LIMITED. Stepping it, which every waypoint
          change does, excites an oscillation the attitude loop cannot damp,
          and the airframe inverts within a second;
        * the position loop RAMPS IN with altitude, so the climb-out does not
          build lateral speed before the loop has authority;
        * heading is HELD, not slewed to face each waypoint. The ranger and
          camera look straight down so heading is irrelevant to a survey, and
          yawing at each waypoint tumbled it.
        """
        x, y, alt, roll, pitch, yaw = pose
        roll_rate, pitch_rate, yaw_rate, vx, vy = rates

        if self.yaw0 is None:
            self.yaw0 = yaw

        if self.landing:
            target_alt, want_p, want_r = 0.0, 0.0, 0.0
        else:
            gx, gy, gz = self.goal if self.goal else (x, y, SURVEY_ALT)
            target_alt = gz
            ex, ey = gx - x, gy - y
            cy, sy = math.cos(yaw), math.sin(yaw)
            fwd, left = ex * cy + ey * sy, -ex * sy + ey * cy
            v_fwd, v_left = vx * cy + vy * sy, -vx * sy + vy * cy
            gate = clamp((alt - GATE_FROM) / GATE_SPAN, 0.0, 1.0)
            want_p = clamp((KP_POS * fwd - KD_POS * v_fwd) * gate, -MAX_TILT, MAX_TILT)
            want_r = clamp(-(KP_POS * left - KD_POS * v_left) * gate, -MAX_TILT, MAX_TILT)

        step = SP_SLEW * self.dt / 1000.0
        self.pitch_sp += clamp(want_p - self.pitch_sp, -step, step)
        self.roll_sp += clamp(want_r - self.roll_sp, -step, step)

        heading_err = (self.yaw0 - yaw + math.pi) % (2 * math.pi) - math.pi
        yaw_input = clamp(K_YAW_P * heading_err - K_YAW_D * yaw_rate,
                          -YAW_AUTH, YAW_AUTH)

        roll_input = K_ROLL_P * clamp(roll - self.roll_sp, -1, 1) + roll_rate
        pitch_input = K_PITCH_P * clamp(pitch - self.pitch_sp, -1, 1) + pitch_rate
        d_alt = clamp(target_alt - alt + K_VERTICAL_OFFSET, -1, 1)
        vertical_input = K_VERTICAL_P * (d_alt ** 3.0)

        base = K_VERTICAL_THRUST + vertical_input
        self.motors[0].setVelocity(base - yaw_input + pitch_input - roll_input)
        self.motors[1].setVelocity(-(base + yaw_input + pitch_input + roll_input))
        self.motors[2].setVelocity(-(base + yaw_input - pitch_input - roll_input))
        self.motors[3].setVelocity(base - yaw_input - pitch_input + roll_input)

    def run(self):
        if not self.connect():
            return
        while self.step(self.dt) != -1:
            if self.wire.closed:
                print(f"[{self.robot_name}] air_agent closed the link", flush=True)
                return
            self.read_commands()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x, y, alt = self.gps.getValues()
            roll_rate, pitch_rate, yaw_rate = self.gyro.getValues()
            vx, vy, _ = self.gps.getSpeedVector()
            self.fly((x, y, alt, roll, pitch, yaw),
                     (roll_rate, pitch_rate, yaw_rate, vx, vy))

            now = self.getTime()
            step_s = self.dt / 1000.0
            if self.landing and alt < 0.5:
                self.battery = min(1.0, self.battery + step_s / RECHARGE_SECONDS)
            else:
                self.battery = max(0.0, self.battery - step_s / FLIGHT_SECONDS)

            # A periodic line in the Webots console, so "is my drone actually
            # flying?" is answerable from the GUI without reading journals.
            if now - self.last_status >= STATUS_PERIOD:
                self.last_status = now
                where = ("landing" if self.landing else
                         f"-> ({self.goal[0]:.0f}, {self.goal[1]:.0f})"
                         if self.goal else "holding")
                print(f"[{self.robot_name}] t={now:6.1f}s "
                      f"pos=({x:6.1f},{y:6.1f}) alt={alt:5.1f}m "
                      f"clear={self.ranger.getValue():5.1f}m "
                      f"batt={self.battery:.0%} {where}", flush=True)

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

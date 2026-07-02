"""route_supervisor — user controls for the scenario, plus ground-truth access.

Controls (click the 3D view first so it has keyboard focus):
    1 / 2 / 3 / 4   EXCLUSIVE OPEN: open only that route (R1..R4) and block the
                    other three — the "find the one open path" test
    T               random test: open one random route, block the other three
    0               open every route (clear all barriers)
    R               RESET: send every car back to staging, open all routes,
                    and restart the mission (a fresh consensus epoch)

Barriers park below the floor (z = OPEN_Z) when open and rise into the lane
(z = BLOCK_Z) when blocked, so a blocked route physically stops a car.

RESET does two things:
  * physical — teleports each DEF ROBOTi back to its staging pose + resetPhysics,
    and opens all routes (works standalone in the demo world);
  * logical — publishes an incrementing epoch on /reset via rosbridge, which the
    mission_coordinators relay into Vertex as a `reset` transaction so the whole
    fleet wipes and re-explores from a consistent point (ROS scenario only; the
    rosbridge connection is a no-op if no server is running).

This is a Webots Supervisor: it also has ground-truth pose access
(getFromDef(...).getPosition()) for the launch_test assertions.
"""

import json
import os
import random
import threading

from controller import Supervisor

try:
    import websocket  # websocket-client; present on the host for the ROS bridge
    _HAVE_WS = True
except ImportError:
    _HAVE_WS = False

OPEN_Z = -0.5
BLOCK_Z = 0.2
BARRIERS = {"1": "BARRIER0", "2": "BARRIER1", "3": "BARRIER2", "4": "BARRIER3"}
ROBOT_DEFS = ["ROBOT0", "ROBOT1", "ROBOT2", "ROBOT3"]
STAGING = [(-4.0, 2.25), (-4.0, 0.75), (-4.0, -0.75), (-4.0, -2.25)]


class RosbridgePublisher:
    """Minimal rosbridge publisher for one std_msgs/Int32 topic."""

    def __init__(self, url, topic):
        self.topic = topic
        self.connected = False
        if not _HAVE_WS:
            return
        self._app = websocket.WebSocketApp(
            url, on_open=self._on_open, on_close=self._on_close,
            on_error=lambda ws, e: None)
        threading.Thread(target=self._app.run_forever,
                         kwargs={"reconnect": 3, "ping_interval": 10},
                         daemon=True).start()

    def _on_open(self, ws):
        ws.send(json.dumps({"op": "advertise", "topic": self.topic,
                            "type": "std_msgs/Int32"}))
        self.connected = True

    def _on_close(self, ws, *_):
        self.connected = False

    def publish(self, value):
        if not self.connected:
            return
        try:
            self._app.send(json.dumps({"op": "publish", "topic": self.topic,
                                       "msg": {"data": int(value)}}))
        except Exception:
            self.connected = False


def main():
    sup = Supervisor()
    ts = int(sup.getBasicTimeStep())
    kb = sup.getKeyboard()
    kb.enable(ts)

    barriers = {k: sup.getFromDef(d) for k, d in BARRIERS.items()}
    robots = [sup.getFromDef(d) for d in ROBOT_DEFS]
    for k, n in barriers.items():
        if n is None:
            print(f"[route_supervisor] WARNING: DEF {BARRIERS[k]} not found")
    blocked = {k: False for k in BARRIERS}
    reset_epoch = 0

    url = os.environ.get("WEBOTS_ROSBRIDGE_URL", "ws://localhost:9090")
    pub = RosbridgePublisher(url, "/reset")
    world_pub = RosbridgePublisher(url, "/world_changed")
    world_seq = 0

    def world_changed():
        # tell the fleet the barriers moved: recorded blocks are stale now
        nonlocal world_seq
        world_seq += 1
        world_pub.publish(world_seq)
        print(f"[route_supervisor] world-changed #{world_seq} broadcast")

    def set_block(k, want_blocked):
        n = barriers.get(k)
        if n is None:
            return
        tf = n.getField("translation")
        x, y, _ = tf.getSFVec3f()
        tf.setSFVec3f([x, y, BLOCK_Z if want_blocked else OPEN_Z])
        blocked[k] = want_blocked
        print(f"[route_supervisor] route R{k}: "
              f"{'BLOCKED' if want_blocked else 'open'}")

    def do_reset():
        nonlocal reset_epoch
        for i, node in enumerate(robots):
            if node is None:
                continue
            x, y = STAGING[i]
            node.getField("translation").setSFVec3f([x, y, 0.05])
            node.getField("rotation").setSFRotation([0, 0, 1, 0])
            node.resetPhysics()
        for k in BARRIERS:
            set_block(k, False)
        reset_epoch += 1
        pub.publish(reset_epoch)
        print(f"[route_supervisor] RESET #{reset_epoch} — cars to staging, routes open.")

    def open_only(k_open):
        """Exclusive-open test mode: open route k_open, block the other three."""
        for kk in BARRIERS:
            set_block(kk, kk != k_open)
        print(f"[route_supervisor] TEST: only R{k_open} open, others blocked.")
        world_changed()

    print("[route_supervisor] ready — 1-4: open ONLY that route; "
          "T: random open-one; 0: all open; R: reset.")

    # Test fixture: export one main-view frame if WEBOTS_EXPORT_IMAGE is set
    # (used by the harness to verify framing/behaviour headlessly). Inert otherwise.
    export_to = os.environ.get("WEBOTS_EXPORT_IMAGE")
    step_no = 0

    # Test fixture (inert without the env var):
    #   WEBOTS_AUTOTEST="open<k>"  once all cars are committed in their lanes,
    #                              exclusively open route k, then log ground truth.
    #   WEBOTS_AUTOTEST="turn3"    the user's multi-turn scenario: open R2 first;
    #                              once two cars have reached the goal area, flip
    #                              to open R1 (stale-block stress) — the fleet
    #                              must re-explore and still get everyone home.
    # Logs positions, pairwise near-collisions, and SUCCESS when all four arrive.
    autotest = os.environ.get("WEBOTS_AUTOTEST", "")
    at_triggered = False
    at_flip_done = False
    at_last_log = 0.0
    at_done = False
    COLLIDE_DIST = 0.24          # centre distance ~ two half-lengths overlapping

    # Webots returns a held key's code on EVERY step, so a single press would
    # otherwise toggle many times. Act only on the rising edge (keys newly down
    # this step that were not down last step).
    prev_down = set()
    while sup.step(ts) != -1:
        step_no += 1
        if export_to and step_no in (90, 240):
            out = export_to.replace(".png", f"_{step_no}.png")
            sup.exportImage(out, 100)
            print(f"[route_supervisor] exported frame to {out}")

        if (autotest.startswith("open") or autotest == "turn3") and not at_done:
            t = sup.getTime()
            poses = [n.getPosition() if n else (0, 0, 0) for n in robots]
            first_open = autotest[4] if autotest.startswith("open") else "2"
            # trigger once every car is committed inside its lane (x > -2.9)
            if not at_triggered and all(p[0] > -2.9 for p in poses):
                open_only(first_open)
                at_triggered = True
                print(f"[autotest] t={t:.1f}s TRIGGER: exclusive-open R{first_open}")
            # turn3: once two cars made it, flip the world to open-R1-only —
            # every recorded block is now stale; the fleet must re-discover
            if (autotest == "turn3" and at_triggered and not at_flip_done
                    and sum(1 for p in poses if p[0] > 3.4) >= 2):
                open_only("1")
                at_flip_done = True
                print(f"[autotest] t={t:.1f}s FLIP: now only R1 open "
                      "(stale-block stress)")
            for i in range(len(poses)):          # live collision monitor
                for j in range(i + 1, len(poses)):
                    dx = poses[i][0] - poses[j][0]
                    dy = poses[i][1] - poses[j][1]
                    if (dx * dx + dy * dy) ** 0.5 < COLLIDE_DIST:
                        print(f"[autotest] t={t:.1f}s COLLISION robots {i}&{j}")
            if t - at_last_log >= 5.0:           # position telemetry
                at_last_log = t
                pos = " ".join(f"r{i}=({p[0]:.1f},{p[1]:.1f})"
                               for i, p in enumerate(poses))
                print(f"[autotest] t={t:.1f}s {pos}")
            if at_triggered and (autotest != "turn3" or at_flip_done) \
                    and all(p[0] > 3.4 for p in poses):
                print(f"[autotest] t={t:.1f}s SUCCESS: all four cars reached "
                      "the goal area after the exclusive-open")
                at_done = True
        down = set()
        k = kb.getKey()
        while k != -1:
            down.add(k & 0xFF)
            k = kb.getKey()

        for code in down - prev_down:            # newly pressed this step
            ch = chr(code) if 0 <= code < 128 else ""
            if ch in BARRIERS:
                open_only(ch)                    # exclusive open: 1 open, 3 blocked
            elif ch in ("t", "T"):
                open_only(random.choice(list(BARRIERS)))
            elif ch == "0":
                for kk in BARRIERS:
                    set_block(kk, False)
                print("[route_supervisor] all routes open.")
                world_changed()
            elif ch in ("r", "R"):
                do_reset()
        prev_down = down


if __name__ == "__main__":
    main()

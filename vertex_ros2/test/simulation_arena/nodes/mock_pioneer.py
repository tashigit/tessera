#!/usr/bin/env python3
"""mock_pioneer — headless stand-in for the pioneer_explorer Webots controller,
used by arena_exploration.launch_test.py so the full vertex_node +
arena_coordinator consensus loop can be asserted in CI without Webots.

It emulates the physical robot the coordinator reacts to:
  * goto=<sector> : drive straight toward that sector's centre.
  * goto=HOLD     : stay put.
  * goto=STOP     : mission over, halt.

Publishes /robot_i/pose (PointStamped), /robot_i/telemetry (String JSON of
per-stream sensor ages, the input to the coordinator's health beacon), and
/robot_i/detection (String JSON) for planted sightings. Subscribes
/robot_i/goto (String). Grid geometry comes from arena_fsm.make_grid with the
same parameters the coordinator uses.

Test hooks:
  * fail_streams_after_sec  : from this point the reported sensor ages grow
      (the camera/lidar "died") -> the coordinator self-reports not-ok and the
      fleet marks this bot unhealthy. -1 disables.
  * recover_streams_after_sec : ages return to 0 -> readmission. -1 disables.
  * detections_json : list of {"id", "label", "x", "y"} planted sightings,
      each published once when the robot comes within detect_radius of it —
      or, with "at_sec", at a fixed time instead (deterministic scripting for
      the health-gating assertion).
"""

import json
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from arena_fsm import make_grid

SPEED = 0.35        # metres per 0.1 s tick (~3.5 m/s) — keeps the test short
DETECT_RADIUS = 3.0


class MockPioneer(Node):
    def __init__(self):
        super().__init__("mock_pioneer")
        self.declare_parameter("start_x", -24.0)
        self.declare_parameter("start_y", 0.0)
        self.declare_parameter("grid_nx", 5)
        self.declare_parameter("grid_ny", 4)
        self.declare_parameter("grid_min_x", -20.0)
        self.declare_parameter("grid_min_y", -15.0)
        self.declare_parameter("cell_w", 8.0)
        self.declare_parameter("cell_h", 7.5)
        self.declare_parameter("fail_streams_after_sec", -1.0)
        self.declare_parameter("recover_streams_after_sec", -1.0)
        self.declare_parameter("detections_json", "[]")

        p = lambda n: self.get_parameter(n).value
        self.x = float(p("start_x"))
        self.y = float(p("start_y"))
        _, self.centers = make_grid(
            int(p("grid_nx")), int(p("grid_ny")),
            float(p("grid_min_x")), float(p("grid_min_y")),
            float(p("cell_w")), float(p("cell_h")))
        self.fail_after = float(p("fail_streams_after_sec"))
        self.recover_after = float(p("recover_streams_after_sec"))
        self.detections = json.loads(str(p("detections_json")))
        self._published_dets = set()

        self.t0 = time.monotonic()
        self.cmd = "HOLD"

        self.pose_pub = self.create_publisher(PointStamped, "pose", 10)
        self.telemetry_pub = self.create_publisher(String, "telemetry", 10)
        self.detection_pub = self.create_publisher(String, "detection", 10)
        self.create_subscription(String, "goto", self._on_goto, 10)
        self.create_timer(0.1, self._tick)

    def _on_goto(self, msg: String):
        self.cmd = msg.data

    def _streams_failed(self) -> bool:
        if self.fail_after < 0:
            return False
        t = time.monotonic() - self.t0
        if t < self.fail_after:
            return False
        if self.recover_after >= 0 and t >= self.recover_after:
            return False
        return True

    def _tick(self):
        # drive
        if self.cmd in self.centers:
            cx, cy = self.centers[self.cmd]
            dx, dy = cx - self.x, cy - self.y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist > 1e-6:
                step = min(SPEED, dist)
                self.x += dx / dist * step
                self.y += dy / dist * step
        # HOLD / STOP: stay put

        ps = PointStamped()
        ps.header.frame_id = "map"
        ps.point.x, ps.point.y = self.x, self.y
        self.pose_pub.publish(ps)

        # telemetry: per-stream sensor ages, the coordinator's health input
        if self._streams_failed():
            t = time.monotonic() - self.t0
            age = t - self.fail_after
        else:
            age = 0.0
        self.telemetry_pub.publish(String(data=json.dumps(
            {"scan_age": age, "camera_age": age})))

        # planted sightings: publish each once, on schedule or on proximity
        for det in self.detections:
            if det["id"] in self._published_dets:
                continue
            if "at_sec" in det:
                due = (time.monotonic() - self.t0) >= float(det["at_sec"])
            else:
                due = ((self.x - det["x"]) ** 2
                       + (self.y - det["y"]) ** 2) ** 0.5 < DETECT_RADIUS
            if due:
                self._published_dets.add(det["id"])
                self.detection_pub.publish(String(data=json.dumps(det)))


def main(args=None):
    rclpy.init(args=args)
    # Stateless test fixture: exit immediately on the harness's SIGINT/SIGTERM
    # instead of walking rclpy's teardown, which has a native shutdown race
    # that can fail a launch_test's clean-exit assertion through no fault of
    # the system under test (same shape as mock_robot.py).
    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    node = MockPioneer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        pass                    # benign rclpy teardown race on SIGINT
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""mock_pioneer — headless stand-in for the Webots pioneer_sweeper controller,
so the mixed-fleet consensus loop can be asserted in CI without Webots.

The ground-tier twin of mock_airframe.py. It emulates the physical robot the
ground_coordinator reacts to:
  * goto=<sector> : drive straight toward that sector's centre
  * goto=HOLD     : stay put
  * goto=STOP     : mission over, halt

Publishes pose (PointStamped) and telemetry (String JSON of per-stream sensor
ages, the input to the coordinator's health beacon); subscribes goto (String).

The one hook that matters for this scenario is `pits_json`. A Pioneer's lidar
is horizontal and cannot see a hole, so a mock robot commanded into a crater
does what the real one does: it drives to the rim and stops making progress.
The coordinator turns that stall into `abandon`, and into `corroborate` if a
drone had already flagged the cell from above. That is the ground half of the
evidence rule, and it is why a drone reporting a block falsely clear costs one
robot rather than the mission.

Test hooks:
  * pits_json : sectors that are craters. Approaching one, the robot stops at
      `pit_rim` metres from the centre and cannot get closer.
  * fail_streams_after_sec / recover_streams_after_sec : scripted sensor-stream
      outage, driving the health beacon unhealthy and back. -1 disables.
"""

import json
import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from airground_fsm import make_sectors

SPEED = 0.45        # metres per 0.1 s tick (~4.5 m/s) — keeps the test short
TICK = 0.1


class MockPioneer(Node):
    def __init__(self):
        super().__init__("mock_pioneer")
        self.declare_parameter("agent_id", "bot_0")
        self.declare_parameter("start_x", -20.0)
        self.declare_parameter("start_y", -12.0)
        self.declare_parameter("grid_nx", 4)
        self.declare_parameter("grid_ny", 3)
        self.declare_parameter("grid_min_x", -20.0)
        self.declare_parameter("grid_min_y", -15.0)
        self.declare_parameter("cell_w", 10.0)
        self.declare_parameter("cell_h", 10.0)
        self.declare_parameter("pits_json", "[]")
        # How close to a crater's centre the robot can get before the ground
        # falls away. Must exceed the coordinator's cover_radius or the sector
        # would count as explored despite the hole.
        self.declare_parameter("pit_rim", 4.0)
        self.declare_parameter("fail_streams_after_sec", -1.0)
        self.declare_parameter("recover_streams_after_sec", -1.0)

        p = lambda n: self.get_parameter(n).value
        self.me = str(p("agent_id"))
        self.x, self.y = float(p("start_x")), float(p("start_y"))
        _, self.centers = make_sectors(
            int(p("grid_nx")), int(p("grid_ny")),
            float(p("grid_min_x")), float(p("grid_min_y")),
            float(p("cell_w")), float(p("cell_h")))
        self.pits = set(json.loads(str(p("pits_json"))))
        self.pit_rim = float(p("pit_rim"))
        self.fail_after = float(p("fail_streams_after_sec"))
        self.recover_after = float(p("recover_streams_after_sec"))

        self.target = None
        self.t = 0.0

        self.pose_pub = self.create_publisher(PointStamped, "pose", 10)
        self.tel_pub = self.create_publisher(String, "telemetry", 10)
        self.create_subscription(String, "goto", self._on_goto, 10)
        self.create_timer(TICK, self._step)
        self.get_logger().info(
            f"mock_pioneer up: {self.me} at ({self.x:.1f}, {self.y:.1f}) "
            f"pits={sorted(self.pits)}")

    def _on_goto(self, msg):
        d = msg.data
        self.target = None if d in ("HOLD", "STOP") else d

    def _step(self):
        self.t += TICK

        if self.target is not None and self.target in self.centers:
            cx, cy = self.centers[self.target]
            dx, dy = cx - self.x, cy - self.y
            dist = math.hypot(dx, dy)
            # A crater stops the robot at its rim: the horizontal lidar never
            # saw it, the wheels simply cannot continue.
            limit = self.pit_rim if self.target in self.pits else 0.0
            if dist > max(limit, 1e-9):
                step = min(SPEED, dist - limit) if limit else min(SPEED, dist)
                if step > 0:
                    self.x += dx / dist * step
                    self.y += dy / dist * step

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = self.x, self.y
        self.pose_pub.publish(msg)

        stale = (self.fail_after >= 0 and self.t >= self.fail_after
                 and (self.recover_after < 0 or self.t < self.recover_after))
        age = (self.t - self.fail_after) if stale else 0.02
        self.tel_pub.publish(String(data=json.dumps(
            {"lidar": age, "camera": age, "odom": age})))


def main():
    rclpy.init()
    node = MockPioneer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

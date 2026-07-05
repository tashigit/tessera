#!/usr/bin/env python3
"""mock_robot — headless stand-in for waypoint_follower + Webots physics, used by
route_exploration.launch_test.py so the full vertex_node + mission_coordinator
consensus loop can be asserted in CI without Webots (which has no arm64 Linux
build and can't run in the container).

It emulates the *physical outcome* the coordinator reacts to:
  * drive=<route> : advance x toward the goal along that lane. If the route is in
                    `blocked_routes`, stop at the barrier and raise `barrier`
                    (-> coordinator emits Blocked). If open, reach the goal
                    (pose.x > goal -> coordinator emits Arrived).
  * drive=STAGING : return to the staging line, barrier low.
  * drive=STOP    : halt.

Publishes /robot_i/pose (PointStamped) and /robot_i/barrier (Bool); subscribes
/robot_i/drive (String). Geometry mirrors config/routes.yaml.

Test hooks:
  * freeze_at_x (param): stop advancing east past this x WITHOUT raising
    `barrier` — the robot looks alive but makes no progress, the stand-in for
    a crashed/stalled explorer in the fault-injection test.
  * blocked_routes (topic, std_msgs/String, JSON list): replace the blocked
    set at runtime — lets the soak test randomize the world between missions.
"""

import json
import os
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String

ROUTE_LANE_Y = {"R1": 2.25, "R2": 0.75, "R3": -0.75, "R4": -2.25}
STAGING_X, BARRIER_X, GOAL_X = -4.0, 1.5, 3.6
SPEED = 0.4  # metres per 0.1 s tick (~4 m/s) — keeps the test short


class MockRobot(Node):
    def __init__(self):
        super().__init__("mock_robot")
        self.declare_parameter("home_lane_y", 0.0)
        self.declare_parameter("blocked_routes", [""])
        # crash stand-in: stop advancing past this x, barrier stays low
        self.declare_parameter("freeze_at_x", 1e9)
        self.home_y = float(self.get_parameter("home_lane_y").value)
        self.blocked = {r for r in self.get_parameter("blocked_routes").value if r}
        self.freeze_at_x = float(self.get_parameter("freeze_at_x").value)

        self.x = STAGING_X
        self.y = self.home_y
        self.cmd = "STAGING"
        self.barrier = False

        self.pose_pub = self.create_publisher(PointStamped, "pose", 10)
        self.barrier_pub = self.create_publisher(Bool, "barrier", 10)
        self.create_subscription(String, "drive", self._on_drive, 10)
        self.create_subscription(String, "blocked_routes", self._on_blocked, 10)
        self.create_timer(0.1, self._tick)

    def _on_drive(self, msg: String):
        self.cmd = msg.data

    def _on_blocked(self, msg: String):
        try:
            self.blocked = {r for r in json.loads(msg.data) if r}
        except ValueError:
            pass

    def _tick(self):
        cmd = self.cmd
        if cmd in ROUTE_LANE_Y:
            self.y = ROUTE_LANE_Y[cmd]
            if cmd in self.blocked and self.x >= BARRIER_X:
                self.x = BARRIER_X          # stopped at the barrier
                self.barrier = True
            else:
                self.barrier = False
                nxt = min(self.x + SPEED, GOAL_X + 1.0)      # advance (reaches goal if open)
                self.x = min(nxt, max(self.x, self.freeze_at_x))
        elif cmd == "STAGING":
            self.barrier = False
            self.y = self.home_y
            self.x = max(self.x - SPEED, STAGING_X)
        else:                               # STOP / unknown
            self.barrier = False

        ps = PointStamped()
        ps.header.frame_id = "map"
        ps.point.x, ps.point.y = self.x, self.y
        self.pose_pub.publish(ps)
        self.barrier_pub.publish(Bool(data=self.barrier))


def main(args=None):
    rclpy.init(args=args)
    # This is a stateless test fixture: exit immediately on the harness's
    # SIGINT/SIGTERM instead of walking rclpy's teardown, which has a native
    # (segfault-level) shutdown race that can fail a launch_test's clean-exit
    # assertion through no fault of the system under test.
    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
    node = MockRobot()
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
        if rclpy.ok():          # SIGINT teardown may have shut the context already
            rclpy.shutdown()


if __name__ == "__main__":
    main()

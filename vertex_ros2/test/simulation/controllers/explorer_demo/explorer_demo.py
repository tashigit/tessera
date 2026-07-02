"""explorer_demo — a standalone Webots controller so the world is runnable and
demonstrates route blocking *before* the ROS 2 / Vertex stack is wired in.

Behaviour (per robot, no consensus): drive straight down the lane; report
BLOCKED when forward progress stalls (pushed against a raised barrier) and
ARRIVED once past the goal line. Detection is by GPS progress, NOT the distance
sensor — immune to chassis-pitch/bounce artifacts on a fast, light car.

For the real scenario each robot is driven by the ROS 2 `waypoint_follower` +
`mission_coordinator` over rosbridge (see ../../README.md).
"""

from controller import Robot

CRUISE = 16.0        # wheel angular velocity (rad/s) -> ~0.8 m/s
GOAL_X = 3.6         # world x of the goal line
STALL_WINDOW = 2.0   # seconds of no forward progress => blocked
STALL_EPS = 0.03     # metres of progress that counts as "moving"
WHEELS = ("front left wheel motor", "front right wheel motor",
          "rear left wheel motor", "rear right wheel motor")


def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())
    name = robot.getName()

    motors = [robot.getDevice(w) for w in WHEELS]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    gps = robot.getDevice("gps")
    gps.enable(ts)

    def drive(v):
        for m in motors:
            m.setVelocity(v)

    state = "driving"          # driving | blocked | arrived
    best_x, t_improve = -1e9, 0.0

    while robot.step(ts) != -1:
        now = robot.getTime()
        x = gps.getValues()[0]
        if x > best_x + STALL_EPS:      # made forward progress
            best_x, t_improve = x, now

        if state == "arrived":
            drive(0.0)
            continue
        if x > GOAL_X:
            drive(0.0)
            print(f"[{name}] ARRIVED at goal (x={x:.2f})")
            state = "arrived"
            continue

        drive(CRUISE)                   # keep pushing forward (a barrier stops us)
        stalled = (now - t_improve) > STALL_WINDOW
        if stalled and state != "blocked":
            print(f"[{name}] route BLOCKED (no progress) — would publish RouteClosed")
            state = "blocked"
        elif not stalled and state == "blocked":
            print(f"[{name}] route clear again — resuming")
            state = "driving"


if __name__ == "__main__":
    main()

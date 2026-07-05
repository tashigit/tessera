#!/usr/bin/env bash
# Entrypoint for the tessera ROS 2 Jazzy image.
#
# Sources the base ROS install + the ros2-rust overlay, then dispatches on the
# first argument:
#
#   core    cargo-test vertex_core only (real libtashi-vertex, no ROS layer)
#   build   colcon build the vertex_ros2 node (+ its colcon deps)
#   test    core + build + the 3-node launch_test (design §7)
#   soak    the 10-minute RSS soak launch_test (SOAK_SECONDS overrides length)
#   shell   drop into an interactive shell with the environment sourced
#
# Expects the repos bind-mounted (see docker-compose.yml):
#   /ws/src/tessera           this repo
#   /ws/src/tashi-vertex-rs   sibling crate (vertex_core's path dependency)
set -euo pipefail

# ROS setup scripts reference unset variables (AMENT_TRACE_SETUP_FILES, ...) and
# are not `set -u` safe, so disable nounset just while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/ros2_rust/install/setup.bash
set -u

# Isolate this container's ROS 2 DDS discovery to itself. Every node we run
# (vertex_node, mission_coordinator, rosbridge, mock_robot, launch_tests) lives
# in this one container; the Mac-side Webots follower talks to us over the
# rosbridge WebSocket, not DDS. Without this, a leftover or parallel container
# on the same Docker network shares ROS_DOMAIN_ID 0 and its nodes collide with
# ours (duplicate transition services, drive-topic fights) — which manifests as
# "activate not valid from state Active" and bots that never move.
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

TESSERA=/ws/src/tessera
VERTEX_RS=/ws/src/tashi-vertex-rs
export VERTEX_RS

# Optional: link against a locally-mounted tashi-vertex-c instead of letting
# CMake download the release archive (set TASHI_VERTEX_LOCAL_DIR via compose).
if [ -n "${TASHI_VERTEX_LOCAL_DIR:-}" ]; then
  export TASHI_VERTEX_LOCAL_BUILD=1
  echo "==> Using local tashi-vertex-c at ${TASHI_VERTEX_LOCAL_DIR}"
fi

run_core() {
  echo "==> cargo test vertex_core (real libtashi-vertex, no ROS)"
  ( cd "${TESSERA}/vertex_core" && cargo test )
}

build_ros() {
  echo "==> colcon build --packages-up-to vertex_ros2 vertex_fleet"
  cd /ws
  # tashi-vertex-rs lives under src/ as a path dependency, not a colcon package;
  # --packages-up-to builds only the listed packages and their colcon deps.
  colcon build --packages-up-to vertex_ros2 vertex_fleet \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  # colcon's setup.bash is not `set -u` safe (COLCON_TRACE, ...).
  set +u
  source /ws/install/setup.bash
  set -u
}

# The colcon-installed vertex_node links libtashi-vertex.so but has no rpath to
# it on Linux (tashi-vertex-rs's build.rs only sets an rpath on macOS). Locate
# the .so produced during the cargo build and put its directory on
# LD_LIBRARY_PATH; launched node processes inherit it.
export_tv_libpath() {
  local so
  so=$(find /ws/build /ws/install -name 'libtashi-vertex.so' 2>/dev/null | head -n1)
  [ -z "$so" ] && so=$(find /ws -name 'libtashi-vertex.so' 2>/dev/null | head -n1)
  if [ -n "$so" ]; then
    export LD_LIBRARY_PATH="$(dirname "$so")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    echo "==> libtashi-vertex.so: $(dirname "$so") (added to LD_LIBRARY_PATH)"
  else
    echo "WARNING: libtashi-vertex.so not found under /ws — vertex_node will fail to load it"
  fi
}

gen_fixtures() {
  if [ ! -f "${TESSERA}/vertex_ros2/test/fixtures/peers.json" ]; then
    echo "==> generating test/fixtures/peers.json"
    ( cd "${TESSERA}/vertex_ros2/test" && VERTEX_RS="${VERTEX_RS}" bash gen_test_keys.sh )
  fi
}

gen_peers4() {
  if [ ! -f "${TESSERA}/vertex_ros2/test/simulation/fixtures/peers4.json" ]; then
    echo "==> generating simulation fixtures/peers4.json"
    ( VERTEX_RS="${VERTEX_RS}" bash \
        "${TESSERA}/vertex_ros2/test/simulation/fixtures/gen_peers4.sh" )
  fi
}

case "${1:-test}" in
  core)
    run_core
    ;;
  build)
    build_ros
    ;;
  test)
    run_core
    build_ros
    export_tv_libpath
    gen_fixtures
    echo "==> launch_test: three vertex_node processes (system_three_peers)"
    launch_test "${TESSERA}/vertex_ros2/test/system_three_peers.launch_test.py"
    echo "==> vertex_fleet unit tests"
    python3 "${TESSERA}/vertex_fleet/test/test_state.py"
    echo "==> launch_test: vertex_fleet ledger demo (consumer-shaped app on real consensus)"
    launch_test "${TESSERA}/vertex_fleet/test/ledger_demo.launch_test.py"
    ;;
  soak)
    build_ros
    export_tv_libpath
    gen_fixtures
    echo "==> launch_test: soak (SOAK_SECONDS=${SOAK_SECONDS:-600})"
    SOAK_SECONDS="${SOAK_SECONDS:-600}" \
      launch_test "${TESSERA}/vertex_ros2/test/soak.launch_test.py"
    ;;
  sim)
    # Route-exploration simulation, container side. Webots runs NATIVELY on the
    # host (no arm64 Linux build); this serves rosbridge + the Vertex/ROS graph.
    build_ros
    export_tv_libpath
    gen_peers4
    echo "==> sim: rosbridge (9090) + 4x vertex_node + 4x mission_coordinator"
    echo "    On the host: open vertex_ros2/test/simulation/worlds/routes_4bot_ros2.wbt in Webots."
    ros2 launch "${TESSERA}/vertex_ros2/test/simulation/route_exploration.launch.py"
    ;;
  simtest)
    # Headless automated assertions for the route-exploration scenario: real
    # vertex_node consensus + mission_coordinator + mock_robot (no Webots).
    # Runs the full functional suite: N1 (exploration), N3 (fault injection),
    # N4 (lifecycle churn). The mission soak (N5) is the separate `simsoak`.
    build_ros
    export_tv_libpath
    gen_peers4
    echo "==> mission_fsm unit tests"
    python3 "${TESSERA}/vertex_ros2/test/simulation/nodes/test_mission_fsm.py"
    echo "==> launch_test: route exploration (4x vertex_node + coordinator + mock_robot)"
    launch_test "${TESSERA}/vertex_ros2/test/simulation/route_exploration.launch_test.py"
    echo "==> launch_test: fault injection (crash the explorer, lease recovers)"
    launch_test "${TESSERA}/vertex_ros2/test/simulation/fault_injection.launch_test.py"
    echo "==> launch_test: lifecycle churn (deactivate/activate mid-mission)"
    launch_test "${TESSERA}/vertex_ros2/test/simulation/lifecycle_churn.launch_test.py"
    ;;
  simsoak)
    # Randomized back-to-back mission soak (N5). SOAK_SECONDS overrides length.
    build_ros
    export_tv_libpath
    gen_peers4
    echo "==> launch_test: mission soak (SOAK_SECONDS=${SOAK_SECONDS:-120})"
    SOAK_SECONDS="${SOAK_SECONDS:-120}" \
      launch_test "${TESSERA}/vertex_ros2/test/simulation/soak_missions.launch_test.py"
    ;;
  shell)
    exec bash
    ;;
  *)
    exec "$@"
    ;;
esac

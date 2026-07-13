# Dockerized ROS 2 Jazzy environment (incl. Apple Silicon)

Runs the `vertex_ros2` node and its `launch_test`s on a machine without a native
ROS 2 install. Built and tested target: **ROS 2 Jazzy on Ubuntu 24.04**, the same
distro the design acceptance criteria (§7) target.

## Why containerized (and not native macOS)

ROS 2 dropped macOS from its supported tiers; Jazzy targets Ubuntu 24.04. On an
Apple Silicon (M-series) Mac the right move is a Linux container:

- The `ros:jazzy-ros-base` image is **multi-arch**: on an M3 Docker pulls the
  `linux/arm64` variant and runs it **natively** (no x86 emulation, full speed).
- `tashi-vertex-rs`'s CMake downloads the prebuilt `tashi-vertex-c` release and,
  on Linux/arm64, links **`libtashi-vertex-arm64.so`** (shipped since v0.14.0).

[OrbStack](https://orbstack.dev) is a lighter, faster alternative to Docker
Desktop on macOS and works unchanged with these files.

## Layout

| File | Role |
|---|---|
| `docker/Dockerfile` | The environment: ROS Jazzy + Rust + a ros2-rust overlay (`rclrs` + Rust message bindings) + `colcon-cargo`/`colcon-ros-cargo` + CMake ≥ 4.2.3 |
| `docker/extra_interfaces.repos` | Source-builds `common_interfaces` + `rcl_interfaces` so Rust bindings exist for `std_msgs`/`diagnostic_msgs`/`builtin_interfaces` |
| `docker/entrypoint.sh` | Sources the env and dispatches `core` / `build` / `test` / `soak` / `shell` |
| `docker-compose.yml` (repo root) | Bind-mounts the source and defines the run targets |

The image holds only the environment; your repos are **bind-mounted** at run
time, so edits on the host take effect immediately with no image rebuild.

## Prerequisites

- Docker Desktop or OrbStack.
- `tashi-vertex-rs` checked out as a **sibling** of this repo (it is the
  `tashi-vertex` path dependency of `vertex_core`):
  ```
  Work/Tashi/
  ├── tessera/            # this repo
  └── tashi-vertex-rs/    # required sibling
  ```

## Use

From the `tessera` repo root:

```console
docker compose build            # build the env image once (slow: builds ros2-rust)

docker compose run --rm core    # vertex_core cargo test only (fast smoke test)
docker compose run --rm build   # colcon build the vertex_ros2 node
docker compose run --rm test    # core + build + the 3-node launch_test (§7 ordering)
docker compose run --rm -e SOAK_SECONDS=120 soak   # shortened soak
docker compose run --rm shell   # interactive, environment pre-sourced
```

`build`/`test`/`install` artifacts persist in named volumes between runs, so only
the first build pays full cost.

### Optional: link the local tashi-vertex-c

To skip CMake's release download and link the `.so` from your local checkout,
uncomment the `tashi-vertex-c` mount and `TASHI_VERTEX_LOCAL_DIR` env in
`docker-compose.yml`.

## CI

The same image backs CI: a GitHub Actions job on an arm64 (or amd64) runner runs
`docker compose run --rm test` and, on a schedule, `soak`. This is the path that
turns the "system tests pass **in CI** on Jazzy" criterion green.

## Caveats (could not be executed from the authoring environment)

This setup is authored against the documented toolchains; it has **not** yet been
run end-to-end here. The step most likely to need a nudge is the **ros2-rust
overlay**:

- `ros2_rust_jazzy.repos` is fetched from `ros2-rust/ros2_rust`'s default branch.
  If upstream renames/moves that file or its layout, adjust the `vcs import` line
  in the `Dockerfile`.
- If a message package still lacks Rust bindings at `colcon build`, add its repo
  to `extra_interfaces.repos` so the generator runs over it.

Run `docker compose run --rm core` first. It exercises the `tashi-vertex` CMake
fetch and the Rust core without the ROS layer, isolating env problems quickly.

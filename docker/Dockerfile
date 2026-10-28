# syntax=docker/dockerfile:1
#
# tessera / vertex_ros2 — ROS 2 build+test environment (Jazzy by default, Humble
# via `--build-arg ROS_DISTRO=humble`).
#
# Builds a Linux image with everything the colcon/ros2-rust path needs so the
# `vertex_ros2` node and its launch_tests run on a machine without a native ROS
# install — including Apple Silicon (M-series) Macs.
#
#   * Base `ros:<distro>-ros-base` is multi-arch: on an M3 host Docker pulls the
#     arm64 variant and runs it NATIVELY (no x86 emulation).
#   * The `tashi-vertex` crate's CMake fetches the prebuilt `tashi-vertex-c`
#     release and, on Linux/arm64, links `libtashi-vertex-arm64.so` — shipped
#     in v0.14.0.
#
# This image carries only the *environment* (ROS + Rust + ros2-rust overlay +
# colcon plugins + a modern CMake). The project source is bind-mounted at run
# time (see docker-compose.yml) so you can iterate without rebuilding the image.
#
# Build:   docker build -t tessera-ros2:jazzy  -f docker/Dockerfile docker
#          docker build -t tessera-ros2:humble -f docker/Dockerfile \
#                       --build-arg ROS_DISTRO=humble docker
# (the build context is ./docker; it only needs the entrypoint, the project
#  itself is mounted at run time.)
#
# Everything distro-specific below is keyed on ROS_DISTRO: the base image, the
# apt package names, the ros2-rust repos file, and the setup scripts. The two
# supported values are `jazzy` (Ubuntu 24.04) and `humble` (Ubuntu 22.04); the
# ros2-rust overlay pinned below ships a repos file for both, and rclrs selects
# its rcl bindings from the same variable at cargo-build time.

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

# An ARG declared before FROM is out of scope after it. The base image also
# exports ROS_DISTRO as an ENV, which would cover the RUN steps below, but the
# redeclaration keeps the dependency on the build arg explicit.
ARG ROS_DISTRO

ARG DEBIAN_FRONTEND=noninteractive
# pip on Ubuntu 24.04 is PEP-668 "externally managed"; this image is disposable
# so installing into the system environment is acceptable. (Ubuntu 22.04's pip
# predates PEP 668 and ignores the variable.)
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# --- system toolchain --------------------------------------------------------
# clang/libclang: rosidl_generator_rs runs bindgen.
# python3-vcstool: import the ros2-rust source set.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates build-essential pkg-config \
        clang libclang-dev \
        python3-pip python3-vcstool \
        ros-${ROS_DISTRO}-launch-testing \
        ros-${ROS_DISTRO}-launch-testing-ament-cmake \
        ros-${ROS_DISTRO}-rosbridge-suite \
        python3-psutil \
    && rm -rf /var/lib/apt/lists/*

# --- modern CMake (the tashi-vertex build requires >= 4.2.3) -----------------
# Ubuntu 24.04 ships 3.28 and 22.04 ships 3.22, both too old. The pip wheel has
# manylinux aarch64 builds, so this works on Apple Silicon. Installs to
# /usr/local/bin (ahead of /usr/bin).
RUN pip3 install "cmake>=4.2.3" && cmake --version

# --- Rust (tashi-vertex is edition 2024 → needs rustc >= 1.85) ---------------
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain stable --profile minimal \
    && rustc --version \
    && cargo install cargo-ament-build --version 0.1.11

# --- colcon plugins for the Rust build path ---------------------------------
# Neither plugin is released to PyPI, so install from git at fixed commits.
RUN pip3 install \
        git+https://github.com/colcon/colcon-cargo.git@518a4f90d7de33d685cfba32a45fb4cb3aa08950 \
        git+https://github.com/colcon/colcon-ros-cargo.git@dc26ba5f4e7d016a5f8b78abdaa1c22de9d35442

# --- ros2-rust overlay (rclrs + Rust message bindings) -----------------------
# rclrs is not in apt; build it once into /opt/ros2_rust as an overlay. The
# upstream repos file also source-builds common_interfaces + rcl_interfaces, so
# the Rust generator emits bindings for std_msgs / diagnostic_msgs /
# builtin_interfaces (prebuilt apt packages carry no Rust crate). Since
# ros2_rust 0.8 those generated crates reach vertex_ros2 through the `ros-env`
# crate, not as individual Cargo dependencies.
#
# Everything here is pinned. The repos file names only branches for the
# ros2-rust side repos, so the generator and runtime are fixed to commits that
# match the tag. Bump the tag and the two commits together. The per-distro
# repos files (`ros2_rust_jazzy.repos`, `ros2_rust_humble.repos`) differ only
# in which branch of the ROS interface packages they check out; the ros2-rust
# side is the same `main` in both, so the two commit pins apply to either.
ARG ROS2_RUST_TAG=v0.8.0
ARG ROSIDL_RUST_REV=19d57818dab3b51e418c0893b70b3a1a8b64c495
ARG ROSIDL_RUNTIME_RS_REV=21def427689fda0156dd4b0dff28d8d0318af03b
WORKDIR /opt/ros2_rust
RUN git clone --branch "${ROS2_RUST_TAG}" --depth 1 \
        https://github.com/ros2-rust/ros2_rust.git src/ros2_rust \
    && vcs import src < src/ros2_rust/ros2_rust_${ROS_DISTRO}.repos \
    && git -C src/ros2-rust/rosidl_rust checkout --detach "${ROSIDL_RUST_REV}" \
    && git -C src/ros2-rust/rosidl_runtime_rs checkout --detach "${ROSIDL_RUNTIME_RS_REV}"
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
        || echo "rosdep: some keys unresolved (rust deps are expected); continuing"
# The upstream example packages are imported by the repos file but nothing
# here uses them, so leave them out of the overlay build.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --packages-skip-regex '^examples_rclrs_' \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- workspace mount point ---------------------------------------------------
# docker-compose mounts this repo here at run time:
#   /ws/src/tessera          (the only checkout needed; tashi-vertex comes
#                             from crates.io and fetches its own engine)
WORKDIR /ws
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Invoke via bash so a bind-mounted entrypoint (see docker-compose.yml) works
# even when the host's executable bit isn't preserved by the file share.
ENTRYPOINT ["bash", "/usr/local/bin/entrypoint.sh"]
CMD ["test"]

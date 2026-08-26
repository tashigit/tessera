# syntax=docker/dockerfile:1
#
# tessera / vertex_ros2 — ROS 2 Jazzy build+test environment.
#
# Builds a Linux image with everything the colcon/ros2-rust path needs so the
# `vertex_ros2` node and its launch_tests run on a machine without a native ROS
# install — including Apple Silicon (M-series) Macs.
#
#   * Base `ros:jazzy` is multi-arch: on an M3 host Docker pulls the arm64
#     variant and runs it NATIVELY (no x86 emulation).
#   * The `tashi-vertex` crate's CMake fetches the prebuilt `tashi-vertex-c`
#     release and, on Linux/arm64, links `libtashi-vertex-arm64.so` — shipped
#     in v0.14.0.
#
# This image carries only the *environment* (ROS + Rust + ros2-rust overlay +
# colcon plugins + a modern CMake). The project source is bind-mounted at run
# time (see docker-compose.yml) so you can iterate without rebuilding the image.
#
# Build:   docker build -t tessera-ros2:jazzy -f docker/Dockerfile docker
# (the build context is ./docker; it only needs extra_interfaces.repos + the
#  entrypoint, the project itself is mounted at run time.)

FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
# pip on Ubuntu 24.04 is PEP-668 "externally managed"; this image is disposable
# so installing into the system environment is acceptable.
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# --- system toolchain --------------------------------------------------------
# clang/libclang: rosidl_generator_rs runs bindgen.
# python3-vcstool: import the ros2-rust source set.
RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ca-certificates build-essential pkg-config \
        clang libclang-dev \
        python3-pip python3-vcstool \
        ros-jazzy-launch-testing \
        ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-rosbridge-suite \
        python3-psutil \
    && rm -rf /var/lib/apt/lists/*

# --- modern CMake (the tashi-vertex build requires >= 4.2.3) -----------------
# Ubuntu 24.04 ships 3.28, too old. The pip wheel has manylinux aarch64 builds,
# so this works on Apple Silicon. Installs to /usr/local/bin (ahead of /usr/bin).
RUN pip3 install "cmake>=4.2.3" && cmake --version

# --- Rust (tashi-vertex is edition 2024 → needs rustc >= 1.85) ---------------
ENV RUSTUP_HOME=/opt/rust/rustup \
    CARGO_HOME=/opt/rust/cargo \
    PATH=/opt/rust/cargo/bin:$PATH
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain stable --profile minimal \
    && rustc --version \
    && cargo install cargo-ament-build

# --- colcon plugins for the Rust build path ---------------------------------
RUN pip3 install \
        git+https://github.com/colcon/colcon-cargo.git \
        git+https://github.com/colcon/colcon-ros-cargo.git

# --- ros2-rust overlay (rclrs + Rust message bindings) -----------------------
# rclrs is not in apt; build it once into /opt/ros2_rust as an overlay. We also
# source-build common_interfaces + rcl_interfaces so the Rust generator emits
# bindings for std_msgs / diagnostic_msgs / builtin_interfaces (the message
# packages vertex_ros2 depends on) — prebuilt apt packages carry no Rust crate.
WORKDIR /opt/ros2_rust
COPY extra_interfaces.repos /opt/ros2_rust/extra_interfaces.repos
RUN git clone https://github.com/ros2-rust/ros2_rust.git src/ros2_rust \
    && vcs import src < src/ros2_rust/ros2_rust_jazzy.repos \
    && vcs import src < extra_interfaces.repos
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
        || echo "rosdep: some keys unresolved (rust deps are expected); continuing"
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

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

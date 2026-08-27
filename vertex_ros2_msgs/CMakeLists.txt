cmake_minimum_required(VERSION 3.8)
project(vertex_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VertexTransaction.msg"
  "msg/VertexEvent.msg"
  "msg/VertexSyncPoint.msg"
  "srv/VertexStatus.srv"
  "srv/VertexTransition.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Tell colcon-ros-cargo that this package ships a Rust crate.
#
# rosidl_generator_rs generates one into
# share/vertex_ros2_msgs/rust/ during the build above, but colcon-ros-cargo
# 0.2.0 only discovers installed Rust crates through an ament resource index
# entry named `rust_packages` (see find_installed_cargo_packages), and the
# generator does not register one. With no entry, nothing patches
# `vertex_ros2_msgs` into cargo's crates-io namespace and the vertex_ros2
# build dies with "no matching package named `vertex_ros2_msgs` found /
# location searched: crates.io index".
#
# That combination breaks any COLD build, which is every CI run. A warm
# install space hides it completely, which is why it went unnoticed for a
# week of red nightlies while local builds kept passing.
#
# Registering the resource ourselves is deliberate rather than pinning the
# two upstreams into agreement: the Dockerfile clones ros2_rust's default
# branch and installs colcon-ros-cargo from git HEAD, so both move
# independently, and this one line is what the consumer side actually needs.
ament_index_register_resource("rust_packages")

ament_package()

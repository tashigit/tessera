# Releasing the ROS 2 + Vertex integration

How I cut a versioned release of this repository, what is pinned where, and
how consumers upgrade. The consumer-facing view is `CONSUMING.md`.

## What a release is

A release is an annotated git tag `vX.Y.Z` on `main`. Pushing the tag runs the
`release` workflow, which validates the full test suite in the Jazzy harness,
exports the built install space as a self-contained tarball, and publishes a
GitHub Release carrying:

- `vertex-ros2-jazzy-<tag>-amd64.tar.gz`: the colcon install space
  (`vertex_ros2_msgs`, `vertex_ros2`, `vertex_fleet`) with `libtashi-vertex.so`
  copied next to the binaries so the `$ORIGIN`-relative rpath resolves. Extract
  into a workspace and `source install/setup.bash`; no build required
- release notes recording the exact `tashi-vertex-rs` commit and
  `tashi-vertex-c` version the artifacts were built against

## The version pinning chain

| Level | Pinned by | Where |
|---|---|---|
| tessera packages (`vertex_ros2_msgs`, `vertex_ros2`, `vertex_core`, `vertex_fleet`) | the release tag; all four share one version | `package.xml`, `Cargo.toml`, `setup.py` |
| `tashi-vertex-rs` (Rust bindings) | the commit checked out next to tessera; recorded in the release notes at tag time | `tessera.repos` (`version:` field) for consumers |
| `tashi-vertex-c` (the engine) | `TASHI_VERTEX_VERSION` in `tashi-vertex-rs/CMakeLists.txt`; its build downloads that GitHub release archive | upstream repository |

So one tessera tag transitively identifies the exact engine build: tag →
recorded bindings commit → its pinned engine release.

## What the version means

The version tracks the **consumer-facing API**: the `vertex_ros2_msgs`
contract, the `vertex_node` behavior (topics, parameters, lifecycle verbs),
and the `vertex_fleet` library surface. Internal milestones do not move it.
`v0.1.0` is the first release third-party ROS 2 developers can build on.
While in `0.x` the API may still change between minors; `1.0.0` is reserved
for freezing the contract. Confirm the target number with the team before
changing anything; nothing in this repository derives or bumps versions
automatically.

## Cutting a release

1. Set the agreed version in `vertex_ros2_msgs/package.xml`,
   `vertex_ros2/package.xml` + `Cargo.toml`, `vertex_core/Cargo.toml`, and
   `vertex_fleet/package.xml` + `setup.py`. Build once so the `Cargo.lock`
   files pick up the new versions, and commit the locks with the change.
2. Run the full suite: `docker compose run --rm test` and
   `docker compose run --rm sim simtest` must both pass.
3. Tag and push (any `v*` tag triggers the workflow):

```bash
git tag -a vX.Y.Z -m "vX.Y.Z"
git push origin vX.Y.Z
```

4. The `release` workflow does the rest. Check the run, then sanity-check the
   published artifact list on the GitHub Release.

## Upgrading the engine

To move to a new `tashi-vertex-c` release: bump `TASHI_VERTEX_VERSION` in
`tashi-vertex-rs`, land whatever binding changes the new engine needs there,
then in tessera run the full suite against the updated sibling checkout. The
integration's engine-facing assumptions are concentrated on purpose: time-unit
pinning lives in `vertex_core/src/convert.rs` (`nanos_to_time`), FFI
concurrency constraints in `vertex_core/src/engine_task.rs`. If an engine
upgrade changes either contract, those are the two files to revisit. Cut a new
tessera release recording the new bindings commit.

## Consumers pinning a release

`tessera.repos` on `main` tracks `main`. For reproducible builds, consumers
replace both `version:` fields with the values from a release: the tessera
tag, and the `tashi-vertex-rs` commit recorded in that release's notes.

## Distribution beyond source: evaluation

Where this can go next, and my recommendation. Constraint driving everything:
`tessera` and `tashi-vertex-rs` are private, and `tashi-vertex-c` ships as a
private GitHub release. Public channels are out until that changes.

| Channel | Verdict | Notes |
|---|---|---|
| **GitHub Releases with install-space tarballs** | **adopted (this document)** | works today with existing repo access control, zero new infrastructure, covers the "run it without building it" case. amd64 only for now; an arm64 job needs an arm runner (`ubuntu-24.04-arm`) |
| **Container image** (`ghcr.io`, private) | good next step | the Jazzy harness image plus a baked install space gives a runnable `vertex_node` in one pull; natural for fleet deployments already using containers. Adds registry auth management |
| **bloom / apt (rosdistro)** | not viable while private | bloom releases route through the public rosdistro index. A self-hosted private apt repo is possible but is real infrastructure for little gain over tarballs at current consumer count |
| **crates.io for `vertex_core`** | blocked upstream | `vertex_core` depends on `tashi-vertex` by path, and crates.io forbids path/git dependencies. Publishable only if `tashi-vertex-rs` (and effectively the engine license story) goes public first |
| **git dependency for `vertex_core`** | viable with a small change | a consumer Cargo project can depend on `vertex_core` by git URL only if the `tashi-vertex` path dependency is replaced by a git dependency plus a local `[patch]` for workspace development. Worth doing when the first non-ROS Rust consumer shows up, not before |

Recommendation: stay on GitHub Releases now, add the container image when a
deployment asks for it, and revisit apt/crates.io only if the repositories go
public.

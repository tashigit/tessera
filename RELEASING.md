# Releasing the ROS 2 + Vertex integration

How I cut a versioned release of this repository, what is pinned where, and
how consumers upgrade. The consumer-facing view is `CONSUMING.md`.

## What a release is

A release is an annotated git tag `vX.Y.Z` on `main`. Pushing the tag runs the
`release` workflow, which validates the full test suite in the harness for each
supported ROS 2 distribution (Jazzy and Humble) on each supported architecture
(amd64 and arm64), exports each built install space as a self-contained
tarball, and publishes one GitHub Release carrying:

- `vertex-ros2-<distro>-<tag>-<arch>.tar.gz`, four in all (`jazzy` and `humble`,
  each for `amd64` and `arm64`): the colcon install space (`vertex_ros2_msgs`,
  `vertex_ros2`, `vertex_fleet`) for that distribution and architecture, with
  `libtashi-vertex.so` copied next to the binaries so the `$ORIGIN`-relative
  rpath resolves. Extract into a workspace and `source install/setup.bash`; no
  build required
- release notes recording the exact `tashi-vertex` version and
  `tashi-vertex-c` version the artifacts were built against (shared by all
  the tarballs: the pin lives in the tree, not in the distribution or the
  architecture)

One version covers both distributions. The consumer-facing API (messages, node
behaviour, `vertex_fleet`) is identical on Jazzy and Humble; only the ROS
runtime underneath differs. Each arm64 tarball is built and tested on an arm64
runner, not cross-compiled, and bundles the arm64 engine library.

## The version pinning chain

| Level | Pinned by | Where |
|---|---|---|
| tessera packages (`vertex_ros2_msgs`, `vertex_ros2`, `vertex_core`, `vertex_fleet`) | the release tag; all four share one version | `package.xml`, `Cargo.toml`, `setup.py` |
| `tashi-vertex` (Rust bindings) | an exact-version requirement resolved from crates.io, with the checksum in the lockfile | `vertex_core/Cargo.toml` (`=X.Y.Z`), `vertex_core/Cargo.lock` |
| `tashi-vertex-c` (the engine) | `TASHI_VERTEX_VERSION` inside the pinned `tashi-vertex` crate; its build downloads that GitHub release archive | upstream, fixed by the crate version above |

So one tessera tag identifies the exact engine build with nothing recorded out
of band: tag → the `=X.Y.Z` in `vertex_core/Cargo.toml` → the engine release
that crate downloads. Checking out the tag reproduces the build, because the
bindings version is in the tree rather than in whatever happened to be checked
out beside it.

## Changing the pinned bindings version

`vertex_core/Cargo.toml` carries `tashi-vertex = "=X.Y.Z"`. The `=` is
deliberate: a caret requirement would let a `cargo update` silently move the
engine underneath a release. To move versions, edit that requirement, run
`cargo update -p tashi-vertex --precise <new>`, run the full suite, and commit
the lockfile with the change.

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
2. Run the full suite on both distributions: `docker compose run --rm test` and
   `docker compose run --rm sim simtest` must pass as is (Jazzy) and with
   `-f docker-compose.yml -f docker-compose.humble.yml` in front of `run` (Humble).
3. Tag and push (any `v*` tag triggers the workflow):

```bash
git tag -a vX.Y.Z -m "vX.Y.Z"
git push origin vX.Y.Z
```

4. The `release` workflow does the rest: one `build` leg per distribution and
   architecture, then a single `publish` step. Check the run, then sanity-check
   the published artifact list on the GitHub Release (four tarballs).

If one leg fails, use "Re-run failed jobs": the workflow reuses the artifact
from the leg that passed, treats an already-created release as the target, and
uploads with `--clobber`, so a partial run never needs the tag re-pushed.

## Adding a distribution or an architecture to an existing release

The `release` workflow has a manual trigger that builds the chosen
distribution(s) and architecture(s) and attaches the tarball(s) to a release
that already exists. `source_ref` names the tree to build; without it the run
builds the dispatched branch.

An architecture the release was published without is built from the tag
itself, as long as the tag's tree builds with the current workflow. This is
how the arm64 tarballs joined `v0.1.1`:

```bash
gh workflow run release.yml --ref main \
  -f release_tag=v0.1.1 -f distros=both -f arches=arm64 -f source_ref=v0.1.1
```

A release whose tag predates support for a distribution cannot be rebuilt from
the tag for it, so that tarball is built from a later commit:

```bash
gh workflow run release.yml --ref main -f release_tag=v0.1.0 -f distros=humble
```

The dispatched run validates the full suite for each leg, uploads
`vertex-ros2-<distro>-<tag>-<arch>.tar.gz` to the existing release, and appends
a section to its notes naming the commit the tarball was built from and saying
whether that is the tag's tree. Existing assets and notes are left as they
were. `arches` defaults to `amd64`.

`v0.1.0` is the one release whose tag no longer builds: its tree predates the
pinned overlay sources and the crates.io bindings. Its Humble tarball and both
of its arm64 tarballs were built from `e919ab2`, the commit that added Humble.

## Upgrading the engine

A new engine release reaches tessera through a new `tashi-vertex` crate
release: upstream bumps `TASHI_VERTEX_VERSION`, lands whatever binding changes
the engine needs, and publishes. In tessera, bump the pinned requirement as
above and run the full suite. The integration's engine-facing assumptions are
concentrated on purpose: time-unit pinning lives in
`vertex_core/src/convert.rs` (`nanos_to_time`), FFI concurrency constraints in
`vertex_core/src/engine_task.rs`. If an engine upgrade changes either
contract, those are the two files to revisit.

To test against an unpublished bindings change, add a temporary
`[patch.crates-io]` entry pointing `tashi-vertex` at a local checkout. Do not
land it: the pinned crates.io version is what makes a tag reproducible.

## Consumers pinning a release

`tessera.repos` on `main` tracks `main`. For reproducible builds, consumers
replace the `version:` field with a release tag. That one value fixes the
bindings and engine too, since both are pinned inside the tree.

## Distribution beyond source: evaluation

Where this can go next, and my recommendation. The constraint is now narrower
than it used to be: `tessera` itself is private, but the dependencies below it
are not. `tashi-vertex` is on crates.io, and the `tashi-vertex-c` release
archive its build script fetches downloads unauthenticated. So a public
channel is gated on this repo going public, and nothing else.

| Channel | Verdict | Notes |
|---|---|---|
| **GitHub Releases with install-space tarballs** | **adopted (this document)** | works today with existing repo access control, zero new infrastructure, covers the "run it without building it" case, for amd64 and arm64 |
| **Container image** (`ghcr.io`, private) | good next step | the harness image (Jazzy or Humble) plus a baked install space gives a runnable `vertex_node` in one pull; natural for fleet deployments already using containers. Adds registry auth management |
| **bloom / apt (rosdistro)** | not viable while private | bloom releases route through the public rosdistro index. A self-hosted private apt repo is possible but is real infrastructure for little gain over tarballs at current consumer count |
| **crates.io for `vertex_core`** | unblocked, gated on this repo going public | the old blocker was the `tashi-vertex` path dependency, which crates.io forbids. That is now an exact version requirement from the registry, so `vertex_core` is publishable as soon as the repo is public. Worth doing when the first non-ROS Rust consumer shows up |
| **git dependency for `vertex_core`** | works today | a consumer Cargo project can depend on `vertex_core` by git URL as-is. The path dependency that used to make this need a local `[patch]` is gone |

Recommendation: stay on GitHub Releases now, add the container image when a
deployment asks for it, and revisit apt/crates.io only if the repositories go
public.

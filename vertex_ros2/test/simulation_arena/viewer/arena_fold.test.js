"use strict";

// Unit tests for ArenaFold (arena_fold.js) — the viewer's client-side mirror
// of arena_fsm.ArenaState. Runs on any host with plain node, no framework,
// no build step:  node arena_fold.test.js
//
// Mirrors vertex_ros2/test/simulation_arena/nodes/test_arena_fsm.py scenario
// for scenario (same records, same assertions), so a divergence between the
// Python fold and this JS fold shows up here instead of only in a live run.
// role()/my_claim()/claimable_sectors() are ArenaState-only conveniences the
// viewer never needed, so those scenarios are adapted to check phase/claimed
// directly instead.

const { ArenaFold } = require("./arena_fold.js");

const SECTORS = ["S00", "S01", "S02", "S03", "S04", "S05"]; // mirrors make_grid(3, 2, ...)

function freshFleet(n = 5) {
  return Array.from({ length: n }, () => new ArenaFold(SECTORS));
}

function feed(fleet, log) {
  for (const fold of fleet) for (const rec of log) fold.applyRaw(rec);
}

function snap(fold) {
  return JSON.stringify([
    Object.entries(fold.claimed).sort(),
    [...fold.explored].sort(),
    Object.entries(fold.exploredBy).sort(),
    [...fold.unreachable].sort(),
    [...fold.unhealthy].sort(),
    fold.detections.map((d) => [d.bot, d.seq, d.label]),
    fold.phase,
  ]);
}

function assertAgreement(fleet, label) {
  const snaps = fleet.map(snap);
  assert(snaps.every((s) => s === snaps[0]), `${label}: fleet diverged: ${snaps.join(" | ")}`);
}

function R(op, bot = null, sector = null, epoch = 0, extra = {}) {
  const d = { op, epoch, ...extra };
  if (bot !== null) d.bot = bot;
  if (sector !== null) d.sector = sector;
  return d;
}

function assert(cond, msg) {
  if (!cond) throw new Error(msg || "assertion failed");
}
function assertEqual(actual, expected, msg) {
  const a = JSON.stringify(actual), e = JSON.stringify(expected);
  assert(a === e, `${msg || "mismatch"}: got ${a}, want ${e}`);
}

function test_exclusive_claims() {
  const fleet = freshFleet();
  feed(fleet, [
    R("claim", 0, "S00"), R("claim", 1, "S00"), // bot1 loses S00
    R("claim", 1, "S01"), R("claim", 0, "S02"), // bot0 already holds S00
  ]);
  assertAgreement(fleet, "exclusive_claims");
  assertEqual(fleet[0].claimed, { S00: 0, S01: 1 });
  console.log("ok  exclusive_claims");
}

function test_explored_requires_holder() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("explored", 1, "S00")); // not the holder -> no-op
  assertEqual([...fold.explored], []);
  fold.applyRaw(R("claim", 1, "S00"));
  fold.applyRaw(R("explored", 2, "S00")); // someone else -> no-op
  assertEqual([...fold.explored], []);
  fold.applyRaw(R("explored", 1, "S00")); // the holder
  assertEqual([...fold.explored], ["S00"]);
  assertEqual(fold.claimed, {});
  assertEqual(fold.exploredBy, { S00: 1 });
  fold.applyRaw(R("claim", 2, "S00")); // explored: never re-claimed
  assertEqual(fold.claimed, {});
  console.log("ok  explored_requires_holder");
}

function test_abandon_releases_for_others() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("claim", 0, "S01"));
  fold.applyRaw(R("abandon", 0, "S01"));
  assertEqual(fold.claimed, {});
  fold.applyRaw(R("claim", 3, "S01")); // someone else may try
  assertEqual(fold.claimed, { S01: 3 });
  console.log("ok  abandon_releases_for_others");
}

function test_unreachable_requires_holder_and_excludes() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("unreachable", 0, "S01")); // not holding -> no-op
  assertEqual([...fold.unreachable], []);
  fold.applyRaw(R("claim", 0, "S01"));
  fold.applyRaw(R("unreachable", 0, "S01"));
  assertEqual([...fold.unreachable], ["S01"]);
  assertEqual(fold.claimed, {});
  fold.applyRaw(R("claim", 2, "S01")); // condemned: not claimable
  assertEqual(fold.claimed, {});
  console.log("ok  unreachable_requires_holder_and_excludes");
}

function test_health_beacon_marks_and_readmits() {
  const fleet = freshFleet();
  feed(fleet, [
    R("claim", 4, "S02"),
    R("health", 4, null, 0, { seq: 0, ok: true }),
    R("health", 4, null, 0, { seq: 1, ok: false }), // unhealthy: claim released
  ]);
  assertAgreement(fleet, "health_mark");
  assertEqual([...fleet[0].unhealthy], [4]);
  assertEqual(fleet[0].claimed, {});
  feed(fleet, [R("claim", 4, "S02")]); // ignored while unhealthy
  assertEqual(fleet[0].claimed, {});
  feed(fleet, [
    R("health", 4, null, 0, { seq: 2, ok: true }), // readmitted
    R("claim", 4, "S02"),
  ]);
  assertAgreement(fleet, "health_readmit");
  assertEqual([...fleet[0].unhealthy], []);
  assertEqual(fleet[0].claimed, { S02: 4 });
  console.log("ok  health_beacon_marks_and_readmits");
}

function test_health_stale_seq_ignored() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("health", 2, null, 0, { seq: 5, ok: true }));
  fold.applyRaw(R("health", 2, null, 0, { seq: 4, ok: false })); // stale -> no-op
  assertEqual([...fold.unhealthy], []);
  assertEqual(fold.healthSeq[2], 5);
  fold.applyRaw(R("health", 2, null, 0, { seq: 5, ok: false })); // duplicate seq -> no-op
  assertEqual([...fold.unhealthy], []);
  console.log("ok  health_stale_seq_ignored");
}

function test_suspect_silence_lease() {
  const fleet = freshFleet();
  feed(fleet, [
    R("health", 2, null, 0, { seq: 7, ok: true }),
    R("claim", 2, "S03"),
    R("suspect", 0, null, 0, { victim: 2, seen_seq: 7 }), // silent since seq 7 -> out
  ]);
  assertAgreement(fleet, "suspect");
  assertEqual([...fleet[0].unhealthy], [2]);
  assertEqual(fleet[0].claimed, {});
  feed(fleet, [R("suspect", 1, null, 0, { victim: 2, seen_seq: 7 })]); // duplicate -> no-op
  assertEqual([...fleet[0].unhealthy], [2]);
  console.log("ok  suspect_silence_lease");
}

function test_late_beacon_voids_suspicion() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("health", 2, null, 0, { seq: 7, ok: true }));
  fold.applyRaw(R("health", 2, null, 0, { seq: 8, ok: true })); // beacon lands first
  fold.applyRaw(R("suspect", 0, null, 0, { victim: 2, seen_seq: 7 })); // stale -> no-op
  assertEqual([...fold.unhealthy], []);
  console.log("ok  late_beacon_voids_suspicion");
}

function test_detection_gated_on_health() {
  const fleet = freshFleet();
  feed(fleet, [
    R("health", 1, null, 0, { seq: 0, ok: true }),
    R("detection", 1, null, 0, { seq: 0, label: "deer", x: -14.6, y: 9.6 }), // accepted
    R("health", 4, null, 0, { seq: 0, ok: false }),
    R("detection", 4, null, 0, { seq: 0, label: "deer", x: 11.8, y: 5.0 }), // rejected
    R("detection", 1, null, 0, { seq: 0, label: "deer", x: -14.6, y: 9.6 }), // dup -> no-op
  ]);
  assertAgreement(fleet, "detections");
  assertEqual(
    fleet[0].detections.map((d) => [d.bot, d.label]),
    [[1, "deer"]]
  );
  console.log("ok  detection_gated_on_health");
}

function test_mission_completes() {
  const fleet = freshFleet();
  const log = [];
  for (let i = 0; i < SECTORS.length - 1; i++) {
    const bot = i % 5;
    log.push(R("claim", bot, SECTORS[i]), R("explored", bot, SECTORS[i]));
  }
  log.push(R("claim", 0, SECTORS[SECTORS.length - 1]), R("unreachable", 0, SECTORS[SECTORS.length - 1]));
  feed(fleet, log);
  assertAgreement(fleet, "completion");
  assertEqual(fleet[0].phase, "done");
  feed(fleet, [R("claim", 2, "S00")]); // done: claims ignored...
  assertEqual(fleet[0].claimed, {});
  console.log("ok  mission_completes");
}

function test_reset_epoch() {
  const fold = new ArenaFold(SECTORS);
  fold.applyRaw(R("claim", 0, "S00"));
  fold.applyRaw(R("explored", 0, "S00"));
  fold.applyRaw(R("health", 4, null, 0, { seq: 3, ok: false }));
  fold.applyRaw({ op: "reset", epoch: 1 });
  assertEqual(fold.epoch, 1);
  assertEqual([...fold.explored], []);
  assertEqual([...fold.unhealthy], []);
  assertEqual(fold.exploredBy, {});
  fold.applyRaw(R("claim", 0, "S00", 0)); // stale epoch -> ignored
  assertEqual(fold.claimed, {});
  fold.applyRaw(R("claim", 0, "S00", 1));
  assertEqual(fold.claimed, { S00: 0 });
  console.log("ok  reset_epoch");
}

function test_same_log_same_state_shuffled_sources() {
  // the log interleaves records from every bot; all replicas agree at every
  // prefix, not just at the end
  const log = [
    R("health", 0, null, 0, { seq: 0, ok: true }), R("claim", 0, "S00"),
    R("health", 1, null, 0, { seq: 0, ok: true }), R("claim", 1, "S00"),
    R("claim", 1, "S01"), R("explored", 0, "S00"),
    R("health", 1, null, 0, { seq: 1, ok: false }), R("claim", 2, "S01"),
    R("claim", 2, "S02"), // ignored: bot2 already holds S01
    R("detection", 2, null, 0, { seq: 0, label: "rock", x: 9.1, y: 7.1 }),
    R("suspect", 0, null, 0, { victim: 3, seen_seq: -1 }),
    R("health", 1, null, 0, { seq: 2, ok: true }), R("claim", 1, "S01"), // S01 taken by 2
  ];
  const fleet = freshFleet();
  for (let k = 0; k < log.length; k++) {
    feed(fleet, [log[k]]);
    assertAgreement(fleet, `prefix ${k}`);
  }
  assertEqual(fleet[0].claimed, { S01: 2 });
  assertEqual([...fleet[0].unhealthy], [3]); // never beaconed: suspect held
  console.log("ok  same_log_same_state_shuffled_sources");
}

function main() {
  const tests = [
    test_exclusive_claims,
    test_explored_requires_holder,
    test_abandon_releases_for_others,
    test_unreachable_requires_holder_and_excludes,
    test_health_beacon_marks_and_readmits,
    test_health_stale_seq_ignored,
    test_suspect_silence_lease,
    test_late_beacon_voids_suspicion,
    test_detection_gated_on_health,
    test_mission_completes,
    test_reset_epoch,
    test_same_log_same_state_shuffled_sources,
  ];
  for (const t of tests) t();
  console.log(`\nall ${tests.length} arena_fold tests passed`);
}

try {
  main();
} catch (e) {
  console.log(`FAIL: ${e.message}`);
  process.exit(1);
}

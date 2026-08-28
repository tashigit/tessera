"use strict";
// Pin the viewer's fold to the same contract the Python and Rust folds obey.
//
//   node airground_fold.test.js
//
// The air/ground fold exists three times: Python for the bots, Rust for the
// drones, JavaScript here so the viewer can decode /vertex/event itself rather
// than trust a summary. Three copies of a rule is how a protocol drifts, so
// all three replay fixtures/conformance.json and check the canonical snapshot
// after every single record. A divergence fails at the record that caused it.

const fs = require("fs");
const path = require("path");
const { AirGroundFold, makeSectors, makeBlocks } = require("./airground_fold.js");

// The fixture is written with sorted keys (Python's json.dump(sort_keys=True)),
// so compare canonically rather than by raw JSON text: identical state must
// not fail on key order alone.
function canon(v) {
  if (Array.isArray(v)) return "[" + v.map(canon).join(",") + "]";
  if (v && typeof v === "object") {
    return "{" + Object.keys(v).sort().map((k) => JSON.stringify(k) + ":" + canon(v[k])).join(",") + "}";
  }
  return JSON.stringify(v);
}

let failures = 0;
function ok(name) { console.log("ok  " + name); }
function fail(name, detail) { failures++; console.log("FAIL " + name + "\n  " + detail); }

const FIXTURE = path.join(__dirname, "..", "fixtures", "conformance.json");
if (!fs.existsSync(FIXTURE)) {
  console.error(`missing ${FIXTURE} — run: python3 fixtures/gen_conformance.py`);
  process.exit(2);
}
const doc = JSON.parse(fs.readFileSync(FIXTURE, "utf8"));

// ---- the contract ----
(function conformance() {
  const fold = new AirGroundFold(doc.sectors, doc.blocks, doc.block_cells);
  const got0 = canon(fold.snapshot());
  const want0 = canon(doc.snapshots[0]);
  if (got0 !== want0) {
    return fail("conformance/initial", `got  ${got0}\n  want ${want0}`);
  }
  for (let i = 0; i < doc.log.length; i++) {
    fold.apply(doc.log[i]);
    const got = canon(fold.snapshot());
    const want = canon(doc.snapshots[i + 1]);
    if (got !== want) {
      return fail("conformance",
        `snapshot ${i + 1} differs after ${JSON.stringify(doc.log[i])}\n` +
        `  got  ${got}\n  want ${want}`);
    }
  }
  ok(`conformance (${doc.log.length} records, snapshot checked after each)`);
})();

// ---- geometry, also duplicated across the three languages ----
(function geometry() {
  const g = doc.geometry;
  const { ids: sectors } = makeSectors(g.nx, g.ny, g.min_x, g.min_y, g.cell_w, g.cell_h);
  const { ids: blocks, cells } = makeBlocks(g.nx, g.ny, g.block_w, g.block_h);
  if (JSON.stringify(sectors) !== JSON.stringify(doc.sectors)) {
    return fail("geometry/sectors", JSON.stringify(sectors));
  }
  if (JSON.stringify(blocks) !== JSON.stringify(doc.blocks)) {
    return fail("geometry/blocks", JSON.stringify(blocks));
  }
  for (const b of blocks) {
    if (JSON.stringify(cells[b]) !== JSON.stringify(doc.block_cells[b])) {
      return fail("geometry/cells", `${b}: ${JSON.stringify(cells[b])}`);
    }
  }
  ok("geometry matches the fixture");
})();

// ---- a couple of viewer-specific behaviours the fixture does not cover ----
(function acceptedFlag() {
  // The feed greys out records the fold rejected, so `accepted` has to mean
  // "this actually changed the shared state", not "it parsed".
  const { ids: sectors } = makeSectors(4, 3, -20, -15, 10, 10);
  const { ids: blocks, cells } = makeBlocks(4, 3, 2, 1);
  const f = new AirGroundFold(sectors, blocks, cells);
  const won = f.apply({ op: "survey_claim", agent: "drone_0", block: "B00", epoch: 0 });
  const lost = f.apply({ op: "survey_claim", agent: "drone_1", block: "B00", epoch: 0 });
  if (!won.accepted) return fail("accepted/won", "first claim was not accepted");
  if (lost.accepted) return fail("accepted/lost", "losing claim reported as accepted");
  const early = f.apply({ op: "claim", agent: "bot_0", sector: "S00", epoch: 0 });
  if (early.accepted) return fail("accepted/gate", "claim before survey was accepted");
  ok("accepted flag reflects real state change");
})();

(function provisional() {
  const { ids: sectors } = makeSectors(4, 3, -20, -15, 10, 10);
  const { ids: blocks, cells } = makeBlocks(4, 3, 2, 1);
  const f = new AirGroundFold(sectors, blocks, cells);
  f.apply({ op: "hazard", agent: "drone_0", cell: "S05", epoch: 0 });
  if (JSON.stringify(f.provisionalHazards()) !== JSON.stringify(["S05"])) {
    return fail("provisional/one", JSON.stringify(f.provisionalHazards()));
  }
  if (f.unreachable.size !== 0) return fail("provisional/one", "one witness condemned a sector");
  f.apply({ op: "corroborate", agent: "bot_1", cell: "S05", epoch: 0 });
  if (!f.confirmedHazards.has("S05")) return fail("provisional/two", "two witnesses did not confirm");
  if (f.provisionalHazards().length !== 0) return fail("provisional/two", "still provisional");
  ok("hazards: one witness provisional, two distinct witnesses confirm");
})();

console.log(failures ? `\n${failures} failure(s)` : "\nall airground_fold tests passed");
process.exit(failures ? 1 : 0);

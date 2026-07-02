#!/usr/bin/env bash
# Generate FOUR Vertex keypairs + bind addresses for the route-exploration
# simulation, writing them to simulation/fixtures/peers4.json.
#
# Mirrors ../../gen_test_keys.sh (which is fixed at 3 peers) but emits 4 — the
# minimum fleet size for the scenario (n >= 3f+1, f=1). Uses the `key-generate`
# example from tashi-vertex-rs; point VERTEX_RS at that checkout.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_RS="${VERTEX_RS:-$here/../../../../tashi-vertex-rs}"
OUT="$here/peers4.json"
BASE_PORT="${BASE_PORT:-47611}"   # distinct from the 3-peer test (47511)

emit() {
  ( cd "$VERTEX_RS" && cargo run --quiet --example key-generate ) \
    | awk '/^Secret:/{s=$2} /^Public:/{p=$2} END{print s, p}'
}

{
  echo "["
  for i in 0 1 2 3; do
    read -r secret public < <(emit)
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 3 ] && sep=""
    printf '  {"secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$OUT"

echo "wrote $OUT"

#!/usr/bin/env bash
# Generate FOUR Vertex keypairs + bind addresses for the route-exploration
# simulation, writing them to simulation/fixtures/peers4.json.
#
# Mirrors ../../gen_test_keys.sh (which is fixed at 3 peers) but emits 4 — the
# minimum fleet size for the scenario (n >= 3f+1, f=1). Uses vertex_core's
# `key-generate` example, so nothing outside the tessera checkout is needed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_CORE="${VERTEX_CORE:-$here/../../../../vertex_core}"
OUT="$here/peers4.json"
BASE_PORT="${BASE_PORT:-47611}"   # distinct from the 3-peer test (47511)

# prints "<secret_b58> <public_b58>", or fails loudly. A silent failure here
# used to yield a well-formed peers.json full of empty keys, which only shows
# up much later as an unexplained engine start failure.
emit() {
  local out secret public
  if ! out=$( cd "$VERTEX_CORE" && cargo run --quiet --example key-generate ); then
    echo "error: key-generate failed in $VERTEX_CORE" >&2
    return 1
  fi
  secret=$(printf '%s\n' "$out" | sed -n 's/^Secret: //p')
  public=$(printf '%s\n' "$out" | sed -n 's/^Public: //p')
  if [ -z "$secret" ] || [ -z "$public" ]; then
    echo "error: key-generate printed no keypair" >&2
    return 1
  fi
  printf '%s %s\n' "$secret" "$public"
}

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

{
  echo "["
  for i in 0 1 2 3; do
    read -r secret public < <(emit) \
      || { echo "aborting: no keypair for peer $i" >&2; exit 1; }
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 3 ] && sep=""
    printf '  {"secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$tmp"

mv "$tmp" "$OUT"

echo "wrote $OUT"

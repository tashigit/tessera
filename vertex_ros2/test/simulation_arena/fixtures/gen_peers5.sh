#!/usr/bin/env bash
# Generate FIVE Vertex keypairs + bind addresses for the arena-exploration
# simulation, writing them to simulation_arena/fixtures/peers5.json.
#
# Mirrors gen_peers4.sh (route exploration) but emits 5 — the pioneer arena
# fleet size (n = 5 still tolerates f = 1 under n >= 3f+1). Uses vertex_core's
# `key-generate` example, so nothing outside the tessera checkout is needed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_CORE="${VERTEX_CORE:-$here/../../../../vertex_core}"
OUT="$here/peers5.json"
BASE_PORT="${BASE_PORT:-47621}"   # distinct from route exploration (47611)

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
  for i in 0 1 2 3 4; do
    read -r secret public < <(emit) \
      || { echo "aborting: no keypair for peer $i" >&2; exit 1; }
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 4 ] && sep=""
    printf '  {"secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$tmp"

mv "$tmp" "$OUT"

echo "wrote $OUT"

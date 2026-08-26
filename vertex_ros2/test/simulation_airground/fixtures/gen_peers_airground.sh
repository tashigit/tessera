#!/usr/bin/env bash
# Generate FOUR Vertex keypairs + bind addresses for the air/ground
# simulation, writing them to simulation_airground/fixtures/peers_airground.json.
#
# The committee is mixed: peers 0 and 1 are the ground bots (tessera
# vertex_node processes) and peers 2 and 3 are the drones (native Rust
# air_agent binaries). Nothing in the key material distinguishes them, which
# is the point: the engine does not know or care which tier a peer is on.
# n = 4 tolerates f = 1 under n >= 3f+1.
#
# Uses vertex_core's `key-generate` example, so nothing outside the tessera
# checkout is needed.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_CORE="${VERTEX_CORE:-$here/../../../../vertex_core}"
OUT="$here/peers_airground.json"
BASE_PORT="${BASE_PORT:-47631}"   # clear of route (47611) and arena (47621)

# prints "<secret_b58> <public_b58>", or fails loudly. A silent failure here
# used to yield a well-formed peers file full of empty keys, which only shows
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

names=(bot_0 bot_1 drone_0 drone_1)
tiers=(ground ground air air)

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

{
  echo "["
  for i in 0 1 2 3; do
    read -r secret public < <(emit) \
      || { echo "aborting: no keypair for peer $i" >&2; exit 1; }
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 3 ] && sep=""
    printf '  {"name": "%s", "tier": "%s", "secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "${names[$i]}" "${tiers[$i]}" "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$tmp"

mv "$tmp" "$OUT"

echo "wrote $OUT"

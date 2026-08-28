#!/usr/bin/env bash
# Generate three Vertex keypairs + bind addresses for the multi-node launch
# tests, writing them to test/fixtures/peers.json.
#
# Uses vertex_core's `key-generate` example, so this needs nothing outside the
# tessera checkout (the tashi-vertex crate is pinned from crates.io).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_CORE="${VERTEX_CORE:-$here/../../vertex_core}"
OUT="$here/fixtures/peers.json"
BASE_PORT="${BASE_PORT:-47511}"

mkdir -p "$here/fixtures"

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
  for i in 0 1 2; do
    read -r secret public < <(emit) \
      || { echo "aborting: no keypair for peer $i" >&2; exit 1; }
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 2 ] && sep=""
    printf '  {"secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$tmp"

mv "$tmp" "$OUT"

echo "wrote $OUT"

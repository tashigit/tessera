#!/usr/bin/env bash
# Generate three Vertex keypairs + bind addresses for the multi-node launch
# tests, writing them to test/fixtures/peers.json.
#
# Uses the `key-generate` example from tashi-vertex-rs. Point VERTEX_RS at that
# checkout (defaults to a sibling of the tessera workspace).
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERTEX_RS="${VERTEX_RS:-$here/../../../tashi-vertex-rs}"
OUT="$here/fixtures/peers.json"
BASE_PORT="${BASE_PORT:-47511}"

mkdir -p "$here/fixtures"

emit() {
  # prints "<secret_b58> <public_b58>"
  ( cd "$VERTEX_RS" && cargo run --quiet --example key-generate ) \
    | awk '/^Secret:/{s=$2} /^Public:/{p=$2} END{print s, p}'
}

{
  echo "["
  for i in 0 1 2; do
    read -r secret public < <(emit)
    port=$((BASE_PORT + i))
    sep=,; [ "$i" -eq 2 ] && sep=""
    printf '  {"secret": "%s", "public": "%s", "addr": "127.0.0.1:%s"}%s\n' \
      "$secret" "$public" "$port" "$sep"
  done
  echo "]"
} > "$OUT"

echo "wrote $OUT"

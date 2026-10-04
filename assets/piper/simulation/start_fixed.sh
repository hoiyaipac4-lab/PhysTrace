#!/usr/bin/env bash
set -euo pipefail
SUITE_DIR="$(cd -- "$(dirname -- "$0")/.." && pwd)"
exec bash "$SUITE_DIR/scripts/launch.sh" "$@"

#!/usr/bin/env bash
set -euo pipefail
SUITE_DIR="$(cd -- "$(dirname -- "$0")/.." && pwd)"
export PHYSTRACE_LEGACY_ENTRY=1
exec bash "$SUITE_DIR/scripts/launch.sh" "$@"

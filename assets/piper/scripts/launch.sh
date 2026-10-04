#!/usr/bin/env bash
# Portable environment selection only; simulation/control code remains unchanged.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
help() {
  cat <<'USAGE'
Usage: bash scripts/launch.sh [--validate-training] [original Python arguments]

Choose one:
  export ISAAC_PYTHON=../isaac-env/bin/python
  export ISAAC_SIM_DIR=../IsaacSim
Or initialize an Isaac Sim environment first and set ISAAC_USE_ACTIVE_PYTHON=1.
For Conda installations, activate Conda and source Isaac Sim's setup script first.

After reading and accepting NVIDIA's applicable license terms, explicitly set:
  export OMNI_KIT_ACCEPT_EULA=YES

Examples:
  bash scripts/launch.sh --config configs/local.example.json
  bash scripts/launch.sh --validate --seconds 10 --config configs/local.example.json
  bash scripts/launch.sh --validate-training --repeats 2 --seconds 3

New outputs are placed in outputs/<timestamp-pid>/ by default.
Physics defaults remain in simulation/training_config.json.
USAGE
}
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then help; exit 0; fi
entry=run_fixed.py
if [[ "${PHYSTRACE_LEGACY_ENTRY:-0}" == 1 ]]; then entry=run_scene.py; fi
if [[ "${1:-}" == "--validate-training" ]]; then entry=validate_training.py; shift; fi
if [[ "${OMNI_KIT_ACCEPT_EULA:-}" != YES ]]; then
  echo 'Read and accept the applicable NVIDIA terms, then set OMNI_KIT_ACCEPT_EULA=YES.' >&2
  exit 2
fi
if [[ -n "${ISAAC_PYTHON:-}" ]]; then
  runner="$ISAAC_PYTHON"
elif [[ -n "${ISAAC_SIM_DIR:-}" && -x "$ISAAC_SIM_DIR/python.sh" ]]; then
  runner="$ISAAC_SIM_DIR/python.sh"
elif [[ "${ISAAC_USE_ACTIVE_PYTHON:-}" == 1 ]] && command -v python >/dev/null 2>&1; then
  runner="$(command -v python)"
else
  echo 'Set ISAAC_PYTHON, ISAAC_SIM_DIR or ISAAC_USE_ACTIVE_PYTHON. See --help.' >&2; exit 2
fi
[[ -x "$runner" ]] || { echo "Python launcher is not executable: $runner" >&2; exit 2; }
command -v flock >/dev/null 2>&1 || { echo 'Linux flock is required (util-linux).' >&2; exit 2; }
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
out="$ROOT/outputs/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$out"
exec 9>"$ROOT/simulation/.runtime.lock"
flock -n 9 || { echo 'This checkout already has an active scene process.' >&2; exit 2; }
# User arguments come last, so an explicit --output-dir overrides this location.
"$runner" -u "$ROOT/simulation/$entry" --output-dir "$out" "$@" 2>&1 | tee "$out/run.log"
if grep -Eq 'Unable to create triangle mesh|Out of GPU memory|ERROR_OUT_OF_DEVICE_MEMORY|Non-finite physics|Invalid PhysX transform|RUN_FAILED|TRAINING_INTERFACE_FAILED|Traceback' "$out/run.log"; then
  echo "Validation rejected: inspect $out/run.log" >&2; exit 3
fi
echo "Process exited without the monitored critical errors. Log: $out/run.log"

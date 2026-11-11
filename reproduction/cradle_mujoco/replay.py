"""Replay the original MuJoCo cradle cases and compare native readbacks."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import runpy
import subprocess
import sys

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parent
first_peak = runpy.run_path(str(ROOT.parent / "cradle_thresholds/recompute.py"))["first_peak"]


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replay(case_id, reference, archived, destination):
    case = ROOT / "cases" / f"{case_id}.json"
    observer = ROOT / "native_cradle_observer.py"
    reference_scene = ROOT / "reference_scenes" / f"{case_id}.xml"
    assert sha(case) == reference["case_sha256"], case_id
    assert sha(reference_scene) == reference["scene_sha256"], case_id
    target = destination / case_id
    target.mkdir()
    command = [
        sys.executable, "-B", str(observer), "--engine", "mujoco",
        "--case", str(case), "--out", str(target),
    ]
    with (target / "runtime.log").open("wb") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=120)
    result = read(target / "result.json")
    states = read(target / "states.json")
    assert result["status"] == "completed"
    assert result["case"] == read(case)
    assert len(states) - 1 == result["observed_steps"] == reference["observed_steps"]
    for field in ("native_dt", "native_masses", "contact"):
        assert result[field] == reference[field], (case_id, field)
    assert sha(target / "scene.xml") == reference["scene_sha256"], case_id
    times = np.asarray([row["t"] for row in states])
    positions = np.asarray([row["position"] for row in states])
    assert np.isfinite(positions).all()
    assert np.max(np.abs(np.diff(times) - result["native_dt"])) < 1e-10
    geo = result["geometry"]
    pivots = np.asarray(geo["pivots"])
    length, radius, mass = (geo[k] for k in ("length_m", "radius_m", "mass_kg"))
    motion, (peak_time, peak_d) = first_peak(
        times, (positions[:, 0, 0] - pivots[0, 0]) / (2 * radius)
    )
    rod_mass = result["rod"]["mass_kg"]
    rod_z = (length + radius) / 2
    angles = np.deg2rad(result["case"]["angles_deg"])
    initial = 9.80665 * (mass * length + rod_mass * (length - rod_z)) * sum(1 - np.cos(angles))
    potential = 9.80665 * (mass + rod_mass * (1 - rod_z / length)) * np.sum(
        positions[:, :, 2] - (pivots[:, 2] - length)[None, :], axis=1
    )
    energy = float(np.max(potential) / initial)
    assert np.isfinite(energy) and initial > 0
    for label, value in (("first_peak_D", peak_d), ("first_peak_s", peak_time),
                         ("outgoing_time_s", motion), ("energy_ratio", energy)):
        assert abs(value - float(archived[label])) <= 1e-9, (case_id, label)
    row = {
        "condition": case_id, "steps": len(states) - 1,
        "scene_sha256": sha(target / "scene.xml"),
        "first_peak_D": peak_d, "first_peak_s": peak_time,
        "outgoing_time_s": motion, "energy_ratio": energy,
        "peak_difference_D_vs_3_10_0": peak_d - float(archived["archived_first_peak_D"]),
        "energy_difference_vs_3_10_0": energy - float(archived["archived_energy_ratio"]),
        "motion_difference_s_vs_3_10_0": motion - float(archived["archived_outgoing_time_s"]),
    }
    print(json.dumps(row), flush=True)
    return row


def main():
    protocol = read(ROOT / "archived_configuration.json")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", choices=protocol["cases"], help="Repeat to select multiple cases")
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory for generated traces")
    args = parser.parse_args()
    if mujoco.__version__ != "3.13.0":
        parser.error("Install the pinned MuJoCo runtime from requirements.txt.")
    case_ids = args.case or list(protocol["cases"])
    assert len(case_ids) == len(set(case_ids))
    assert sha(ROOT / "native_cradle_observer.py") == protocol["observer_sha256"]
    with (ROOT / "expected_metrics.csv").open(newline="", encoding="utf-8") as stream:
        archived = {row["condition"]: row for row in csv.DictReader(stream)}
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    records = [replay(case, protocol["cases"][case], archived[case], destination) for case in case_ids]
    summary = {
        "runtime": mujoco.__version__, "cases_completed": len(records),
        "physics_steps": sum(row["steps"] for row in records),
        "scene_and_readback_checks_passed": True, "results": records,
    }
    (destination / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

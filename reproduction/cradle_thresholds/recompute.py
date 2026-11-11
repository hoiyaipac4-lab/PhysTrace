"""Recompute two Figure 4 columns and the Bullet gap control from saved arrays."""
from pathlib import Path
from datetime import datetime
import argparse
import csv
import hashlib
import json
import math

import numpy as np

PLATFORMS = ["mujoco", "pybullet", "gazebo", "drake", "isaac", "newton"]
MULTIPLIERS = [0.5, 0.75, 1, 1.25, 1.5, 2]


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def verify_inputs(root, manifest):
    for item in manifest["files"]:
        path = root / item["relative_path"]
        assert path.is_file(), f"Missing input: {item['relative_path']}"
        assert sha(path.read_bytes()) == item["sha256"], item["relative_path"]
        if item["kind"] == "derived_lossless_numeric_subset":
            with np.load(path, allow_pickle=False) as data:
                for member in item["arrays"]:
                    values = data[member["output_member"]]
                    assert list(values.shape) == member["shape"]
                    assert str(values.dtype) == member["dtype"]
                    assert sha(values.tobytes(order="C")) == member["array_sha256_C_order"]


def load_npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {name: data[name].copy() for name in data.files}


def first_peak(times, left):
    indices = np.flatnonzero(left < -0.18)
    assert len(indices)
    event = float(times[indices[0]])
    peaks = []
    for index in range(1, len(times) - 1):
        if not (times[index] > event and left[index] < -0.05
                and left[index] <= left[index - 1] and left[index] < left[index + 1]):
            continue
        if peaks and times[index] - peaks[-1][0] < 0.35:
            if left[index] >= -peaks[-1][1]:
                continue
            peaks.pop()
        peaks.append([float(times[index]), float(-left[index])])
    assert peaks
    return event, peaks[0]


def threshold_recompute(root):
    original = read_json(root / "source_json/original_threshold_conditions.json")["cradle"]
    rows, decisions = [], []
    for record in original:
        platform, case = record["platform"], record["condition"]
        arrays = load_npz(root / "numeric/thin" / f"{platform}_{case}.npz")
        event, peak = first_peak(arrays["t"], arrays["left_x_D"])
        energy = float(np.max(arrays["U_over_E0"]))
        assert peak[1] == record["sim_peak_D"], (platform, case, "peak")
        assert energy == record["energy_ratio"], (platform, case, "energy")
        error = abs(peak[1] / record["real_peak_D"] - 1)
        rows.append({"platform": platform, "condition": case,
                     "first_peak_D": peak[1], "max_potential_over_initial_energy": energy,
                     "real_peak_D": record["real_peak_D"],
                     "first_peak_absolute_relative_error": error,
                     "first_outgoing_motion_s": event, "first_peak_s": peak[0]})
        for multiplier in MULTIPLIERS:
            for metric, value, limit in [
                ("first_peak_absolute_relative_error", error, 0.1 * multiplier),
                ("max_potential_over_initial_energy", energy, 1 + 0.05 * multiplier),
            ]:
                decisions.append({"platform": platform, "condition": case, "metric": metric,
                                  "multiplier": multiplier, "value": value, "threshold": limit,
                                  "passes": int(value <= limit)})
    counts = []
    for metric in sorted({row["metric"] for row in decisions}):
        for multiplier in MULTIPLIERS:
            for platform in PLATFORMS:
                subset = [row for row in decisions if row["metric"] == metric
                          and row["multiplier"] == multiplier and row["platform"] == platform]
                assert len(subset) == 12
                passed = sum(row["passes"] for row in subset)
                counts.append({"platform": platform, "metric": metric, "multiplier": multiplier,
                               "threshold": subset[0]["threshold"], "passing_conditions": passed,
                               "evaluated_conditions": len(subset),
                               "percentage": 100 * passed / len(subset)})
    expected = read_json(root / "expected/threshold_reanalysis_20261108.json")
    assert counts == expected["sensitivity_counts"]
    expected_original = {
        "first_peak_absolute_relative_error": [1, 6, 1, 3, 2, 0],
        "max_potential_over_initial_energy": [6, 12, 12, 11, 9, 12]}
    for metric, expected_counts in expected_original.items():
        result = [next(row["passing_conditions"] for row in counts
                       if row["platform"] == platform and row["metric"] == metric
                       and row["multiplier"] == 1) for platform in PLATFORMS]
        assert result == expected_counts
    return {"conditions": rows, "sensitivity_counts": counts,
            "original_figure4_integer_counts": expected_original,
            "all_six_multiplier_results_equal_20261108_reanalysis": True}


def gap_recompute(root):
    expected = read_json(root / "expected/bullet_gap_layered_case_20261108.json")
    original = read_json(root / "source_json/original_ideal_cradle_summary.json")
    shared = expected["shared_inputs"]
    diameter, length = 2 * shared["radius_m"], shared["length_m"]
    results = []
    for reference in expected["case_results"]:
        case = reference["case_id"]
        arrays = load_npz(root / "numeric/gap" / f"pybullet_{case}.npz")
        assert len(arrays["t"]) == 5281 and abs(float(arrays["t"][-1]) - 22) < 1e-10
        event, peak = first_peak(arrays["t"], arrays["x_D"][:, 0])
        mask = (arrays["t"] >= event) & (arrays["t"] <= event + 0.35)
        amplitudes = np.max(np.maximum(-arrays["x_D"][mask], 0), axis=0)
        ratio = float(amplitudes[1] / amplitudes[0])
        gap = reference["declared_case"]["gap_m"]
        pivots = np.array([[(i - 2) * (diameter + gap), 0, length + 0.04] for i in range(5)])
        angles = np.deg2rad([0, 0, 0, 0, 60])
        requested = pivots + np.array([length * np.sin(angles), np.zeros(5),
                                       -length * np.cos(angles)]).T
        error = float(np.max(np.abs(arrays["position"][0] - requested)))
        radius_error = float(np.max(np.abs(
            np.linalg.norm(arrays["position"] - pivots[None, :, :], axis=2) - length)))
        energy = np.sum(arrays["position"][:, :, 2] - 0.04, axis=1) / (
            length * (1 - math.cos(math.radians(60))))
        assert np.max(np.abs(energy - arrays["U_over_E0"])) < 1e-12
        energy_max = float(np.max(arrays["U_over_E0"]))
        result = {"case_id": case, "first_peak_D_recomputed": peak[1],
                  "first_peak_s_recomputed": peak[0],
                  "first_outgoing_motion_s_recomputed": event,
                  "R21_recomputed": ratio,
                  "max_potential_over_initial_energy_recomputed": energy_max,
                  "max_initial_readback_error_m_recomputed": error,
                  "max_hinge_radius_error_m_recomputed": radius_error}
        for key in result:
            if key != "case_id":
                assert abs(result[key] - reference[key]) < 1e-13, (case, key)
        results.append(result)
    pair = next(row for row in original
                if row["engine"] == "pybullet" and row["id"] == "l3_pair_h240")
    velocity = pair["pair_native_vx_before_after"]
    effective_e = -(velocity[1][1] - velocity[1][0]) / (velocity[0][1] - velocity[0][0])
    assert abs(effective_e - expected["isolated_pair_recalculation"][
        "effective_relative_velocity_ratio_recomputed"]) < 1e-14
    return {"case_results": results, "isolated_pair_ratio_from_archived_velocity_samples": effective_e,
            "all_gap_key_values_equal_20261108_reanalysis": True,
            "contact_scope": "Four-pair force activation timestamps are manuscript evidence, not recomputed here; compact inputs do not include raw contacts."}


def write_csv(path, rows):
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    package = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path,
                        default=package / "data")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    root = args.input_dir.resolve()
    output = (args.output_dir or package / "outputs").resolve()
    manifest = read_json(root / "manifest.json")
    verify_inputs(root, manifest)
    thresholds = threshold_recompute(root)
    gap = gap_recompute(root)
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "threshold_recomputed.json", thresholds)
    write_csv(output / "threshold_sensitivity_counts.csv", thresholds["sensitivity_counts"])
    write_json(output / "gap_recomputed.json", gap)
    receipt = {
        "completed_local": datetime.now().astimezone().isoformat(),
        "status": "PASS",
        "input_manifest_sha256": sha((root / "manifest.json").read_bytes()),
        "portable_entry_sha256": sha(Path(__file__).read_bytes()),
        "verified_input_files": len(manifest["files"]),
        "verified_numeric_npz_subsets": manifest["source_npz_count"],
        "input_member_integrity": "all included shapes, dtypes and C-order array-byte SHA256 passed",
        "figure4_original_counts": "2 columns × 6 platforms, all reproduced",
        "six_multiplier_counts": "2 metrics × 6 factors × 6 platforms, all equal expected 2026-11-08 results",
        "gap_key_metrics": "all equal expected 2026-11-08 results",
        "historical_absolute_paths_opened": 0,
        "scope": "CPU reanalysis only, no GPU/simulator/network/new-engine execution",
    }
    write_json(output / "PORTABLE_COMPLETE.json", receipt)
    print(json.dumps(receipt, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()


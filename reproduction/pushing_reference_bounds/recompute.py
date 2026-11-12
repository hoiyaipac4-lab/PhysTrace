"""Recompute a conservative analytical bound from archived pushing statistics."""
from pathlib import Path
from itertools import combinations
import argparse
import csv
import hashlib
import json
import math


def recompute(baseline):
    rows = []
    for a, b in combinations(baseline["engines"], 2):
        difference = abs(a["median_rmse_mm"] - b["median_rmse_mm"])
        for epsilon in baseline["epsilon_scenarios_mm"]:
            rows.append({
                "engine_a": a["engine"], "engine_b": b["engine"],
                "rms_reference_bound_mm": epsilon,
                "nominal_abs_difference_mm": difference,
                "worst_case_difference_shift_mm": 2 * epsilon,
                "order_guaranteed_by_bound": difference > 2 * epsilon,
            })
    return rows


def verify_archive(rows, path):
    old = list(csv.DictReader(path.open(encoding="utf-8-sig")))
    assert len(old) == len(rows) == 42
    for expected, actual in zip(old, rows):
        for key in ["engine_a", "engine_b"]:
            assert expected[key] == actual[key]
        for key in ["rms_reference_bound_mm", "nominal_abs_difference_mm", "worst_case_difference_shift_mm"]:
            assert math.isclose(float(expected[key]), actual[key], rel_tol=0, abs_tol=1e-14)
        assert (expected["order_guaranteed_by_bound"] == "True") == actual["order_guaranteed_by_bound"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    src = root / "baseline_statistics.json"
    baseline = json.loads(src.read_text(encoding="utf-8"))
    rows = recompute(baseline)
    verify_archive(rows, root / "archived_bound_scan.csv")
    nominal = [r for r in rows if r["rms_reference_bound_mm"] == 0.03]
    largest = max(r["nominal_abs_difference_mm"] for r in nominal)
    assert len(nominal) == 6
    assert not any(r["order_guaranteed_by_bound"] for r in nominal)
    summary = {
        "analysis_type": baseline["analysis_type"],
        "baseline_statistics_sha256": hashlib.sha256(src.read_bytes()).hexdigest(),
        "pair_count": 6, "epsilon_count": 7, "scan_rows": len(rows),
        "archived_42_rows_match": True,
        "largest_nominal_median_difference_mm": largest,
        "half_largest_difference_mm": largest / 2,
        "selected_epsilon_mm": 0.03,
        "pairwise_bound_mm_at_selected_epsilon": 0.06,
        "pairwise_orders_certified_at_selected_epsilon": 0,
        "interpretation": "At the assumed 0.03 mm RMS reference bound, the conservative interval test certifies none of the six pairwise orders.",
        "epsilon_is_measured_camera_error": False,
        "actual_reference_trajectory_perturbation_replay_performed": False,
        "realized_order_reversals_claimed": False,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "bound_scan.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (args.output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

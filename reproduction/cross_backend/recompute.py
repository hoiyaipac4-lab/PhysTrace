"""Aggregate the archived 6 x 6 policy-evaluation records and force pairs."""
import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parent
ENGINES = ("isaacsim", "mujoco", "pybullet", "drake", "newton", "gazebo")


def rows(name):
    with (ROOT / name).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    outcomes = defaultdict(dict)
    for row in rows("trials.csv"):
        source, execution = row["source_backend"], row["execution_backend"]
        assert source in ENGINES and execution in ENGINES
        assert row["group"] == ("same_backend" if source == execution else "cross_backend")
        completed = int(row["completed"])
        assert completed in (0, 1)
        cell = outcomes[source, execution]
        assert row["scene_id"] not in cell
        cell[row["scene_id"]] = completed
    assert len(outcomes) == 36
    assert all(set(cell) == {f"scene_{i:02d}" for i in range(1, 6)} for cell in outcomes.values())
    matrix = [[sum(outcomes[source, execution].values()) for execution in ENGINES]
              for source in ENGINES]
    diagonal = [matrix[i][i] for i in range(len(ENGINES))]
    assert diagonal == [4, 4, 3, 4, 3, 2]
    assert sum(map(sum, matrix)) - sum(diagonal) == 100
    assert sum(outcomes["newton", "gazebo"].values()) == 4
    assert sum(outcomes["newton", "mujoco"].values()) == 0
    force_changes = []
    for row in rows("fixed_command_force.csv"):
        baseline = float(row["baseline_reported_force_N"])
        treatment = float(row["treatment_reported_force_N"])
        percent = 100 * (treatment / baseline - 1)
        assert math.isclose(percent, float(row["relative_change_percent"]), abs_tol=1e-10)
        force_changes.append({"scene_id": row["scene_id"], "relative_force_change_percent": percent})
    assert len(force_changes) == 3
    result = {
        "backend_order": list(ENGINES), "matrix_completions_per_5": matrix,
        "cross_backend": {"completed": 100, "attempts": 150},
        "same_backend": {"completed": 20, "attempts": 30},
        "fixed_command_force_pairs": force_changes,
    }
    output = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")


if __name__ == "__main__":
    main()

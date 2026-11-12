# Pushing Reference-Error Bound

This package reproduces an analytical sensitivity bound for the four pushing-error medians in Section 5.2. It uses the archived 240 Hz, contact-coefficient 0.20 results, with eight completed clips per engine.

| File | Contents |
| --- | --- |
| `baseline_statistics.json` | Four archived medians, conditions, metric, reference hashes and assumptions. |
| `archived_bound_scan.csv` | The 42-row sensitivity table recorded on 9 November 2026. |
| `recompute.py` | Recomputes six engine pairs at seven assumed reference-error levels and checks the archived table. |
| `verified_results/` | Recomputed table and numerical summary. |

Run from the repository root with Python 3.12:

```sh
python reproduction/pushing_reference_bounds/recompute.py --output-dir outputs/pushing_reference_bounds
```

## Definition and calculation

For one clip, let `E_p = RMS(s_p - r)` be an engine's position RMSE against reference `r`. Under a common reference perturbation `delta` with `RMS(delta) <= epsilon`, the reverse triangle inequality gives:

```text
|RMS(s_p - r - delta) - E_p| <= epsilon
|M'_p - M_p| <= epsilon
|(M'_p - M'_q) - (M_p - M_q)| <= 2 * epsilon
```

`M_p` denotes the median of the eight per-clip RMSE values. The median inherits the per-clip bound. Sampling times, comparison windows, trajectory alignment, simulated paths and included clips are fixed throughout this calculation. The common-reference errors can be correlated across engines; the bound is a conservative sufficient test.

The archived medians are 2.200249 mm (MuJoCo), 2.150590 mm (Bullet), 2.151860 mm (Gazebo), and 2.143863 mm (Isaac). Their largest difference is approximately 0.056386 mm, reported as 0.06 mm in the discussion. Half of that largest difference is approximately 0.028193 mm.

The scan uses assumed RMS reference-error levels of 0.01, 0.03, 0.05, 0.10, 0.25, 0.50 and 1.00 mm. At epsilon = 0.03 mm, the pairwise difference bound is 0.06 mm. All six nominal pairwise differences fall within this bound, so this conservative test provides zero guaranteed pairwise orders. The script reproduces all 42 rows of the archived calculation.

The 0.03 mm value is an assumed sensitivity scenario. The half-gap calculation explains its scale relative to the archived medians. It is distinct from a measured camera-calibration error, a confidence interval or an observed order reversal. The reported result is an analytical interval calculation; recorded trajectories are held fixed. Actual trajectory perturbation replay and physical calibration are separate analyses.

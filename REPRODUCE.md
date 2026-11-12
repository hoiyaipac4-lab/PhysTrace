# Reproduction

Run commands from the repository root. Python 3.12.14 and NumPy 2.3.5 were used for numerical checks.

| Operation | Command |
| --- | --- |
| Recompute cradle thresholds and gap control | `python reproduction/cradle_thresholds/recompute.py --output-dir outputs/cradle_thresholds` |
| Run 12 native MuJoCo cases | `python reproduction/cradle_mujoco/replay.py --output-dir outputs/cradle_mujoco` |
| Aggregate cross-platform evaluation and contact-force results | `python reproduction/cross_backend/recompute.py --output outputs/cross_backend.json` |
| Recompute pushing reference-error bounds | `python reproduction/pushing_reference_bounds/recompute.py --output-dir outputs/pushing_reference_bounds` |
| Validate the example record | `python reproduction/protocol/validate_record.py` |
| Check encoder and scoring functions | `python -B reproduction/cross_backend/frozen_protocol/source_reference/check_scoring.py` |

Install the requirements in the selected reproduction directory. Simulator runtimes are installed from their upstream distributions.

[Data and execution details](reproduction/README.md) · [Policy inference](models/README.md) · [Scene setup](scenes/README.md)

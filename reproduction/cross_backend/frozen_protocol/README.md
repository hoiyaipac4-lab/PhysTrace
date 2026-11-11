# Cross-Platform Evaluation Configuration

| File | Contents |
| --- | --- |
| `evaluation.json` | Prediction horizon, execution cadence, time budget, controller limits, protection and scoring. |
| `scenes.json` | Five paired development initial states. |
| `scene_preparation.json`, `geometry.json`, `accepted_calibration.json` | Scene geometry, preparation and calibration. |
| `native_task_physics.json` | Native physical parameters and prim paths. |
| `isaacsim_trace_schema.json` | Process-record fields and units. |
| `source_map.json` | Source and per-cell hashes. |
| `source_reference/` | Measurement code, scoring functions and evaluator references. |

The evaluation uses two RGB images, six measured joint angles, gripper opening and a task prompt. Each query predicts 16 seven-channel targets; four frames are executed between queries. A common calibrated renderer supplies RGB, while the destination backend supplies physics and feedback. Each rollout has a 900-frame budget and a continuous one-second stable-lift score.

The saved result table contains 150 cross-platform and 30 same-platform trials. [Result aggregation](../recompute.py) recomputes counts from these outcomes.

# Reproduction Packages

| Directory | Inputs and outputs |
| --- | --- |
| `cradle_thresholds/` | 72 saved trajectories, two gap-control records and six threshold settings; recomputed first-peak and energy pass counts. |
| `cradle_mujoco/` | 12 cases, procedural asset construction and reference MJCF files; native simulation outputs and comparison with the recorded MuJoCo versions. |
| `cross_backend/` | 150 cross-platform and 30 same-platform outcomes; aggregate results and three fixed-command contact-force comparisons. |
| `pushing_reference_bounds/` | Four pushing-error medians and seven assumed reference-error levels; a reproducible 42-row analytical order bound. |
| `protocol/` | Task specification, scoring rules, example record and field validation. |

The threshold arrays contain time, normalized displacement and energy ratios. Gap-control arrays include five-ball positions. File hashes, shapes and data types are checked on load.

The MuJoCo replay uses the 3.13.0 run and the 3.10.0 reference; environment records include the operating system and package versions. The policy summary aggregates saved outcomes. Fixed-command comparisons use 32 native substeps at 960 Hz and the selected 21.6 mm commanded opening.

[Commands](../REPRODUCE.md) · [Cross-platform configuration](cross_backend/frozen_protocol/README.md)

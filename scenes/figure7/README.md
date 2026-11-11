# Calibrated PiPER Scene

1. Copy the repository's `assets/piper/` directory into a separate scene workspace.
2. Extract the `simulation/` subtree from `PhysTrace_Figure7_PiPER_calibrated_scene_overlay_20261019.zip` into the same directory.
3. Open `simulation/figure7_calibration_1400024/checkerboard_scene.usda` in Isaac Sim 5.1.

Isaac Sim supplies the OmniPBR material modules. [scene_manifest.json](scene_manifest.json) records scene files and hashes; [static_validation.json](static_validation.json) records the 72-layer and 65-asset dependency check.

[Evaluation configuration](../../reproduction/cross_backend/frozen_protocol/README.md) supplies initial states, calibration, control settings and scoring parameters. Component terms are recorded in `assets/piper/THIRD_PARTY_NOTICES.md`.

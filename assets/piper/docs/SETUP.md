# Setup, paths and interface

## Runtime

Activate an installed Isaac Sim 5.1.0 environment using its own setup instructions, then select one launcher option:

```bash
export ISAAC_USE_ACTIVE_PYTHON=1
# Alternatively, for an installation alongside the repository:
# export ISAAC_SIM_DIR=../IsaacSim
# Or select an already initialized interpreter:
# export ISAAC_PYTHON=../isaac-env/bin/python
export OMNI_KIT_ACCEPT_EULA=YES  # after accepting NVIDIA's terms
bash scripts/launch.sh
```

The simulator is an external dependency, not an included asset. Environment variables select its location; personal installation paths belong in the user's shell or ignored local configuration. The launch script leaves GPU visibility unchanged.

## Path conventions

1. USD layers, mesh references and texture references resolve relative to their containing asset files. Directory structure is preserved.
2. Python code derives the asset root from `__file__`, avoiding dependency on a particular username or checkout location.
3. Relative `--config` and `--output-dir` arguments resolve from the launch working directory. Run examples from the repository root.
4. JSON provenance paths are relative to the containing JSON file. Paths in historical reports are for reading, not commands to rerun archived jobs. `external_sources/` entries identify inputs outside the supplied bundle; they do not claim that those inputs are included.
5. USD prim paths, such as `/World/TaskAssets`, and URDF link/joint identifiers are scene identifiers. They remain unchanged. System interfaces and package URLs retain their required syntax.

## Parameters

GUI entry: `--config PATH`, `--resolution WIDTH HEIGHT`, `--validate`, `--seconds N`, `--output-dir PATH`.

Training validation: `--config PATH`, `--resolution WIDTH HEIGHT`, `--repeats N`, `--seconds N`, `--output-dir PATH`. Its own default resolution is 320 × 240.

Configuration overrides are shallow top-level merges. Supplying `material_overrides` replaces that whole mapping. Original defaults are physics 480 Hz, control 30 Hz, GPU 0, image 1280 × 960, model image 224 × 224, and `plate_fixed: false`.

`home` has eight components: six joint angles in radians plus two finger positions in metres. Policy actions have seven components: six joint angles and full gripper opening. Create `SimulationApp` before importing `training_env.py`. `TrainingScene` provides reset, step and observation methods. `env.frames` contains physics-step records from the latest control interval; copy or save them before the next step. Native contact impulse units are N·s. Impulse divided by the physics interval is interval-average force.

`simulation/start_fixed.sh` forwards to the portable current launcher. `simulation/start.sh` retains the earlier `run_scene.py` entry through the same environment selector. Use the current launcher for the training scene. Windows component-preview scripts require an explicit Isaac Sim environment variable instead of a workstation-specific default.

## Validation

```bash
python tools/check_repository.py --full
```

This verifies syntax, manifest and configuration consistency. It does not initialize Isaac Sim or exercise a real robot. Runtime dependency and dynamics checks require the simulator. Original historical logs are retained as evidence, with identifying paths removed; they are not a new execution record for this anonymous copy.

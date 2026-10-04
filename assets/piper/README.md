# PhysTrace — PiPER simulation assets

Anonymous review snapshot of a PiPER manipulation workspace: lift desk, camera mount, clamps, appliances, task objects, and a simulation RGB-D/action interface.

## Runtime and scope

Use Linux and an initialized **Isaac Sim 5.1.0** Python environment, with a compatible NVIDIA GPU. Some source assets were authored with 6.0.1; archived execution records concern 5.1.0. This package contains assets and a simulation interface; trained policy weights and the complete benchmark are separate research artifacts.

Physics defaults remain **480 Hz physics / 30 Hz control**. Mass, friction, inertia, collision geometry, joint limits and control parameters retain their supplied values. Path portability and removal of identifying metadata are packaging changes, rather than a new dynamics validation.

## Get started

Download the full anonymous archive, or fetch all Git LFS objects if obtaining a Git checkout. Run from the repository root:

```bash
python tools/check_repository.py --full

# Activate your installed Isaac Sim Python environment first.
export ISAAC_USE_ACTIVE_PYTHON=1
# Set only after reading and accepting NVIDIA's applicable license terms.
export OMNI_KIT_ACCEPT_EULA=YES
bash scripts/launch.sh --config configs/local.example.json
```

An installation alongside this repository can instead be selected with `export ISAAC_SIM_DIR=../IsaacSim`. A custom initialized interpreter can be selected with `ISAAC_PYTHON`. These are external runtime locations; the runtime itself is not distributed here.

```bash
bash scripts/launch.sh --validate --seconds 10 --config configs/local.example.json
bash scripts/launch.sh --validate-training --repeats 2 --seconds 3 --config configs/local.example.json
```

Outputs go to `outputs/`. Keep the asset directory hierarchy intact. Unicode directory names are intentional and referenced by scene files.

## Layout

| Path | Contents |
|---|---|
| `simulation/scene_training.usda` | Current training scene |
| `simulation/run_fixed.py` | Scene entrypoint |
| `simulation/training_env.py` | Simulation observation/action interface |
| `simulation/training_config.json` | Physical and control configuration |
| `simulation/robot_description/` | Robot description and portable resources |
| `simulation/PiPER_Scene_Assets_v1/` | Scene components |
| `simulation/source_v4/`, `simulation/task_assets/` | Task objects and source representations |
| `simulation/repair_evidence/` | Archived observations; paths anonymized, measured values retained |
| `scripts/launch.sh` | Portable launcher |
| `docs/SETUP.md` | Configuration and path semantics |
| `docs/ANONYMITY.md` | Review scope, exclusions and remaining publication checks |
| `docs/bundle_manifest.json` | Current anonymous package integrity manifest |

## Attribution and evidence

Upstream manufacturer/source references remain available in `simulation/robot_description/provenance.json` and `simulation/URDF_核查报告.md`. They identify third-party models, rather than the submitting authors. See `THIRD_PARTY_NOTICES.md` for redistribution checks.

Historical evidence contains hashes of the original tested files. The current anonymous package has its own integrity manifest; metadata/path-only edits do not constitute a rerun of the archived experiment. External source-history locations are represented by relative `external_sources/` references and documented in `docs/external_sources.json`. These are provenance identifiers, not bundled runtime dependencies.

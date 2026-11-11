# Newton's Cradle

| Item | Contents |
| --- | --- |
| `cases/` | 16 cases per backend: geometry, release state, contact settings and integration budget. |
| `native/` | 96 asset manifests and 48 native Gazebo, Isaac Sim and MuJoCo scene files. |
| `native_cradle_observer.py` | Native simulation and observation runner for six backends. |
| `measurement.json` | Measurement and estimation provenance. |
| `selected_profiles.json` | Isolated-pair parameter selection. |
| `manifest.json` | File hashes and reference-scene mappings. |

The model uses five finite-mass pendulums with native revolute joints and ball contacts. Paired `matched` and `gap100um` cases vary adjacent ball spacing.

```sh
python native_cradle_observer.py --engine mujoco --case cases/mujoco_l1_matched.json --out runs/mujoco_l1_matched
```

Run from this directory with NumPy and the selected backend installed. Backend identifiers are `drake`, `gazebo`, `isaac`, `mujoco`, `newton` and `pybullet`. Output files include `result.json`, `states.json` and `contacts.json`.

The matched and gap-control MuJoCo checks each produced 21,120 native steps and 21,121 state samples over 22 simulated seconds. The 16 Isaac scene layers passed OpenUSD loading checks; the 16 Gazebo scenes passed XML structure checks.

# Policy Inference Materials

| Item | Contents |
| --- | --- |
| `models.json` | Six frozen 12,000-update policy identities, checksums and attachment manifests. |
| `<backend>/12000/assets/` | Checkpoint-specific normalization files. |
| `runtime/` | OpenPI inference sources, configuration and environment versions. |
| `scripts/` | Observation loading and policy inference. |
| `training/` | Training specifications and data provenance. |

Checkpoint archives contain standalone OpenPI JAX/Orbax inference parameters. Restore an archive with its matching manifest:

```sh
python tools/checkpoint_release.py restore --manifest phystrace-isaacsim-12000-release-manifest.json --output checkpoints/isaacsim
python models/scripts/infer_npz.py --model isaacsim --checkpoint-root checkpoints/isaacsim --input observation.npz --output actions.npy
```

Input fields are `head` and `wrist` HWC RGB uint8 images, a seven-value `state`, and a scalar `prompt`. State and action channels are six absolute joint angles in radians and total gripper opening in metres. Output shape is 16 by 7. Optional `seed` and `frame` fields select the recorded fixed-noise behavior; inference uses 20 denoising steps.

OpenPI sources retain [Apache 2.0](runtime/vendor/OPENPI_LICENSE.txt). Model components follow the upstream [OpenPI](https://github.com/Physical-Intelligence/openpi) and [Gemma terms](https://ai.google.dev/gemma/terms).

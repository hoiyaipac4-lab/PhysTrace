# Measurement and Scoring Sources

| Directory | Contents |
| --- | --- |
| `measurement/` | 18 source files for state encoding, clearance projection, native timing, geometry, contact checks and stable-lift scoring. |
| `evaluator_references/` | Seven evaluator entrypoints covering six destination backends and the Isaac Sim I/O variant. |
| `source_manifest.json` | Source identity and file hashes. |

```sh
python -B reproduction/cross_backend/frozen_protocol/source_reference/check_scoring.py
```

The offline check exercises the encoder and three scoring helpers using synthetic arrays: stable lift, short duration, bilateral contact, force, clearance and timestamp conditions.

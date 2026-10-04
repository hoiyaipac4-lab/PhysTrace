# Provenance and publication checklist

This repository wraps the supplied `PiPER_Final_20260926.zip`. Runtime assets, code and numerical evidence are retained under `simulation/`; optional authoring/history files are excluded as described in `docs/ANONYMITY.md`.

## License status

The archive inventory did not identify a LICENSE/COPYING file. The packaging step assigns no blanket open-source license or third-party ownership claim. Choose a project license only after the rights holders confirm its scope. Keep upstream notices and add the applicable component-specific terms when confirmed.

| Component | Existing provenance / scope | Before public publication |
|---|---|---|
| PiPER robot description and meshes | `simulation/robot_description/provenance.json`, `simulation/URDF_核查报告.md` | Confirm upstream model/code redistribution and notices |
| Scanned/reconstructed fixtures and supports | Asset manifests and reports in the bundled asset directories | Confirm ownership of scan inputs, reconstructed geometry and textures |
| Microwave, air fryer, plate and task assets | `simulation/source_v4/`, `simulation/task_assets/`, `simulation/asset_ledger.json` | Confirm source licenses and permitted redistribution |
| Reference photos and rendered evidence | `simulation/references/`, asset reports and `simulation/repair_evidence/` | Review visibility of people, location, screens and proprietary equipment |
| NVIDIA Isaac Sim / standard MDL modules | External runtime dependency | Users obtain and accept the applicable NVIDIA software terms separately |
| Repository wrapper and launch/check tools | Packaging additions dated 2026-09-29 | Project owner chooses release terms together with the project |

Workstation paths in this review copy are converted to relative references. External provenance inputs are identified separately in `docs/external_sources.json`. Automated secret scanning here checks common private-key/token signatures, and complements a human publication review. It does not establish comprehensive privacy clearance.

Recommended first publication setting: **Private**, followed by a rights/privacy review and an explicit license decision before switching to Public.

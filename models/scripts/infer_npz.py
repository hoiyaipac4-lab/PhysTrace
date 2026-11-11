"""Run the archived PiPER policy on a saved observation file."""

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_external_policy(checkpoint: Path):
    from policy_core import make_inference_config
    from openpi.policies import policy_config

    options = json.loads((ROOT / "runtime/inference_options.json").read_text(encoding="utf-8"))
    policy = policy_config.create_trained_policy(
        make_inference_config(checkpoint),
        checkpoint,
        sample_kwargs={"num_steps": int(options["denoise_steps"])},
    )
    return policy, options


def main() -> None:
    manifest = json.loads((ROOT / "models.json").read_text(encoding="utf-8"))
    models = {item["engine"]: item for item in manifest["models"]}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, choices=models)
    parser.add_argument(
        "--checkpoint-root",
        type=Path,
        help="Directory created by checkpoint_release.py restore, containing models/",
    )
    parser.add_argument("--input", required=True, type=Path, help="Observation NPZ")
    parser.add_argument("--output", required=True, type=Path, help="New action NPY")
    args = parser.parse_args()

    model = models[args.model]
    root = ROOT if args.checkpoint_root is None else args.checkpoint_root / "models"
    checkpoint = (root / model["checkpoint"]).resolve()
    params = checkpoint / "params"
    if not params.is_dir() or not any(params.iterdir()):
        parser.error(f"restore the model weights under {params} first")
    if not args.input.is_file():
        parser.error("the observation NPZ does not exist")
    if args.output.exists():
        parser.error("the output path already exists")

    norm = checkpoint / "assets/piper_three_tasks/norm_stats.json"
    if not norm.is_file() or hashlib.sha256(norm.read_bytes()).hexdigest() != model["normalization_sha256"]:
        parser.error(f"normalization file missing or SHA-256 mismatch: {norm}")

    from policy_core import infer_body, load_policy

    if args.checkpoint_root is None:
        policy, options, _ = load_policy(args.model)
    else:
        policy, options = load_external_policy(checkpoint)
    prediction = infer_body(policy, options, args.input.read_bytes())
    with args.output.open("xb") as output:
        output.write(prediction)


if __name__ == "__main__":
    main()

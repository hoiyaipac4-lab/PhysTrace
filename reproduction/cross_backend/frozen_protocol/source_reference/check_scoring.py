#!/usr/bin/env python3
"""Check archived source hashes and exercise the scoring code on synthetic arrays."""

import ast
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent


def verify_sources():
    manifest = json.loads((ROOT / "source_manifest.json").read_text(encoding="utf-8"))
    provenance = json.loads((ROOT / manifest["original_provenance"]).read_text(encoding="utf-8"))
    original_digests = {
        (path, digest)
        for group in provenance["cross_backend_dependency_variants"].values()
        for path, digest in group["source_sha256"].items()
    }
    for entry in manifest["files"]:
        released = (ROOT / entry["release_path"]).read_bytes()
        assert hashlib.sha256(released).hexdigest() == entry["release_sha256"]
        assert (entry["historical_source"], entry["historical_sha256"]) in original_digests
        assert entry["byte_identical"] == (entry["historical_sha256"] == entry["release_sha256"])
        ast.parse(released, filename=entry["release_path"])
    return len(manifest["files"])


def check_scorer(name, score, clock, timebase):
    poses = np.repeat(np.eye(4)[None, :, :], len(clock), axis=0)
    poses[:, 2, 3] = 0.080
    contacts = np.zeros((len(clock), 8))
    contacts[:, 0:3] = [2.0, 1.0, 1.0]
    contacts[:, 5] = 0.0675
    geometry = {"minimum_clearance_m": 0.006, "minimum_close_tip_m": 0.009, "table_contact_samples": 0}

    full = score(clock, poses, contacts, timebase, geometry)
    assert full["strict_pass"]
    assert full["complete_full1s_window"]
    assert full["window_samples"] == len(clock)

    too_short = score(clock[:-1], poses[:-1], contacts[:-1], timebase, geometry)
    assert not too_short["strict_pass"]
    assert not too_short["complete_full1s_window"]

    one_finger = contacts.copy()
    one_finger[-1, 2] = 0.0
    incomplete_contact = score(clock, poses, one_finger, timebase, geometry)
    assert not incomplete_contact["strict_pass"]
    assert "bilateral_height_velocity_all_full1s_samples" in incomplete_contact["failed_checks"]

    over_force = contacts.copy()
    over_force[0, 0] = 12.1
    excessive_force = score(clock, poses, over_force, timebase, geometry)
    assert not excessive_force["strict_pass"]
    assert "all_policy_robot_object_force_at_most_12N" in excessive_force["failed_checks"]

    tight_clearance = geometry | {"minimum_close_tip_m": 0.008}
    geometry_rejection = score(clock, poses, contacts, timebase, tight_clearance)
    assert not geometry_rejection["strict_pass"]
    assert "original_geometry_guard_passed" in geometry_rejection["failed_checks"]

    bad_clock = clock.copy()
    bad_clock[2] = bad_clock[1]
    try:
        score(bad_clock, poses, contacts, timebase, geometry)
    except ValueError:
        pass
    else:
        raise AssertionError("Duplicate native timestamp was accepted")
    return {"clock": name, "synthetic_cases_passed": 6}


def main():
    count = verify_sources()
    sys.path.insert(0, str(ROOT / "measurement"))
    from data_contract import encode
    from strict_native_actual_dt_full1s_v1_20261016 import full1s_actual_dt_grasp_lift_metrics
    from strict_native_full1s_metric_v1_20261016 import full1s_grasp_lift_metrics
    from strict_native_nanosecond_full1s_v1_20261016 import full1s_grasp_lift_metrics_ns

    joints = np.array([0.1, -0.2, 0.3, 0.4, 0.5, -0.6, 0.03, -0.02])
    expected = np.array([0.1, -0.2, 0.3, 0.4, 0.5, -0.6, 0.05], dtype=np.float32)
    np.testing.assert_array_equal(encode(joints), expected)

    cases = [
        check_scorer("nominal-rate seconds", full1s_grasp_lift_metrics, np.arange(961) / 960, 960),
        check_scorer("measured-step seconds", full1s_actual_dt_grasp_lift_metrics, np.arange(1025) / 1024, 1 / 1024),
        check_scorer("integer nanoseconds", full1s_grasp_lift_metrics_ns, np.arange(1001) * 1_000_000, 1_000_000),
    ]
    print(json.dumps({"source_hashes_and_syntax_verified": count, "input_encoding_cases_passed": 1,
                      "synthetic_scoring_cases": cases, "scope": "No native engine or archived rollout execution."}, indent=2))


if __name__ == "__main__":
    main()

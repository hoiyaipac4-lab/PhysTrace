"""Verified Gazebo force/penetration failures; unknown errors stay invalid."""
import ast,numpy as np
from native_policy_guard_classification_v1_20261016 import verified_physical_guard as common_guard
def verified_physical_guard(error,native_reply,advance_in_progress):
 if not advance_in_progress or not isinstance(native_reply,dict) or native_reply.get('recording_error'):return False
 snapshot=native_reply.get('failure_snapshot')
 if not isinstance(snapshot,dict) or not isinstance(snapshot.get('observed_after_error'),dict) or snapshot['observed_after_error'].get('engine')!='gazebo':return False
 marker='RuntimeError: Gazebo native per-step contact guard ';message=error.strip().splitlines()[-1]
 if message.startswith('AssertionError: '):return common_guard(error,native_reply,advance_in_progress)
 if not message.startswith(marker):return False
 try:row=np.asarray(ast.literal_eval(message[len(marker):]),dtype=float)
 except Exception:return False
 return row.shape==(8,) and bool(np.isfinite(row).all()) and bool(row[0]>12 or max(row[1:3])>12 or row[3]>8 or row[4]>.002)

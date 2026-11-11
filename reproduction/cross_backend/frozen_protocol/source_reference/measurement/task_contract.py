"""Shared task definitions; acceptance uses measured native physical traces."""
import numpy as np

PROMPTS = {
    'grasp_only': 'Grasp the wood cube and hold it steadily on the table.',
    'lift_only': 'Lift the already grasped wood cube and hold it steadily above the table.',
    'grasp_lift': 'Grasp the wood cube, lift it, and hold it steadily above the table.',
}

def physical_metrics(contacts, poses, task):
    c = np.asarray(contacts); p = np.asarray(poses)
    assert c.ndim == 2 and c.shape[1] == 8 and np.isfinite(c).all()
    assert p.ndim == 3 and p.shape[1:] == (4, 4) and np.isfinite(p).all()
    bilateral = (c[:, 1] > .05) & (c[:, 2] > .05)
    stable = bilateral & (c[:, 6] < .015)
    stable &= (c[:, 5] < .002) if task == 'grasp_only' else (c[:, 5] > .05)
    bad = np.flatnonzero(~stable)
    trailing = len(stable) - 1 - bad[-1] if len(bad) else len(stable)
    last = p[-30:]
    drift = np.linalg.norm(last[:, :3, 3] - last[0, :3, 3], axis=1).max()
    rotation = last[:, :3, :3] @ last[0, :3, :3].T
    angle = np.degrees(np.arccos(np.clip((np.trace(rotation, axis1=1, axis2=2) - 1) / 2, -1, 1))).max()
    bottom = last[:, 2, 3] - .0125 * np.abs(last[:, 2, :3]).sum(axis=1)
    height_ok = bottom.max() < .002 if task == 'grasp_only' else bottom.min() > .05
    forces_ok = c[:, 0].max() <= 12 and c[:, 1:3].max() <= 12 and c[:, 3].max() <= 8 and c[:, 4].max() <= .002
    return dict(success=bool(len(last) == 30 and trailing >= 960 and drift < .002 and angle < 5 and height_ok and forces_ok),
                stable_hold_s=float(trailing / 960), position_drift_m=float(drift), rotation_drift_deg=float(angle),
                minimum_final_bottom_m=float(bottom.min()), maximum_final_bottom_m=float(bottom.max()),
                peak_robot_object_N=float(c[:, 0].max()), peak_finger_N=float(c[:, 1:3].max()),
                peak_unexpected_N=float(c[:, 3].max()), max_penetration_m=float(c[:, 4].max()))

def acceptance(contacts, poses, task, rt):
    result = physical_metrics(contacts, poses, task)
    result.update(closing_tip_min_m=float(rt.closing_tip_min), gripper_table_min_m=float(rt.audit_min),
                  gripper_table_contacts=int(rt.audit_contacts))
    result['success'] &= rt.closing_tip_min > .008 and rt.audit_min >= .005 and rt.audit_contacts == 0
    return result

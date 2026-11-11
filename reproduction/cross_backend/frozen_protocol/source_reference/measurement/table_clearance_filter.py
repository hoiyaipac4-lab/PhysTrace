"""Table-only workspace constraint. No object pose, seed, phase, or expert input."""
import numpy as np

TARGET_CLEARANCE_M = .010  # 8 mm acceptance floor plus 2 mm command margin.
MAX_UPWARD_SHIFT_M = .020

def project_target(target, kinematics):
    target = np.asarray(target, dtype=float)
    if target.shape != (8,) or not np.isfinite(target).all():
        raise ValueError('Table filter requires eight finite joint targets')
    clearance = kinematics.fingertip_clearance(target)
    if clearance >= TARGET_CLEARANCE_M:
        return target.copy(), 0.
    shift = TARGET_CLEARANCE_M - clearance + 1e-6
    if shift > MAX_UPWARD_SHIFT_M:
        raise ValueError('Model target exceeds maximum table safety correction')
    original = kinematics.link6(target)
    desired = original.copy()
    desired[2, 3] += shift
    base_pose = kinematics.inv @ desired
    protected = kinematics.solve_link6(base_pose[:3, 3], base_pose[:3, :3],
                                       target[:6], target[6:])
    achieved = kinematics.link6(protected)
    if (kinematics.fingertip_clearance(protected) < TARGET_CLEARANCE_M - 1e-7
            or np.max(abs(achieved[:2, 3] - original[:2, 3])) > 1e-6
            or np.max(abs(achieved[:3, :3] - original[:3, :3])) > 1e-5
            or not np.array_equal(protected[6:], target[6:])):
        raise ValueError('Table safety projection failed its geometry constraints')
    return protected, shift

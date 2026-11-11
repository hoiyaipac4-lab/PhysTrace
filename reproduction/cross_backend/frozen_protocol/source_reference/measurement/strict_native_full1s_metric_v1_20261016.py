"""Candidate measurement helper, not an admitted final driver.

Requires producer-verified actual native object/contact samples, without
interpolation. No engine, policy, scene or final holdout access. Existing
30Hz legacy outcomes remain separate. Integration/native validation pending.
"""
import numpy as np


def full1s_grasp_lift_metrics(native_time_s, object_base_pose, contact_telemetry,
                            native_hz, geometry_audit):
    """Evaluate exact endpoint-inclusive last1s; malformed clocks are invalid.

    Inputs must contain the complete POLICY interval, excluding settling.
    Contact fields use the existing eight-column native telemetry contract.
    Geometry values come from the existing native FK/guard audit; this helper
    does not independently establish their provenance or physical fidelity.
    """
    t=np.asarray(native_time_s,dtype=np.float64)
    p=np.asarray(object_base_pose,dtype=np.float64)
    c=np.asarray(contact_telemetry,dtype=np.float64)
    if not isinstance(native_hz,(int,np.integer)) or native_hz<=0:
        raise ValueError('Declared positive integer native sample rate required')
    if t.ndim!=1 or len(t)==0 or p.shape!=(len(t),4,4) or c.shape!=(len(t),8):
        raise ValueError('Synchronized actual native time/pose/contact shapes required')
    if not all(np.isfinite(a).all() for a in (t,p,c)):
        raise ValueError('Nonfinite native evidence')
    # Clock tolerance is only float accumulated-time roundoff, not permission
    # to accept dropped samples, duplicated timestamps or shortened windows.
    if len(t)>1 and not np.allclose(np.diff(t),1/native_hz,rtol=0,atol=1e-9):
        raise ValueError('Native samples missing, duplicated or incorrectly timed')
    if not np.allclose(p[:,3,:],np.array([0.,0.,0.,1.]),rtol=0,atol=1e-9):
        raise ValueError('Invalid homogeneous pose matrices')
    rotations=p[:,:3,:3]
    if not np.allclose(rotations@np.transpose(rotations,(0,2,1)),np.eye(3),rtol=0,atol=1e-6) or not np.allclose(np.linalg.det(rotations),1.,rtol=0,atol=1e-6):
        raise ValueError('Object rotation is not a proper native rotation matrix')
    fields=('minimum_clearance_m','minimum_close_tip_m','table_contact_samples')
    if not all(k in geometry_audit and np.isfinite(geometry_audit[k]) for k in fields):
        raise ValueError('Original geometry-guard evidence missing or nonfinite')
    if geometry_audit['table_contact_samples']<0:
        raise ValueError('Invalid geometry count')
    out={'measurement_version':'strict_native_full1s_v1_20261016',
         'declared_native_hz':int(native_hz),'available_policy_samples':len(t),
         'complete_full1s_window':False,'strict_pass':False,
         'sampling_scope':'Actual native observations required; no interpolation in helper',
         'not_final_admission_or_replacement_for_legacy_results':True}
    if len(t)<native_hz+1:
        out['reason']='insufficient_endpoint_inclusive_one_second_observations'
        return out
    start=len(t)-native_hz-1
    span=float(t[-1]-t[start])
    if abs(span-1.)>1e-9:
        raise ValueError('Last native window does not span exactly one second')
    w=p[start:];ct=c[start:]
    drift=float(np.linalg.norm(w[:,:3,3]-w[0,:3,3],axis=1).max())
    rotation=w[:,:3,:3]@w[0,:3,:3].T
    angle=float(np.degrees(np.arccos(np.clip((np.trace(rotation,axis1=1,axis2=2)-1)/2,-1,1))).max())
    bottom=w[:,2,3]-.0125*np.abs(w[:,2,:3]).sum(axis=1)
    stable=(ct[:,1]>.05)&(ct[:,2]>.05)&(ct[:,6]<.015)&(ct[:,5]>.05)
    checks={
      'bilateral_height_velocity_all_full1s_samples':bool(stable.all()),
      'translation_drift_below_2mm':bool(drift<.002),
      'rotation_drift_below_5deg':bool(angle<5),
      'entire_full1s_object_bottom_above_50mm':bool(bottom.min()>.05),
      'all_policy_robot_object_force_at_most_12N':bool(c[:,0].max()<=12),
      'all_policy_finger_force_at_most_12N':bool(c[:,1:3].max()<=12),
      'all_policy_unexpected_force_at_most_8N':bool(c[:,3].max()<=8),
      'all_policy_penetration_at_most_2mm':bool(c[:,4].max()<=.002),
      'original_geometry_guard_passed':bool(geometry_audit['minimum_clearance_m']>=.005 and geometry_audit['minimum_close_tip_m']>.008 and geometry_audit['table_contact_samples']==0)}
    out.update(complete_full1s_window=True,window_samples=native_hz+1,
               actual_window_span_s=span,position_drift_m=drift,
               rotation_drift_deg=angle,minimum_final_bottom_m=float(bottom.min()),
               checks=checks,failed_checks=[k for k,v in checks.items() if not v],
               strict_pass=all(checks.values()))
    return out

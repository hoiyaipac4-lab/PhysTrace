"""Measurement for verified integer native clocks, without resampling.

Uses the closest actual sample at or before end-minus-one-second, so the
observed window spans at least one full second. Physical thresholds match
strict_native_full1s_metric_v1_20261016. This alone is not final admission.
"""
import numpy as np

def full1s_grasp_lift_metrics_ns(native_time_ns,object_base_pose,contact_telemetry,
                               actual_native_step_ns,geometry_audit):
    raw=np.asarray(native_time_ns)
    if raw.ndim!=1 or not len(raw) or raw.dtype.kind not in 'iu':
        raise ValueError('Actual integer native nanosecond timestamps required')
    if np.any(raw<0) or np.any(raw>np.iinfo(np.int64).max):
        raise ValueError('Invalid native nanosecond clock')
    ns=raw.astype(np.int64,copy=False)
    if isinstance(actual_native_step_ns,(bool,np.bool_)) or not isinstance(actual_native_step_ns,(int,np.integer)) or actual_native_step_ns<=0:
        raise ValueError('Positive integer native clock step required')
    if len(ns)>1 and not np.all(np.diff(ns)==actual_native_step_ns):
        raise ValueError('Native sample missing, duplicated or clock step mismatched')
    p=np.asarray(object_base_pose,dtype=np.float64);c=np.asarray(contact_telemetry,dtype=np.float64)
    if p.shape!=(len(ns),4,4) or c.shape!=(len(ns),8) or not np.isfinite(p).all() or not np.isfinite(c).all():
        raise ValueError('Synchronized finite native poses and contact observations required')
    if not np.allclose(p[:,3,:],np.array([0.,0.,0.,1.]),rtol=0,atol=1e-9):
        raise ValueError('Invalid homogeneous pose matrices')
    rotations=p[:,:3,:3]
    if not np.allclose(rotations@np.transpose(rotations,(0,2,1)),np.eye(3),rtol=0,atol=1e-6) or not np.allclose(np.linalg.det(rotations),1.,rtol=0,atol=1e-6):
        raise ValueError('Object rotation is not a proper native rotation matrix')
    fields=('minimum_clearance_m','minimum_close_tip_m','table_contact_samples')
    if not all(k in geometry_audit and np.isfinite(geometry_audit[k]) for k in fields) or geometry_audit['table_contact_samples']<0:
        raise ValueError('Original geometry-guard evidence missing or invalid')
    out={'measurement_version':'strict_native_nanosecond_full1s_v1_20261016','actual_native_step_ns':int(actual_native_step_ns),'available_policy_samples':len(ns),'complete_full1s_window':False,'strict_pass':False,'sampling_scope':'Actual integer native timestamps, no interpolation, rescaling or nominal clock substitution','not_final_admission_or_replacement_for_legacy_results':True}
    if int(ns[-1])-int(ns[0])<1_000_000_000:
        out['reason']='insufficient_actual_native_duration_for_full_second';return out
    start=int(np.searchsorted(ns,int(ns[-1])-1_000_000_000,side='right'))-1
    span_ns=int(ns[-1])-int(ns[start]);assert 1_000_000_000<=span_ns<1_000_000_000+actual_native_step_ns
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
    out.update(complete_full1s_window=True,window_samples=len(ns)-start,actual_window_span_ns=span_ns,actual_window_span_s=span_ns/1e9,window_overshoot_ns=span_ns-1_000_000_000,position_drift_m=drift,rotation_drift_deg=angle,minimum_final_bottom_m=float(bottom.min()),checks=checks,failed_checks=[k for k,v in checks.items() if not v],strict_pass=all(checks.values()))
    return out

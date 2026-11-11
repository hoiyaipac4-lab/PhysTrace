"""Read-only strict metric on existing native observer post-step records."""
import numpy as np
from strict_native_full1s_metric_v1_20261016 import full1s_grasp_lift_metrics
def geometry_from_runtime(rt):
 return {'minimum_clearance_m':float(rt.audit_min),'minimum_close_tip_m':float(rt.closing_tip_min),'table_contact_samples':int(rt.audit_contacts)}
def strict_metrics(arrays,geometry):
 if not arrays or len(arrays['native_t_end'])<961:return {'strict_pass':False,'complete_full1s_window':False,'reason':'Insufficient endpointinclusive native policy samples'}
 t=arrays['native_t_end'];assert np.allclose(t-arrays['native_t_start'],1/960,rtol=0,atol=1e-10)
 if not np.asarray(arrays['native_contact_valid']).all() or not np.asarray(arrays['native_returned']).all():return {'strict_pass':False,'complete_full1s_window':False,'reason':'Failed native step retained; stale or unreturned contact not interpolated or counted as success'}
 return full1s_grasp_lift_metrics(t,arrays['native_object_base_pose'],arrays['native_contact'],960,geometry)
def verified_physical_guard(error,rows,geometry,advance_in_progress):
 if not advance_in_progress or not rows:return False
 row=rows[-1]
 try:
  assert row['returned'] is False and abs(row['t_end']-row['t_start']-1/960)<1e-10
  assert np.array_equal(row['command_after'],row['sent_position']) and np.array_equal(row['sent_position'],row['position_target_readback'])
  assert np.asarray(row['q_after']).shape==(8,) and np.isfinite(row['q_after']).all()
  if error.startswith('Native contact guard:'):
   c=np.asarray(row['contact']);return bool(row['contact_valid'] and c.shape==(8,) and np.isfinite(c).all() and (c[0]>12 or max(c[1:3])>12 or c[3]>8 or c[4]>.002))
  if error.startswith('Closing fingertip clearance <=8mm:'):return bool(not row['contact_valid'] and geometry['minimum_close_tip_m']<=.008)
  if error.startswith('Visible gripper-table clearance below 5 mm:'):return bool(not row['contact_valid'] and geometry['minimum_clearance_m']<.005)
 except (AssertionError,ValueError,TypeError,KeyError,IndexError):return False
 return False

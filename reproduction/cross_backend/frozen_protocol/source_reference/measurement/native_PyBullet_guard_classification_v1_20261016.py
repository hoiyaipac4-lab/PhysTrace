"""Classify only an observed native physical guard; unknown errors stay invalid."""
import ast,numpy as np
from native_policy_guard_classification_v1_20261016 import verified_physical_guard as original_classifier
def verified_physical_guard(error,native_reply,advance_in_progress):
 if not original_classifier(error,native_reply,advance_in_progress):return False
 snapshot=native_reply['failure_snapshot'];after=snapshot['observed_after_error'];record=snapshot.get('substep_observer',{})
 if after.get('engine')!='pybullet' or record.get('recording_error') or native_reply.get('recording_error'):return False
 rows=record.get('actual_successful_step_rows',[])
 if not 0<len(rows)<=128:return False
 try:
  q=np.asarray(rows[-1]['native_joint_q']);last=rows[-1]
  if q.shape!=(8,) or not np.array_equal(q,after['actual_q']):return False
  index=np.asarray([r['executed_native_step_index'] for r in rows]);times=np.asarray([r['step_count_time_s'] for r in rows]);dt=np.asarray([r['reported_fixed_timestep_s'] for r in rows])
  if not np.all(dt==1/3840) or not np.all(np.diff(index)==1) or not np.array_equal(times,index*(1/3840)):return False
  if record['observed_native_step_count']!=index[-1]:return False
  line=error.strip().splitlines()[-1];prefix='RuntimeError: Native PyBullet v4 contact guard '
  if line.startswith(prefix):return np.array_equal(np.asarray(ast.literal_eval(line[len(prefix):])),last['native_contact_telemetry'])
  if line.startswith('AssertionError: '):
   item=ast.literal_eval(line[len('AssertionError: '):]);return isinstance(item,tuple) and len(item)==4 and item[0]=='Native continuous visual clearance'
 except (ValueError,TypeError,KeyError,IndexError,SyntaxError):return False
 return False

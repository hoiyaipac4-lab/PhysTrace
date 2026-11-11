"""Candidate classifier; only an observed native guard may count as policy failure."""
import ast,numpy as np
def verified_physical_guard(error,native_reply,advance_in_progress):
 if not advance_in_progress or not isinstance(native_reply,dict) or native_reply.get('recording_error'):return False
 snapshot=native_reply.get('failure_snapshot');
 if not isinstance(snapshot,dict):return False
 observer=snapshot.get('native_readonly_observer',{});after=snapshot.get('observed_after_error',{});row=observer.get('failed_actual_native_row')
 if observer.get('recording_error') or after.get('engine')!='newton' or not isinstance(row,dict):return False
 try:
  dt=float(row['actual_native_dt_s']);index=int(row['native_solver_call_index']);assert index>0 and index==row['native_solver_call_index'] and dt==float(np.float32(1/960)) and row['native_step_count_time_s']==index*dt
  returned=observer['successful_returned_rows'];assert 0<=len(returned)<32
  if returned:assert row['native_solver_call_index']==returned[-1]['native_solver_call_index']+1
  command=np.asarray(row['requested_controller_command']);assert command.shape==(8,) and np.isfinite(command).all() and np.array_equal(command,after['controller_command'])
  if row['output_state_committed']:assert np.array_equal(row['actual_joint_q_scratch_IK'],after['actual_q'])
  line=error.strip().splitlines()[-1];prefix='RuntimeError: Native Kamino contact guard '
  if line.startswith(prefix):
   contact=np.asarray(ast.literal_eval(line[len(prefix):]),dtype=float);assert contact.shape==(8,) and np.isfinite(contact).all() and np.array_equal(contact,row['original_contact_telemetry'])
   return bool(contact[0]>12 or max(contact[1:3])>12 or contact[3]>8 or contact[4]>.002)
  prefix='AssertionError: '
  if line.startswith(prefix):
   item=ast.literal_eval(line[len(prefix):])
   if not (isinstance(item,tuple) and len(item)==4 and item[0]=='Native continuous visual clearance' and row['output_state_committed']):return False
   _,minimum,tip,phase=item;return bool(np.isfinite([minimum,tip]).all() and (minimum<.005 or phase in ('close','grasp_hold') and tip<=.008))
 except (AssertionError,ValueError,TypeError,KeyError,IndexError,SyntaxError,OverflowError):return False
 return False

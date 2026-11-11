"""Actual Newton native port readings; original physics and limiter unchanged."""
import numpy as np
from strict_native_actual_dt_full1s_v1_20261016 import full1s_actual_dt_grasp_lift_metrics
FIELDS={'native_solver_call_index':(),'actual_native_dt_s':(),'native_step_count_time_s':(),'actual_joint_q_scratch_IK':(8,),'actual_joint_velocity_scratch_IK':(8,),'actual_native_target_q':(8,),'requested_controller_command':(8,),'native_dynamic_constraint_force':(8,),'native_object_base_pose':(4,4),'original_contact_telemetry':(8,),'model_frame':(),'output_state_committed':()}
ACTUAL_DT=float(np.float32(1/960))
def new_control_history():return {k:[] for k in FIELDS}
def append_control_history(history,executed):
 payload=executed['native_newton_readback'];assert payload['sampling_hz_nominal']==960 and len(payload['rows'])==32
 arrays={k:np.asarray([r[k] for r in payload['rows']],dtype=np.float64) for k in FIELDS}
 for k,shape in FIELDS.items():assert arrays[k].shape==(32,)+shape and np.isfinite(arrays[k]).all(),k
 previous=len(history['native_solver_call_index']);index=arrays['native_solver_call_index'];assert np.array_equal(index,np.arange(previous+1,previous+33))
 assert np.all(arrays['actual_native_dt_s']==ACTUAL_DT) and np.array_equal(arrays['native_step_count_time_s'],index*ACTUAL_DT)
 assert np.all(arrays['model_frame']==previous//32) and np.all(arrays['output_state_committed']==1)
 assert np.array_equal(arrays['actual_joint_q_scratch_IK'][-1],executed['observation']['actual_q']) and np.array_equal(arrays['native_object_base_pose'][-1],executed['observation']['object_base_pose'])
 assert np.array_equal(arrays['requested_controller_command'],executed['controller_command_substeps']) and np.array_equal(arrays['original_contact_telemetry'],executed['contact_substeps']) and np.array_equal(arrays['native_dynamic_constraint_force'],executed['actual_motor_effort_substeps'])
 assert np.array_equal(arrays['actual_native_target_q'],np.asarray(executed['controller_command_substeps']).astype(np.float32).astype(float))
 for k in FIELDS:history[k].extend(arrays[k].tolist())
def strict_control_metrics(history,geometry):
 if len(history['native_solver_call_index'])<961:return {'strict_pass':False,'complete_full1s_window':False,'available_native_samples':len(history['native_solver_call_index']),'reason':'Insufficient endpointinclusive actual solver-call samples'}
 assert np.all(np.asarray(history['actual_native_dt_s'])==ACTUAL_DT)
 return full1s_actual_dt_grasp_lift_metrics(history['native_step_count_time_s'],history['native_object_base_pose'],history['original_contact_telemetry'],ACTUAL_DT,geometry)
def control_trace_arrays(history):return {'native_newton_'+k:np.asarray(history[k],dtype=np.float64).reshape((-1,)+shape) for k,shape in FIELDS.items()}

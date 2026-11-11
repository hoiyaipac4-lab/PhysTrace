"""Read-only measurement over actual native SDK step calls, not sensor feedback."""
import numpy as np
from strict_native_full1s_metric_v1_20261016 import full1s_grasp_lift_metrics
FIELDS={'executed_native_step_index':(),'step_count_time_s':(),'reported_fixed_timestep_s':(),'model_frame':(),'native_joint_q':(8,),'native_joint_velocity':(8,),'native_torque_command':(8,),'SDK_joint_state_field3_raw':(8,),'SDK_joint_state_field2_raw':(8,6),'controller_position_command':(8,),'native_object_base_pose':(4,4),'native_contact_telemetry':(8,)}
def new_control_history():return {k:[] for k in FIELDS}
def append_control_history(history,executed):
 payload=executed['native_pybullet_readback'];assert payload['sampling_hz']==3840 and payload['sensor_enable_changed'] is False
 rows=payload['rows'];assert len(rows)==128
 arrays={k:np.asarray([row[k] for row in rows],dtype=np.float64) for k in FIELDS}
 for k,shape in FIELDS.items():assert arrays[k].shape==(128,)+shape and np.isfinite(arrays[k]).all(),k
 index=arrays['executed_native_step_index'];assert np.array_equal(index,np.rint(index)) and np.all(np.diff(index)==1)
 assert np.all(arrays['reported_fixed_timestep_s']==1/3840) and np.array_equal(arrays['step_count_time_s'],index*(1/3840))
 assert np.all(arrays['model_frame']==arrays['model_frame'][0]) and arrays['model_frame'][0]==len(history['model_frame'])//128
 if history['executed_native_step_index']:assert index[0]==history['executed_native_step_index'][-1]+1
 assert np.array_equal(arrays['native_joint_q'][-1],executed['observation']['actual_q']) and np.array_equal(arrays['native_object_base_pose'][-1],executed['observation']['object_base_pose'])
 assert np.array_equal(arrays['controller_position_command'].reshape(32,4,8),np.repeat(np.asarray(executed['controller_command_substeps'])[:,None,:],4,axis=1))
 # The legacy key says actual_motor_effort, but the original backend supplies sent commands.
 assert np.array_equal(arrays['native_torque_command'][3::4],executed['actual_motor_effort_substeps'])
 c=arrays['native_contact_telemetry'].reshape(32,4,8);reduced=c[:,-1].copy();reduced[:,:5]=c[:,:,:5].max(axis=1);reduced[:,1:3]=c[:,:,1:3].min(axis=1);reduced[:,6]=c[:,:,6].max(axis=1)
 assert np.array_equal(reduced,executed['contact_substeps'])
 for k in FIELDS:history[k].extend(arrays[k].tolist())
def strict_control_metrics(history,geometry):
 if len(history['step_count_time_s'])<3841:return {'strict_pass':False,'complete_full1s_window':False,'available_native_samples':len(history['step_count_time_s']),'reason':'Insufficient endpointinclusive actual native step calls'}
 return full1s_grasp_lift_metrics(history['step_count_time_s'],history['native_object_base_pose'],history['native_contact_telemetry'],3840,geometry)
def control_trace_arrays(history):return {'native_pybullet_'+k:np.asarray(history[k],dtype=np.float64).reshape((-1,)+shape) for k,shape in FIELDS.items()}

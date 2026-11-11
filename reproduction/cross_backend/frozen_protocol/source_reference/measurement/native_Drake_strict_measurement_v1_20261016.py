"""Read-only actual3840Hz Drake RPC parser; original guards and ports retained."""
import numpy as np
from strict_native_full1s_metric_v1_20261016 import full1s_grasp_lift_metrics
FIELDS={'native_time_s':(),'requested_time_s':(),'model_frame':(),'native_net_actuation':(8,),'native_joint_q':(8,),'native_joint_velocity':(8,),'actual_desired_state_port':(16,),'actual_feedforward_input_port':(8,),'native_object_base_pose':(4,4),'native_contact_telemetry':(8,)}
def new_control_history():return {key:[] for key in FIELDS}
def append_control_history(history,executed):
 payload=executed['native_drake_readback'];rows=payload['rows'];assert payload['sampling_hz']==3840 and len(rows)==128
 arrays={key:np.asarray([row[key] for row in rows]) for key in FIELDS}
 for key,shape in FIELDS.items():assert arrays[key].shape==(128,)+shape and np.isfinite(arrays[key]).all()
 t=arrays['native_time_s'];assert np.allclose(np.diff(t),1/3840,rtol=0,atol=1e-10) and np.allclose(t,arrays['requested_time_s'],rtol=0,atol=1e-10)
 if history['native_time_s']:assert abs(t[0]-history['native_time_s'][-1]-1/3840)<1e-10
 ct=arrays['native_contact_telemetry'].reshape(32,4,8);reduced=ct[:,-1,:].copy();reduced[:,:5]=ct[:,:,:5].max(axis=1);reduced[:,1:3]=ct[:,:,1:3].min(axis=1);reduced[:,6]=ct[:,:,6].max(axis=1);assert np.array_equal(reduced,np.asarray(executed['contact_substeps']))
 desired=arrays['actual_desired_state_port'];assert np.array_equal(desired[:,:8],np.repeat(np.asarray(executed['controller_command_substeps']),4,axis=0)) and np.count_nonzero(desired[:,8:])==0
 assert np.array_equal(arrays['native_net_actuation'][3::4],np.asarray(executed['actual_motor_effort_substeps'])) and np.array_equal(arrays['native_joint_q'][-1],np.asarray(executed['observation']['actual_q'])) and np.array_equal(arrays['native_object_base_pose'][-1],np.asarray(executed['observation']['object_base_pose']))
 assert np.all(np.abs(arrays['native_net_actuation'])<=np.r_[np.full(6,100.),10.,10.]+1e-4)
 assert np.all(arrays['model_frame']==arrays['model_frame'][0]) and int(arrays['model_frame'][0])>=0
 for key in FIELDS:history[key].extend(arrays[key].tolist())
def strict_control_metrics(history,geometry):
 if len(history['native_time_s'])<3841:return {'strict_pass':False,'complete_full1s_window':False,'available_policy_samples':len(history['native_time_s']),'reason':'Insufficient endpointinclusive actual3840Hz policy observations'}
 return full1s_grasp_lift_metrics(history['native_time_s'],history['native_object_base_pose'],history['native_contact_telemetry'],3840,geometry)
def control_trace_arrays(history):return {'dense_drake_'+key:np.asarray(history[key],dtype=np.int64 if key=='model_frame' else np.float64).reshape((-1,)+shape) for key,shape in FIELDS.items()}

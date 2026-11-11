"""Read-only parser of actual Gazebo control RPC data and native clock."""
import numpy as np
from strict_native_nanosecond_full1s_v1_20261016 import full1s_grasp_lift_metrics_ns
FIELDS={'native_time_ns':(),'time_s':(),'object_base_pose':(4,4),'actual_q':(8,),'motor_readback':(8,),'controller_command':(8,),'contact_telemetry':(8,)}
def new_control_history():return {key:[] for key in FIELDS}
def append_control_history(history,executed):
 payload=executed['native_control_readback'];rows=payload['rows'];assert payload['sampling_hz_nominal']==960 and len(rows)==32
 arrays={key:np.asarray([row[key] for row in rows]) for key in FIELDS}
 for key,shape in FIELDS.items():assert arrays[key].shape==(32,)+shape and np.isfinite(arrays[key]).all()
 ns=arrays['native_time_ns'];assert ns.dtype.kind in 'iu';steps=np.unique(np.diff(ns));assert len(steps)==1 and int(steps[0])>0 and steps.tolist()==payload['actual_step_ns_unique'];dt=int(steps[0])
 assert np.array_equal(arrays['time_s'],ns/1e9)
 if history['native_time_ns']:assert int(ns[0])-int(history['native_time_ns'][-1])==history['_actual_native_step_ns']==dt
 if '_configured_timestep_s' in history:assert history['_configured_timestep_s']==payload['configured_timestep_s']
 for key,source in [('contact_telemetry','contact_substeps'),('controller_command','controller_command_substeps'),('motor_readback','actual_motor_effort_substeps')]:assert np.array_equal(arrays[key],np.asarray(executed[source]))
 assert np.array_equal(arrays['actual_q'][-1],np.asarray(executed['observation']['actual_q'])) and np.array_equal(arrays['object_base_pose'][-1],np.asarray(executed['observation']['object_base_pose']))
 for key in FIELDS:history[key].extend(arrays[key].tolist())
 history['_actual_native_step_ns']=dt;history['_configured_timestep_s']=float(payload['configured_timestep_s'])
def strict_control_metrics(history,geometry):
 if not history['native_time_ns']:return {'strict_pass':False,'complete_full1s_window':False,'available_policy_samples':0,'reason':'No complete returned actual native frame'}
 if int(history['native_time_ns'][-1])-int(history['native_time_ns'][0])<1_000_000_000:return {'strict_pass':False,'complete_full1s_window':False,'available_policy_samples':len(history['native_time_ns']),'reason':'Insufficient actual native duration for full second'}
 return full1s_grasp_lift_metrics_ns(history['native_time_ns'],history['object_base_pose'],history['contact_telemetry'],history['_actual_native_step_ns'],geometry)
def control_trace_arrays(history):
 result={'native_control_'+key:np.asarray(history[key],dtype=np.int64 if key=='native_time_ns' else np.float64).reshape((-1,)+shape) for key,shape in FIELDS.items()}
 result['native_control_actual_step_ns']=np.asarray(history.get('_actual_native_step_ns',0),dtype=np.int64);result['native_control_configured_timestep_s']=np.asarray(history.get('_configured_timestep_s',0.),dtype=np.float64)
 return result

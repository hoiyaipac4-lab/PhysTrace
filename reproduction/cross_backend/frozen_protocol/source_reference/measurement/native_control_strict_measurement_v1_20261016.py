"""Read-only parser and full1s measurement for original MuJoCo control RPC."""
import numpy as np
from strict_native_full1s_metric_v1_20261016 import full1s_grasp_lift_metrics
FIELDS={'time_s':(), 'object_base_pose':(4,4), 'actual_q':(8,), 'motor_readback':(8,), 'controller_command':(8,), 'contact_telemetry':(8,)}
def new_control_history():return {k:[] for k in FIELDS}
def append_control_history(history,executed):
 payload=executed['native_control_readback'];rows=payload['rows'];assert payload['sampling_hz']==960 and len(rows)==32
 arrays={k:np.asarray([r[k] for r in rows],dtype=np.float64) for k in FIELDS}
 for k,shape in FIELDS.items():assert arrays[k].shape==(32,)+shape and np.isfinite(arrays[k]).all()
 assert np.allclose(np.diff(arrays['time_s']),1/960,rtol=0,atol=1e-9)
 if history['time_s']:assert abs(arrays['time_s'][0]-history['time_s'][-1]-1/960)<1e-9
 assert np.array_equal(arrays['contact_telemetry'],np.asarray(executed['contact_substeps'])) and np.array_equal(arrays['controller_command'],np.asarray(executed['controller_command_substeps']))
 assert np.array_equal(arrays['motor_readback'],np.asarray(executed['actual_motor_effort_substeps']))
 assert np.array_equal(arrays['actual_q'][-1],np.asarray(executed['observation']['actual_q'])) and np.array_equal(arrays['object_base_pose'][-1],np.asarray(executed['observation']['object_base_pose']))
 if '_solver_timestep_s' in history:assert history['_solver_timestep_s']==payload['actual_solver_timestep_s']
 for k in FIELDS:history[k].extend(arrays[k].tolist())
 history['_solver_timestep_s']=float(payload['actual_solver_timestep_s'])
def strict_control_metrics(history,geometry):
 if len(history['time_s'])<961:return {'strict_pass':False,'complete_full1s_window':False,'available_policy_samples':len(history['time_s']),'reason':'Insufficient endpointinclusive actual control readbacks'}
 return full1s_grasp_lift_metrics(history['time_s'],history['object_base_pose'],history['contact_telemetry'],960,geometry)
def control_trace_arrays(history):
 result={'native_control_'+k:np.asarray(history[k],dtype=np.float64).reshape((-1,)+shape) for k,shape in FIELDS.items()}
 result['native_control_solver_timestep_s']=np.asarray(history.get('_solver_timestep_s',0.),dtype=np.float64)
 return result

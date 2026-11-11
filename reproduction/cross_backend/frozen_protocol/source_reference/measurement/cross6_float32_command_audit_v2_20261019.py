"""Read-only wrapper around the frozen runtime; forwarded setters are never changed."""
import numpy as np

class SetterObserver:
    def __init__(self, native):
        self.native=native; self.position=None; self.feedforward=None
    def __getattr__(self, name):return getattr(self.native,name)
    def set_dof_position_targets(self, values, indices):
        result=self.native.set_dof_position_targets(values,indices)
        self.position=np.asarray(values).copy()
        return result
    def set_dof_actuation_forces(self, values, indices):
        result=self.native.set_dof_actuation_forces(values,indices)
        self.feedforward=np.asarray(values).copy()
        return result

class StepObserver:
    def __init__(self, rt):
        self.rt=rt; self.native=rt.robot; self.old_step=rt.step
        self.proxy=SetterObserver(self.native);rt.robot=self.proxy;rt.step=self.step
        self.reset_records()
    def reset_records(self):
        self.frame=-1;self.micro=-1;self.rows=[]
    def step(self, target, phase):
        rt=self.rt;t0=float(rt.t);before=rt.command[0,rt.idx].copy();error=None
        self.proxy.position=None;self.proxy.feedforward=None
        try:return self.old_step(target,phase)
        except BaseException as exc:error=str(exc);raise
        finally:
            if float(rt.t)>t0:
                row={'t_start':t0,'t_end':float(rt.t),'frame':self.frame,'micro':self.micro,
                     'command_before':before,'command_after':rt.command[0,rt.idx].copy(),
                     'sent_position':self.proxy.position[0,rt.idx].copy(),
                     'sent_feedforward':self.proxy.feedforward[0,rt.idx].copy(),
                     'q_after':rt.q(),'qvel_after':self.native.get_dof_velocities()[0,rt.idx].copy(),
                     'position_target_readback':self.native.get_dof_position_targets()[0,rt.idx].copy(),
                     'explicit_actuation_force_readback':self.native.get_dof_actuation_forces()[0,rt.idx].copy(),
                     'incoming_joint_projected_force':self.native.get_dof_projected_joint_forces()[0,rt.idx].copy(),
                     'link_world_pos_xyzw':self.native.get_link_transforms()[0].copy(),
                     'object_base_pose':rt.object_pose(), 'contact':rt.last.copy(),
                     'contact_valid':error is None or error.startswith('Native contact guard:'),
                     'returned':error is None,'error':error or ''}
                self.rows.append(row)
    def arrays(self):
        if not self.rows:return {}
        return {'native_'+k:np.asarray([r[k] for r in self.rows]) for k in self.rows[0]}
    def detach(self):self.rt.robot=self.native;self.rt.step=self.old_step

def schema():
    return {'version':1,'frequency_hz':960,
      'native_sent_position':'Exact copied values forwarded to native set_dof_position_targets; joint1..6 rad, joint7..8 m',
      'native_sent_feedforward':'Exact values forwarded to native set_dof_actuation_forces, here explicit native gravity compensation; Nm then N',
      'native_incoming_joint_projected_force':'SDK get_dof_projected_joint_forces: incoming link joint force projected into joint motion direction; NOT isolated drive motor effort or hardware sensor',
      'native_explicit_actuation_force_readback':'SDK get_dof_actuation_forces; do not infer implicit PD drive effort from this alone',
      'native_link_world_pos_xyzw':'Native articulation get_link_transforms: world position m and quaternion xyzw, link order stored separately',
      'native_t_start/native_t_end':'Absolute simulation seconds; post-step state at t_end',
      'camera_base_pose':'T_robot_base_from_OpenCV_camera, recorded at each RGB observation before its 32 substeps',
      'contact_valid':'False if original runtime raised before refreshing rt.last; stale contact samples must not be included in recomputed contact metrics',
      'native_returned':'Original rt.step returned normally; failed native step still retained if physics time advanced',
      'model_times':'Simulation observation time and wall-clock request start/end, kept separately',
      'no_robot_commands':True,'does_not_change_model_physics_guards_or_setter_arguments':True}

def audit_sent_commands(arrays,dt,limits):
    a=arrays;before=a['native_command_before'];after=a['native_command_after']
    assert len(before)>0 and all(np.isfinite(v).all() for k,v in a.items() if v.dtype.kind in 'fc')
    assert np.array_equal(after,a['native_sent_position'])
    assert np.array_equal(after,a['native_position_target_readback'])
    assert np.array_equal(a['native_sent_feedforward'],a['native_explicit_actuation_force_readback'])
    delta=np.abs(after.astype(float)-before.astype(float));arm=float(np.max(delta[:,:6]));jaw=float(np.max(delta[:,6:]))
    # The frozen controller computes its bounded target in float64 and stores
    # the command in float32. IEEE round-to-nearest is monotone: its allowed
    # stored endpoints come from rounding before +/- the SAME nominal limit.
    # A bound using spacing(before) alone is invalid when magnitude grows
    # through zero. No controller, rate, dt, force or task threshold is changed.
    assert before.dtype==np.float32 and after.dtype==np.float32
    before64=before.astype(np.float64);after64=after.astype(np.float64)
    nominal=np.r_[np.full(6,.30),np.full(2,.002)]*float(dt)
    # Eight float64 epsilon units bound division/multiplication/add/subtract
    # in the existing limiter. This is arithmetic roundoff, not a rate change.
    algebraic_roundoff64=8*np.finfo(np.float64).eps*(np.abs(before64)+nominal)
    rounded_lower=(before64-nominal-algebraic_roundoff64).astype(np.float32)
    rounded_upper=(before64+nominal+algebraic_roundoff64).astype(np.float32)
    valid=(after64>=rounded_lower.astype(np.float64))&(after64<=rounded_upper.astype(np.float64))
    if not np.all(valid):
        i,j=map(int,np.argwhere(~valid)[0])
        raise AssertionError(('FLOAT32_NOMINAL_RATE_ENVELOPE',{'row':i,'joint':j,
            'before':float(before64[i,j]),'after':float(after64[i,j]),
            'nominal_increment':float(nominal[j]),'delta':float(delta[i,j]),
            'allowed_stored_lower':float(rounded_lower[i,j]),
            'allowed_stored_upper':float(rounded_upper[i,j]),
            'algebraic_roundoff64':float(algebraic_roundoff64[i,j])}))
    tolerance=np.maximum(0.,np.maximum(rounded_upper.astype(np.float64)-before64-nominal,
                                      before64-rounded_lower.astype(np.float64)-nominal))
    assert np.all(after>=limits[:,0]-1e-5) and np.all(after<=limits[:,1]+1e-5)
    assert np.allclose(a['native_t_end']-a['native_t_start'],dt,rtol=0,atol=1e-10)
    return {'steps':len(before),'sent_equals_native_target_readback':True,'explicit_FF_equals_readback':True,
            'max_arm_command_increment_rad':arm,'max_each_finger_increment_m':jaw,
            'maximum_float32_increment_audit_tolerance_per_joint':tolerance.max(axis=0).tolist(),
            'projected_incoming_force_is_not_isolated_motor_effort':True,'audit_rule':'nearest_float32_endpoints_of_unchanged_nominal_rate_interval','nominal_arm_component_rad_s':.30,'nominal_each_finger_m_s':.002,'float64_algebraic_roundoff_factor':8,'control_runtime_unchanged':True}

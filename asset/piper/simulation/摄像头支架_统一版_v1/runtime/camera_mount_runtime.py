"""Runtime integration for the reconstructed camera mount.

Call before_step(dt) before native physics and after_step() after physics at
display frequency. Parameter estimates are supplied as data, never treated as
measurements. Force anchors are body-local; placing the root in another scene
therefore transforms the forces and visual geometry consistently.
Default holding gains target closed_loop_articulation_candidate.usda. These
are virtual test/holding controls, not measured friction or physical motors.
"""
from pathlib import Path
import json,numpy as np
from pxr import UsdPhysics
from spring_force_model import SpringForceModel
from spring_visual_follow import SpringVisualFollow

DRIVEN_JOINTS=['BaseSwivel','joint_base_0_internal_shaft','lock_pivot_elbow_internal_shaft','lock_pivot_camera_internal_shaft','CameraRoll']

def estimated_spring_settings(bindings):
    rows=[]
    for spec in bindings:
        length=float(np.linalg.norm(np.array(spec['anchor_end_m'])-spec['anchor_start_m']))
        turns=max(1,round((length-.020)/.0018));stiffness=79e9*.0011**4/(8*.007**3*turns)
        rows.append(dict(spec,stiffness_N_m=stiffness,damping_N_s_m=2.,rest_length_m=length,initial_tension_N=0.,active_coils_estimated=turns,parameter_status='Estimated 1.1mm wire, 7mm mean coil diameter and 79GPa shear modulus. Zero initial preload is a configurable default, not measured. Axial damping 2 N s/m is a provisional simulation value.'))
    return rows

class CameraMountRuntime:
    def __init__(self,stage,settings,root_path='/CameraMount',control_mode='hold',control_gains=(100.,10.,5.)):
        self.stage=stage;self.root=root_path.rstrip('/');self.settings=[];self.control_gains=control_gains
        for spec in settings:
            row=dict(spec)
            for field in ['mesh','body_start','body_end']:row[field]=self.root+spec[field][len('/CameraMount'):]
            self.settings.append(row)
        self.forces=SpringForceModel(stage,self.settings);self.visual=SpringVisualFollow(stage,self.settings)
        self.set_control_mode(control_mode)

    def set_control_mode(self,mode):
        assert mode in ['hold','free']
        for name in DRIVEN_JOINTS:
            drive=UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.root+'/Joints/'+name),'angular')
            drive.GetStiffnessAttr().Set(self.control_gains[0] if mode=='hold' else 0.)
            drive.GetDampingAttr().Set(self.control_gains[1] if mode=='hold' else .02)
            drive.GetMaxForceAttr().Set(self.control_gains[2])
        self.mode=mode

    def set_targets(self,targets):
        for name,degrees in targets.items():
            if name not in DRIVEN_JOINTS:raise ValueError(name)
            if not np.isfinite(degrees):raise ValueError('Non-finite joint target')
            UsdPhysics.DriveAPI(self.stage.GetPrimAtPath(self.root+'/Joints/'+name),'angular').GetTargetPositionAttr().Set(float(degrees))

    def before_step(self,dt):return self.forces.apply(dt)
    def after_step(self):return self.visual.update()
    def reset(self):self.forces.reset();self.visual.restore()

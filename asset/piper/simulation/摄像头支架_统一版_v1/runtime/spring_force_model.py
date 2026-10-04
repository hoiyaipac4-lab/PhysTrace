"""Explicit tension springs between the scan-derived anchor frames.

Hookean extension with axial damping, equal/opposite world forces. Parameter
estimates remain in the calling configuration; they are not measurements.
"""
import numpy as np
from pxr import Usd,UsdGeom,UsdUtils,PhysicsSchemaTools,Gf
import carb
import omni.physx

class SpringForceModel:
    def __init__(self,stage,settings):
        self.stage=stage;self.stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.interface=omni.physx.get_physx_simulation_interface();self.rows=[]
        for spec in settings:
            row=dict(spec)
            for key in ['stiffness_N_m','damping_N_s_m','rest_length_m','initial_tension_N']:
                value=float(row.get(key,0.0))
                if not np.isfinite(value) or value<0 or (key=='rest_length_m' and value==0):
                    raise ValueError('Invalid spring parameter: '+key)
            row['body_ids']=[PhysicsSchemaTools.sdfPathToInt(spec[k]) for k in ['body_start','body_end']]
            row['last_length']=None;self.rows.append(row)

    def apply(self,dt):
        assert dt>0
        records=[]
        for row in self.rows:
            positions=[]
            for body,anchor in [('body_start','anchor_start_m'),('body_end','anchor_end_m')]:
                matrix=UsdGeom.Xformable(self.stage.GetPrimAtPath(row[body])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                positions.append(np.array(matrix.Transform(Gf.Vec3d(*row[anchor]))))
            a,b=positions;delta=b-a;length=float(np.linalg.norm(delta))
            assert np.isfinite(length) and length>1e-6
            rate=0.0 if row['last_length'] is None else (length-row['last_length'])/dt
            row['last_length']=length
            tension=max(0.0,row.get('initial_tension_N',0.0)+row['stiffness_N_m']*(length-row['rest_length_m'])+row['damping_N_s_m']*rate)
            force=tension*delta/length
            for encoded,position,sign in zip(row['body_ids'],positions,[1,-1]):
                if tension == 0.0:
                    continue  # No impulse; also avoids touching a not-yet-inserted link on initialization.
                self.interface.apply_force_at_pos(self.stage_id,encoded,carb.Float3(*(sign*force)),carb.Float3(*position),'Force')
            records.append({'name':row['name'],'length_m':length,'extension_m':length-row['rest_length_m'],'length_rate_m_s':rate,'tension_N':tension})
        return records

    def reset(self):
        for row in self.rows:row['last_length']=None

from pathlib import Path
import json,numpy as np
from pxr import UsdGeom,UsdPhysics,Gf,Sdf
ROOT=Path(__file__).resolve().parent
def load_config(path=None):
 c=json.loads((ROOT/'training_config.json').read_text())
 if path:c.update(json.loads(Path(path).read_text()))
 if c['physics_hz']<=0 or c['control_hz']<=0 or c['physics_hz']%c['control_hz']:raise ValueError('Invalid physics/control frequencies')
 if len(c['resolution'])!=2 or any(int(x)!=x or x<32 for x in c['resolution']):raise ValueError('Invalid image resolution')
 return c
def apply_config(stage,c):
 assert UsdGeom.GetStageMetersPerUnit(stage)==1. and UsdGeom.GetStageUpAxis(stage)=='Z'
 if stage.GetPrimAtPath('/World/TaskAssets/plate').IsActive():
  UsdPhysics.Joint(stage.GetPrimAtPath('/World/TaskAssets/plate/TableAttachment')).GetJointEnabledAttr().Set(bool(c['plate_fixed']))
 hand=UsdGeom.Xformable(stage.GetPrimAtPath(c['hand_camera']));hand.MakeMatrixXform().Set(Gf.Matrix4d(*np.asarray(c['wrist_local_matrix']).reshape(-1).tolist()))
 for p in stage.Traverse():
  if p.IsA(UsdPhysics.Scene):
   p.CreateAttribute('physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt).Set(c['physics_hz'])
 for name,values in c.get('material_overrides',{}).items():
  prim=stage.GetPrimAtPath('/World/TaskAssets/'+name+'/Contact')
  if not prim:raise ValueError('Unknown material object '+name)
  mat=UsdPhysics.MaterialAPI(prim)
  for key,api in [('static_friction',mat.GetStaticFrictionAttr()),('dynamic_friction',mat.GetDynamicFrictionAttr()),('restitution',mat.GetRestitutionAttr())]:
   if key in values:
    v=float(values[key]);assert np.isfinite(v) and v>=0 and (key!='restitution' or v<=1);api.Set(v)
  if 'friction_combine_mode' in values:
   mode=values['friction_combine_mode'];assert mode in ['average','min','multiply','max'];prim.AddAppliedSchema('PhysxMaterialAPI');prim.CreateAttribute('physxMaterial:frictionCombineMode',Sdf.ValueTypeNames.Token).Set(mode)

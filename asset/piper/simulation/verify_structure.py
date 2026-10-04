from pathlib import Path
import json,sys,numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960})
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
root=Path(__file__).resolve().parent
s=Usd.Stage.Open(str(root/'scene.usda'))
def val(v):
 if v is None or isinstance(v,(str,int,float,bool)):return v
 try:return list(v)
 except:return str(v)
def attrs(p):return {a.GetName():val(a.Get()) for a in p.GetAttributes() if a.GetName().startswith(('physics:','physx'))}
r={'bodies':[],'colliders':[],'joints':[],'filters':[],'articulations':[]}
for p in s.Traverse():
 d={'path':str(p.GetPath()),'type':p.GetTypeName(),'attrs':attrs(p)}
 if p.HasAPI(UsdPhysics.RigidBodyAPI):r['bodies'].append(d)
 if p.HasAPI(UsdPhysics.CollisionAPI):
  a=p
  while a and not a.HasAPI(UsdPhysics.RigidBodyAPI):a=a.GetParent()
  d['body']=str(a.GetPath()) if a else None
  r['colliders'].append(d)
 if p.IsA(UsdPhysics.Joint):
  d['rels']={x.GetName():[str(t) for t in x.GetTargets()] for x in p.GetRelationships()};r['joints'].append(d)
 if p.HasAPI(UsdPhysics.ArticulationRootAPI):r['articulations'].append(d)
 if p.HasAPI(UsdPhysics.FilteredPairsAPI):r['filters'].append({'path':str(p.GetPath()),'targets':[str(t) for t in UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets()]})
(root/'evidence/structural_audit.json').write_text(json.dumps(r,indent=2,default=str))
print('AUDIT_COUNTS',{k:len(v) for k,v in r.items()},flush=True)

app.close()

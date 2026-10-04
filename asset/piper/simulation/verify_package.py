"""Reopen a relocated package, checking dependencies and final material bindings."""
from pathlib import Path
import argparse,json,hashlib
parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parent);parser.add_argument('--report',type=Path);args=parser.parse_args();root=args.root.resolve()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960})
try:
 from pxr import Usd,UsdUtils,UsdGeom,UsdShade,UsdPhysics
 stage=Usd.Stage.Open(str(root/'scene.usda'));assert stage
 layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(root/'scene.usda'))
 external=[];runtime_modules=[]
 for item in [l.realPath for l in layers]+list(assets):
  if item and not Path(item).resolve().is_relative_to(root):
   if Path(item).name in ('OmniPBR.mdl','OmniPBR_Opacity.mdl') and '/kit/mdl/core/Base/' in item:runtime_modules.append(Path(item).name)
   else:external.append(item)
 assert not unresolved,unresolved
 assert not external,external
 assert UsdGeom.GetStageMetersPerUnit(stage)==1
 assert UsdGeom.GetStageUpAxis(stage)=='Z'
 material_checks=[]
 for name in ('microwave','airfryer'):
  for p in Usd.PrimRange(stage.GetPrimAtPath('/World/TaskAssets/'+name)):
   if p.IsA(UsdGeom.Mesh) and '/visuals/' in str(p.GetPath()):
    m=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
    assert str(m.GetPath())=='/World/TaskAssets/'+name+'/OriginalAtlas',(str(p.GetPath()),str(m.GetPath()))
    assert UsdGeom.PrimvarsAPI(p).GetPrimvar('st').Get()
    material_checks.append(str(p.GetPath()))
 invalid=[]
 for p in stage.Traverse():
  if p.IsA(UsdPhysics.Joint) and UsdPhysics.Joint(p).GetJointEnabledAttr().Get():
   for rel in (UsdPhysics.Joint(p).GetBody0Rel(),UsdPhysics.Joint(p).GetBody1Rel()):
    for target in rel.GetTargets():
     if not stage.GetPrimAtPath(target):invalid.append(str(target))
 assert not invalid,invalid
 result={'status':'PASS','layers':len(layers),'assets':len(assets),'unresolved':unresolved,'external_dependencies':external,'isaac_builtin_material_modules':runtime_modules,'textured_appliance_meshes':len(material_checks),'prims':sum(1 for _ in stage.Traverse()),'invalid_joint_targets':invalid,'scene_sha256':hashlib.sha256((root/'scene.usda').read_bytes()).hexdigest(),'scope':'Relocated USD reopen, dependency closure, units, joint targets and effective appliance materials; not dynamic or real robot certification'}
 target=args.report or root/'evidence/package_reopen.json';target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(result,indent=2));print('PACKAGE_REOPEN_PASS',json.dumps(result),flush=True)
except BaseException:
 import traceback;traceback.print_exc();print('PACKAGE_REOPEN_FAILED',flush=True);raise
finally:app.close()

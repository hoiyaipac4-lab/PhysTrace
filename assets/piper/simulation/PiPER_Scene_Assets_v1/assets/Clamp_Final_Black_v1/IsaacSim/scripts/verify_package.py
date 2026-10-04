"""Reopen a delivered package in Isaac Sim and resolve every USD dependency."""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();r=a.root.resolve()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'limit_cpu_threads':8,'extra_args':['--/app/renderer/enabled=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false']})
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,UsdUtils,PhysxSchema,Sdf
try:
 s=Usd.Stage.Open(str(r/'Isaac_CheckScene.usda'));layers,assets,missing=UsdUtils.ComputeAllDependencies(Sdf.AssetPath(str(r/'Isaac_CheckScene.usda')))
 colors={n:list(UsdShade.Shader(s.GetPrimAtPath('/World/Clamp/Black/Shader')).GetInput('diffuseColor').Get()) for n in ['frame','screw','pad','handle']}
 bodies=[str(p.GetPath()) for p in s.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
 limits=UsdPhysics.PrismaticJoint(s.GetPrimAtPath('/World/Clamp/Joints/HandleSlide'))
 checks={'stage_open':bool(s),'no_unresolved_dependencies':not missing,'meter_units':UsdGeom.GetStageMetersPerUnit(s)==1,'default_prim':bool(s.GetDefaultPrim()),'all_bodies':all('/World/Clamp/'+n in bodies for n in ['frame','screw','pad','handle','carriage']),'black_material':all(abs(v-.006)<1e-6 for c in colors.values() for v in c),'handle_retention_limits':abs(limits.GetLowerLimitAttr().Get()+.045)<1e-7 and abs(limits.GetUpperLimitAttr().Get()-.008)<1e-7,'rate_960':PhysxSchema.PhysxSceneAPI(s.GetPrimAtPath('/World/Physics')).GetTimeStepsPerSecondAttr().Get()==960}
 report={'status':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'unresolved':missing,'resolved_layers':[str(x.identifier) for x in layers],'asset_sha256':hashlib.sha256((r/'Clamp.usdc').read_bytes()).hexdigest()};(r/'evidence/package_reopen.json').write_text(json.dumps(report,indent=2),encoding='utf8');print(report,flush=True)
finally:app.close()

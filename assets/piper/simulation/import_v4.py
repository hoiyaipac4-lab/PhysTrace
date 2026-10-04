from pathlib import Path
import json
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960})
try:
 import omni.kit.commands
 from pxr import Usd,UsdGeom,UsdPhysics
 root=Path(__file__).resolve().parent;cat=json.loads((root/'source_v4/asset_catalog.json').read_text());out=root/'task_assets';out.mkdir(exist_ok=True);report=[]
 for name in ['cube_wood','cube_aluminum','cube_iron','cube_brass','microwave','airfryer','plate','support_microwave','support_airfryer']:
  src=root/'source_v4'/cat[name]['urdf'];dest=out/(name+'.usd')
  ok,cfg=omni.kit.commands.execute('URDFCreateImportConfig');cfg.fix_base=False;cfg.merge_fixed_joints=True;cfg.import_inertia_tensor=True;cfg.create_physics_scene=False;cfg.make_default_prim=True
  ok,result=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=str(src),import_config=cfg,dest_path=str(dest));assert ok,(name,result)
  stage=Usd.Stage.Open(str(dest));default=stage.GetDefaultPrim();b=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(default).ComputeAlignedRange();rec={'name':name,'default_prim':str(default.GetPath()),'bounds':[list(b.GetMin()),list(b.GetMax())],'bodies':[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)],'joints':[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]};report.append(rec);print('IMPORTED',rec,flush=True)
 (root/'evidence/task_assets_import.json').write_text(json.dumps(report,indent=2))
except BaseException:
 import traceback;traceback.print_exc();print('IMPORT_FAILED',flush=True)
finally:app.close()

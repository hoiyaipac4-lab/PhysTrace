"""Run with Isaac Sim's python.bat. Native stepping supplies spring forces each step."""
from pathlib import Path
import argparse,sys,json,math
import numpy as np
base=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--headless',action='store_true');parser.add_argument('--motion',action='store_true');parser.add_argument('--steps',type=int,default=2400);parser.add_argument('--scene',default=str(base/'preview.usda'));parser.add_argument('--root',default='/CameraMount');parser.add_argument('--capture',action='store_true');parser.add_argument('--verify',action='store_true');args=parser.parse_args();assert args.steps>0
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.headless,'width':1920,'height':1200,'active_gpu':0,'multi_gpu':False,'extra_args':['--/exts/isaacsim.physics.newton/auto_switch_on_startup=false']})
from pxr import Usd,UsdGeom,UsdPhysics,UsdUtils,Gf
import omni.usd,omni.physics.core
sys.path.insert(0,str(base/'runtime'))
from camera_mount_runtime import CameraMountRuntime,DRIVEN_JOINTS
omni.usd.get_context().open_stage(str(Path(args.scene).resolve()))
for _ in range(20):app.update()
stage=omni.usd.get_context().get_stage();stage.SetEditTarget(stage.GetSessionLayer());runtime=CameraMountRuntime(stage,json.loads((base/'runtime/spring_settings.json').read_text(encoding='utf-8')),root_path=args.root)
if not args.headless:
 from omni.kit.viewport.utility import get_active_viewport
 viewport=get_active_viewport()
 if viewport:viewport.camera_path='/ReviewCamera'
joints=[UsdPhysics.Joint(q) for q in Usd.PrimRange(stage.GetPrimAtPath(args.root)) if q.IsA(UsdPhysics.Joint) and q.GetName()!='WorldClamp'];sim=omni.physics.core.get_physics_simulation_interface();sim.initialize(UsdUtils.StageCache.Get().GetId(stage).ToLongInt());records=[];dt=1/240;amplitudes=dict(zip(DRIVEN_JOINTS,[8,3,3,6,12]));steps=0
for k in range(args.steps):
 if not app.is_running():break
 t=k*dt;phase=math.sin((t-1)*math.pi/3) if args.motion and 1<=t<7 else 0;targets={n:a*phase for n,a in amplitudes.items()};runtime.set_targets(targets);forces=runtime.before_step(dt);sim.simulate(dt,t);steps+=1
 if k%24==0 or k==args.steps-1:
  visuals=runtime.after_step();assert all(r['finite'] for r in visuals);gaps={};angles={}
  for j in joints:
   matrices=[UsdGeom.Xformable(stage.GetPrimAtPath(rel.GetTargets()[0])).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) for rel in [j.GetBody0Rel(),j.GetBody1Rel()]];local=[Gf.Vec3d(j.GetLocalPos0Attr().Get()),Gf.Vec3d(j.GetLocalPos1Attr().Get())];gaps[j.GetPrim().GetName()]=float((matrices[0].Transform(local[0])-matrices[1].Transform(local[1])).GetLength());name=j.GetPrim().GetName()
   if name in DRIVEN_JOINTS:
    rotations=[Gf.Rotation(Gf.Quatd(a.Get())) for a in [j.GetLocalRot0Attr(),j.GetLocalRot1Attr()]];u=[np.array(m.TransformDir(q.TransformDir(Gf.Vec3d(0,1,0)))) for m,q in zip(matrices,rotations)];axis=np.array(matrices[0].TransformDir(rotations[0].TransformDir(Gf.Vec3d(1,0,0))));angles[name]=float(np.rad2deg(np.arctan2(axis@np.cross(u[0],u[1]),u[0]@u[1])))
  records.append({'time_s':t,'targets_deg':targets,'angles_deg':angles,'anchor_gaps_m':gaps,'max_tension_N':max(r['tension_N'] for r in forces)})
 if k%4==0:app.update()
report={'steps':steps,'dt_s':dt,'motion_demo':args.motion,'max_anchor_gap_m':max(max(r['anchor_gaps_m'].values()) for r in records),'final_tracking_errors_deg':{n:records[-1]['angles_deg'][n]-records[-1]['targets_deg'][n] for n in DRIVEN_JOINTS},'max_tension_N':max(r['max_tension_N'] for r in records),'scope':'Packaged runtime limited trajectory, estimated spring and mass parameters. Not full workspace, real stiffness or final scan-reconstruction acceptance.','samples':records}
(base/'checks/runtime_entry_test.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print('RUNTIME_RESULT',json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)
if args.verify:
 assert steps==args.steps and report['max_anchor_gap_m']<.0001
 assert max(abs(x) for x in report['final_tracking_errors_deg'].values())<.1
if args.capture:
 import omni.replicator.core as rep
 from PIL import Image
 product=rep.create.render_product('/ReviewCamera',(1920,1200));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 for _ in range(15):rep.orchestrator.step(rt_subframes=2)
 pixels=rgb.get_data();assert isinstance(pixels,np.ndarray) and pixels.shape[:2]==(1200,1920) and pixels[:,:,:3].std()>1;Image.fromarray(pixels).save(base/'整体预览.png')
app.close()


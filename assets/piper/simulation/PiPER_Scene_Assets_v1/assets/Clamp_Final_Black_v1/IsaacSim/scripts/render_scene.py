"""Create a portable inspection scene and capture measured PhysX joint poses."""
import argparse,json,math
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);ap.add_argument('--reuse-measurements',action='store_true');a=ap.parse_args();root=a.root.resolve()
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1600,'height':1200,'renderer':'RayTracedLighting','limit_cpu_threads':8,'multi_gpu':False,'extra_args':['--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
import numpy as np,omni.usd,omni.physx,omni.physics.tensors,omni.replicator.core as rep,carb
from pxr import Usd,UsdGeom,UsdLux,UsdPhysics,UsdShade,PhysxSchema,Gf,Sdf
from PIL import Image
p=json.loads((root/'parameters.json').read_text());ctx=omni.usd.get_context();s=Usd.Stage.CreateNew(str(root/'Isaac_CheckScene.usda'));UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1);w=UsdGeom.Xform.Define(s,'/World');s.SetDefaultPrim(w.GetPrim())
ps=UsdPhysics.Scene.Define(s,'/World/Physics');ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ps.CreateGravityMagnitudeAttr(9.81);px=PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim());px.CreateEnableGPUDynamicsAttr(True);px.CreateBroadphaseTypeAttr('GPU');px.CreateSolverTypeAttr('TGS');px.CreateTimeStepsPerSecondAttr(960);px.CreateEnableCCDAttr(True);px.CreateEnableExternalForcesEveryIterationAttr(True)
xf=UsdGeom.Xform.Define(s,'/World/Clamp');xf.GetPrim().GetReferences().AddReference('./Clamp.usdc');xf.AddTranslateOp().Set(Gf.Vec3d(0,0,.003))
j=UsdPhysics.FixedJoint.Define(s,'/World/InspectionFixture');j.CreateBody1Rel().SetTargets(['/World/Clamp/frame']);j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(p['part_properties']['frame']['center'])+[0,0,.003])))
ground=UsdGeom.Cube.Define(s,'/World/Ground');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(0,0,-.005));ground.AddScaleOp().Set(Gf.Vec3f(2,2,.01));ground.CreateDisplayColorAttr([Gf.Vec3f(.35,.38,.42)]);UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
UsdLux.DomeLight.Define(s,'/World/DomeLight').CreateIntensityAttr(350);sun=UsdLux.DistantLight.Define(s,'/World/DistantLight');sun.CreateIntensityAttr(1400);sun.AddRotateXYZOp().Set(Gf.Vec3f(-30,30,20));fill=UsdLux.RectLight.Define(s,'/World/Softbox');fill.CreateIntensityAttr(800);fill.CreateWidthAttr(.3);fill.CreateHeightAttr(.3);fill.AddTranslateOp().Set(Gf.Vec3d(-.15,.12,.3));fill.AddRotateXYZOp().Set(Gf.Vec3f(20,-30,0))
cam=UsdGeom.Camera.Define(s,'/World/Camera');cam.CreateFocalLengthAttr(30);cam.CreateClippingRangeAttr(Gf.Vec2f(.001,100));cam.AddTransformOp();target=Gf.Vec3d(0,0,.092)
def camera(eye):
 m=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),target,Gf.Vec3d(0,0,1)).GetInverse();UsdGeom.Xformable(cam).GetOrderedXformOps()[0].Set(m)
camera((.16,-.38,.19));s.GetRootLayer().Save();ctx.open_stage(str(root/'Isaac_CheckScene.usda'))
for _ in range(5):app.update()
s=ctx.get_stage();s.SetEditTarget(Usd.EditTarget(s.GetSessionLayer()));cam=UsdGeom.Camera(s.GetPrimAtPath('/World/Camera'));settings=carb.settings.get_settings();settings.set('/rtx/post/tonemap/op',4);settings.set('/rtx/post/tonemap/filmIso',200);settings.set('/rtx/post/aa/op',3);settings.set('/rtx/rendermode','RayTracedLighting')
rep.orchestrator.set_capture_on_play(False);rp=rep.create.render_product('/World/Camera',(1600,1200));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([rp]);
if a.reuse_measurements:
 snapshots=json.loads((root/'evidence/render_check.json').read_text())['measurements']
else:
 sim=omni.physx.get_physx_simulation_interface();assert sim.attach_stage(ctx.get_stage_id());dt=1/480;clock=0;sim.simulate(dt,clock);sim.fetch_results();clock+=dt;sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=ctx.get_stage_id());art=sv.create_articulation_view('/World/Clamp');names=list(art.shared_metatype.link_names);drive=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/ScrewTurn'),'angular');snapshots=[]
 for goal,label in [(0,'open'),(9,'half_closed'),(18,'closed'),(0,'reopened')]:
  start=drive.GetTargetPositionAttr().Get() or 0;targetdeg=goal/p['thread_pitch_estimated']*360
  for k in range(4*480):
   t=min(1,k/(3*480));t=t*t*(3-2*t);drive.GetTargetPositionAttr().Set(start+(targetdeg-start)*t);sim.simulate(dt,clock);sim.fetch_results();clock+=dt
  snapshots.append({'label':label,'q':art.get_dof_positions()[0].tolist(),'link_names':names,'link_poses':art.get_link_transforms()[0].tolist()})
 sim.detach_stage();art=sv=None
metrics=[]
for shot in snapshots:
 for n,pose in zip(shot['link_names'],shot['link_poses']):
  body=UsdGeom.Xformable(s.GetPrimAtPath('/World/Clamp/'+n));body.ClearXformOpOrder();body.AddTranslateOp().Set(Gf.Vec3d(*(np.array(pose[:3])-[0,0,.003])));body.AddOrientOp().Set(Gf.Quatf(pose[6],Gf.Vec3f(*pose[3:6])))
 for _ in range(8):app.update()
 rep.orchestrator.step(rt_subframes=16,pause_timeline=False);data=np.array(rgb.get_data())
 if data.size==0:
  for _ in range(40):app.update()
  rep.orchestrator.step(rt_subframes=32,pause_timeline=False);data=np.array(rgb.get_data())
 assert data.ndim==3 and data.shape[:2]==(1200,1600)
 path=root/'evidence'/('isaac_'+shot['label']+'.png');Image.fromarray(data[:,:,:3]).save(path);metrics.append({'name':path.name,'mean_rgb':float(data[:,:,:3].mean()),'std_rgb':float(data[:,:,:3].std()),'bytes':path.stat().st_size,'valid':bool(data[:,:,:3].mean()>30 and data[:,:,:3].std()>15 and path.stat().st_size>=150000)})
# Additional views of the same measured reopened pose.
for label,eye in [('back',(-.14,.38,.20)),('side',(.38,-.03,.16)),('top',(.04,-.07,.48))]:
 camera(eye)
 for _ in range(8):app.update()
 rep.orchestrator.step(rt_subframes=16,pause_timeline=False);data=np.array(rgb.get_data());path=root/'evidence'/('isaac_'+label+'.png');Image.fromarray(data[:,:,:3]).save(path);metrics.append({'name':path.name,'mean_rgb':float(data[:,:,:3].mean()),'std_rgb':float(data[:,:,:3].std()),'bytes':path.stat().st_size,'valid':bool(data[:,:,:3].mean()>30 and data[:,:,:3].std()>15 and path.stat().st_size>=150000)})
(root/'evidence/render_check.json').write_text(json.dumps({'measurements':snapshots,'images':metrics,'poses_from':'PhysX articulation tensor link transforms after simulation; replayed only for capture'},indent=2));print('RENDER',metrics,flush=True);app.close()


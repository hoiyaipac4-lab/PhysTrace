"""PiPER V4 task assets and lift desk. Fixtures and estimated properties are documented."""
from pathlib import Path
import argparse,json,time,sys
parser=argparse.ArgumentParser();parser.add_argument('--validate',action='store_true');parser.add_argument('--seconds',type=float,default=10.);parser.add_argument('--output-dir',type=Path);args=parser.parse_args()
root=Path(__file__).resolve().parent;out=args.output_dir or root/'evidence';out.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':args.validate,'width':1280,'height':960,'renderer':'RayTracedLighting','extra_args':['--/renderer/multiGpu/enabled=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false']})
try:
 import carb,omni.usd,omni.physx,numpy as np
 import omni.physics.tensors
 sys.path.insert(0,str(root/'PiPER_Scene_Assets_v1/scene'))
 sys.path.insert(0,str(root/'摄像头支架_统一版_v1/runtime'))
 from lift_controller import LiftController
 from camera_mount_runtime import CameraMountRuntime
 from movable_clamps import CLAMP_PATHS,PITCH_M
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 ctx=omni.usd.get_context();ctx.open_stage(str(root/'scene.usda'));stage=ctx.get_stage();stage.SetEditTarget(stage.GetSessionLayer())
 mount_runtime=CameraMountRuntime(stage,json.loads((root/'摄像头支架_统一版_v1/runtime/spring_settings.json').read_text()),root_path='/World/CameraMount')
 settings=carb.settings.get_settings();settings.set('/rtx/post/tonemap/op',4);settings.set('/rtx/post/tonemap/filmIso',200.)
 from omni.kit.viewport.utility import get_active_viewport
 vp=get_active_viewport()
 if vp:vp.set_active_camera('/World/Camera')
 import omni.replicator.core as rep
 rep.orchestrator.set_capture_on_play(False)
 product=rep.create.render_product('/World/Camera',(1280,960));rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach([product])
 for i in range(60):app.update()
 sim=omni.physx.get_physx_simulation_interface();assert sim.attach_stage(ctx.get_stage_id());dt=1/480;t=0.
 sim.simulate(dt,t);sim.fetch_results();t+=dt
 sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=ctx.get_stage_id());view=sv.create_articulation_view('/World/Desk/Joints/Anchor');control=LiftController(view)
 names=list(view.shared_metatype.dof_names);print('DOFS',names,flush=True)
 clamp_views=[sv.create_articulation_view(p) for p in CLAMP_PATHS]
 clamp_names=[list(v.shared_metatype.dof_names) for v in clamp_views]
 clamp_drives=[UsdPhysics.DriveAPI(stage.GetPrimAtPath(p+'/Joints/ScrewTurn'),'angular') for p in CLAMP_PATHS]
 print('CLAMP_DOFS',clamp_names,flush=True)
 clamp_turns=[0.,0.]
 appliance_drives={name:UsdPhysics.DriveAPI(next(p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/TaskAssets/'+name)) if p.IsA(UsdPhysics.Joint) and p.GetName().endswith('opening_joint')),'angular' if name=='microwave' else 'linear') for name in ['microwave','airfryer'] if stage.GetPrimAtPath('/World/TaskAssets/'+name).IsActive()}
 idx=[names.index('joint'+str(i)) for i in range(1,9)];indices=np.array([0],dtype=np.int32)
 targets=view.get_dof_positions().copy();home=np.array([0,.65,-.85,0,.2,0,.02,-.02]);targets[0,idx]=home
 view.set_dof_positions(targets,indices);view.set_dof_position_targets(targets,indices)
 samples=[];saved=set();reference=None
 if not args.validate:
  import omni.ui as ui
  window=ui.Window('PiPER Controls',width=390,height=780,flags=ui.WINDOW_FLAGS_NO_DOCKING)
  window.undock();window.position_x=1010;window.position_y=35
  with window.frame:
   with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
    with ui.VStack(spacing=5,height=0):
     with ui.VStack(height=112,spacing=2):
      for label,camera in [('Main','/World/Camera'),('Detail','/World/MountDetail'),('Overview','/World/Overview'),('Camera','/World/CameraMount/Links/CameraBody/WorkspaceCamera')]:
       ui.Button(label,height=26,clicked_fn=lambda camera=camera:vp.set_active_camera(camera))
     ui.Label('Table height');height=ui.FloatSlider(min=.635,max=1.235);height.model.set_value(.635);height.model.add_value_changed_fn(lambda m:control.set_height(m.as_float))
     sliders=[]
     for j,(lo,hi) in enumerate([(-150,150),(0,179),(-169,0),(-99,99),(-69,69),(-120,120)]):
      ui.Label('Joint '+str(j+1)+' (degrees)');slider=ui.FloatSlider(min=lo,max=hi);slider.model.set_value(float(np.degrees(home[j])));slider.model.add_value_changed_fn(lambda m,j=j:targets.__setitem__((0,idx[j]),np.radians(m.as_float)));sliders.append(slider)
     ui.Label('Gripper half opening (m)');grip=ui.FloatSlider(min=0,max=.035);grip.model.set_value(.02)
     def set_grip(m):targets[0,idx[6]]=m.as_float;targets[0,idx[7]]=-m.as_float
     grip.model.add_value_changed_fn(set_grip)
     for clamp_index,label in enumerate(['Left clamp','Right clamp']):
      ui.Label(label);slider=ui.FloatSlider(min=-2.,max=.25);slider.model.set_value(0.);slider.model.add_value_changed_fn(lambda m,i=clamp_index:clamp_turns.__setitem__(i,m.as_float));sliders.append(slider)
     for appliance,label,maximum in [('microwave','Microwave door',110.),('airfryer','Air fryer drawer',.16)]:
      if appliance not in appliance_drives: continue
      ui.Label(label);slider=ui.FloatSlider(min=0,max=maximum);slider.model.set_value(0);slider.model.add_value_changed_fn(lambda m,name=appliance:appliance_drives[name].GetTargetPositionAttr().Set(m.as_float));sliders.append(slider)
     readout=ui.Label('',height=24);ui.Label('Native simulation running. Do not press timeline Play.')
 if not args.validate:print('GUI_READY: combined scene and control panel loaded',flush=True)
 while app.is_running() and (not args.validate or t<args.seconds):
  start=time.perf_counter()
  if args.validate:
   control.set_height(.715 if 1<t<6 else .635)
   targets[0,idx[0]]=.12 if 2<t<6 else 0
   targets[0,idx[6]]=.03 if 2<t<6 else .02;targets[0,idx[7]]=-targets[0,idx[6]]
  if args.validate:
   clamp_turns[:]=[-.5 if 2<t<5 else 0.,-.5 if 3<t<6 else 0.]
  for i,drive in enumerate(clamp_drives):drive.GetTargetPositionAttr().Set(clamp_turns[i]*360.)
  for _ in range(16):
   control.tick(dt);command=view.get_dof_positions().copy();command[0,idx]=targets[0,idx]
   # Preserve lift target generated by LiftController.
   command[:,control.joints]=(control.command-control.minimum)/2
   view.set_dof_position_targets(command,indices)
   spring_forces=mount_runtime.before_step(dt)
   sim.simulate(dt,t);sim.fetch_results();t+=dt
   omni.physx.get_physx_interface().update_transformations(False,True,False)
  visuals=mount_runtime.after_step();assert all(v['finite'] for v in visuals)
  q=view.get_dof_positions();assert np.isfinite(q).all()
  gaps=[]
  for prim in stage.GetPrimAtPath('/World/CameraMount/Joints').GetChildren():
   joint=UsdPhysics.Joint(prim);rels=[joint.GetBody0Rel(),joint.GetBody1Rel()]
   mats=[UsdGeom.Xformable(stage.GetPrimAtPath(r.GetTargets()[0])).ComputeLocalToWorldTransform(0) for r in rels]
   anchors=[Gf.Vec3d(joint.GetLocalPos0Attr().Get()),Gf.Vec3d(joint.GetLocalPos1Attr().Get())]
   gaps.append(float((mats[0].Transform(anchors[0])-mats[1].Transform(anchors[1])).GetLength()))
  assert np.isfinite(gaps).all()
  optical_matrix=UsdGeom.Xformable(stage.GetPrimAtPath('/World/CameraMount/Links/CameraBody/WorkspaceCamera')).ComputeLocalToWorldTransform(0)
  optical_direction=np.array(optical_matrix.TransformDir(Gf.Vec3d(0,0,-1)).GetNormalized())
  optical_angles={'heading_deg':float(np.degrees(np.arctan2(optical_direction[1],-optical_direction[0]))),'downward_deg':float(np.degrees(np.arcsin(-optical_direction[2])))}
  clamp_state=[]
  for cv,cn in zip(clamp_views,clamp_names):
   cq=cv.get_dof_positions()[0];assert np.isfinite(cq).all()
   feed=float(cq[cn.index('ScrewFeed')]);turn=float(cq[cn.index('ScrewTurn')]);clamp_state.append({'feed_m':feed,'turn_rad':turn,'coupling_error_m':abs(feed-turn*PITCH_M/(2*np.pi))})
  if args.validate and len(samples)<int(t*10):
   samples.append({'camera_angles':optical_angles,'clamps':clamp_state,'mount_max_anchor_gap_m':max(gaps),'spring_max_tension_N':max(f['tension_N'] for f in spring_forces),'t':t,'height':control.measured_height(),'q':q[0].tolist(),'arm_error':float(np.max(np.abs(q[0,idx]-targets[0,idx]))),'link_poses':view.get_link_transforms()[0].tolist()})
  if not args.validate:
   omni.physx.get_physx_interface().update_transformations(False,True,False);app.update();readout.text=f'Tabletop {control.measured_height():.4f} m'
   delay=16*dt-(time.perf_counter()-start)
   if delay>0:time.sleep(delay)
  for sec,label in ([(.8,'initial'),(5.5,'raised'),(9.5,'returned')] if args.validate else []):
   if t>=sec and label not in saved:
    omni.physx.get_physx_interface().update_transformations(False,True,False)
    for _ in range(12):app.update()
    rep.orchestrator.step(delta_time=0.,pause_timeline=False)
    from PIL import Image
    data=np.asarray(rgb.get_data())[:,:,:3];Image.fromarray(data).save(out/(label+'.png'));saved.add(label);print('CAPTURE',label,t,control.measured_height(),float(data.mean()),flush=True)
 if args.validate:
  clamp_checks=[]
  for i in range(2):
   feeds=[sample['clamps'][i]['feed_m'] for sample in samples];errors=[sample['clamps'][i]['coupling_error_m'] for sample in samples]
   clamp_checks.append({'feed_range_m':[min(feeds),max(feeds)],'coupling_max_error_m':max(errors),'final_feed_m':feeds[-1]})
  report={'final_camera_angles':samples[-1]['camera_angles'],'clamp_checks':clamp_checks,'scope':'10-second lift and return, small robot joint motion, two half-turn screw loosen/return cycles, camera spring hold and 30/30-degree optical pose. Fixed assembly fixtures remain; no full friction holding, collision workspace, payload or real-camera calibration certification.','isaac_version':'5.1.0','mount_max_anchor_gap_m':max(s['mount_max_anchor_gap_m'] for s in samples),'dof_names':names,'link_names':list(view.shared_metatype.link_names),'samples':samples,'finite':True,'height_range':[min(s['height'] for s in samples),max(s['height'] for s in samples)]}
  (out/'runtime.json').write_text(json.dumps(report,indent=2));print('VALIDATION_DONE',report['height_range'],flush=True)
 if args.validate:
  assert report['mount_max_anchor_gap_m']<.002,report['mount_max_anchor_gap_m']
  assert all(abs(v-30)<3 for v in report['final_camera_angles'].values()),report['final_camera_angles']
  assert all(c['feed_range_m'][1]-c['feed_range_m'][0]>.001 and c['coupling_max_error_m']<.0002 for c in clamp_checks),clamp_checks
 if args.validate:
  for camera_path,label in [('/World/MountDetail','mount_detail'),('/World/Overview','overview'),('/World/CameraMount/Links/CameraBody/WorkspaceCamera','workspace_camera')]:
   extra_product=rep.create.render_product(camera_path,(1280,960));extra_rgb=rep.AnnotatorRegistry.get_annotator('rgb');extra_rgb.attach([extra_product])
   for _ in range(12):app.update()
   rep.orchestrator.step(delta_time=0.,pause_timeline=False)
   data=np.asarray(extra_rgb.get_data())[:,:,:3];assert data.std()>5 and data.mean()>30
   Image.fromarray(data).save(out/(label+'.png'));print('EXTRA_CAPTURE',label,float(data.mean()),flush=True)
 
 if args.validate:
  (out/'validation_summary.json').write_text(json.dumps({'status':'PASS','simulation_seconds':t,'height_range_m':report['height_range'],'mount_max_anchor_gap_mm':report['mount_max_anchor_gap_m']*1000,'clamp_checks':clamp_checks,'render_views':['initial','raised','returned','overview','workspace_camera','mount_detail'],'scope':report['scope']},indent=2))
except BaseException:
 import traceback
 traceback.print_exc()
 print('RUN_FAILED',flush=True)
 raise
finally:
 if 'sim' in globals():sim.detach_stage()
 app.close()

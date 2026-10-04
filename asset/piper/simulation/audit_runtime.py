"""Native PhysX per-step contact audit. Diagnostic probes are session-only."""
from pathlib import Path
import json,sys,argparse,numpy as np
p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=3);args=p.parse_args()
root=Path(__file__).resolve().parent
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960,'renderer':'RayTracedLighting','extra_args':['--/renderer/multiGpu/enabled=false']})
try:
 import omni.usd,omni.physx,omni.physics.tensors
 from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,Gf
 sys.path.insert(0,str(root/'PiPER_Scene_Assets_v1/scene'));sys.path.insert(0,str(root/'摄像头支架_统一版_v1/runtime'))
 from lift_controller import LiftController
 from camera_mount_runtime import CameraMountRuntime
 ctx=omni.usd.get_context();ctx.open_stage(str(root/'scene.usda'));s=ctx.get_stage();s.SetEditTarget(s.GetSessionLayer())
 for prim in s.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get():PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0)
 probes={}
 for name,kind,pos in [('TableSphere','sphere',(.18,-.06,.8)),('TableCube','cube',(-.13,-.12,.8)),('SofaSphere','sphere',(.30,1.8,.65)),('FloorCube','cube',(-1.2,.1,.2))]:
  path='/World/AuditProbes/'+name;g=UsdGeom.Sphere.Define(s,path) if kind=='sphere' else UsdGeom.Cube.Define(s,path)
  if kind=='sphere':g.CreateRadiusAttr(.01)
  else:g.CreateSizeAttr(.02)
  g.AddTranslateOp().Set(Gf.Vec3d(*pos));UsdPhysics.RigidBodyAPI.Apply(g.GetPrim());UsdPhysics.CollisionAPI.Apply(g.GetPrim());UsdPhysics.MassAPI.Apply(g.GetPrim()).CreateMassAttr(.01);PhysxSchema.PhysxContactReportAPI.Apply(g.GetPrim()).CreateThresholdAttr(0);PhysxSchema.PhysxRigidBodyAPI.Apply(g.GetPrim()).CreateEnableCCDAttr(True)
  probes[name]=path
 mount=CameraMountRuntime(s,json.loads((root/'摄像头支架_统一版_v1/runtime/spring_settings.json').read_text()),root_path='/World/CameraMount')
 for _ in range(3):app.update()
 sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ctx.get_stage_id());dt=1/480;t=0.;sim.simulate(dt,t);sim.fetch_results();t+=dt
 sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=ctx.get_stage_id());v=sv.create_articulation_view('/World/Desk/Joints/Anchor');names=list(v.shared_metatype.dof_names);idx=[names.index('joint'+str(i)) for i in range(1,9)];ii=np.array([0],dtype=np.uint32);home=np.array([0,.65,-.85,0,.2,0,.02,-.02]);q=v.get_dof_positions().copy();q[0,idx]=home;v.set_dof_positions(q,ii);v.set_dof_position_targets(q,ii);control=LiftController(v)
 cvs=[sv.create_articulation_view('/World/Clamps/'+name) for name in ['Left','Right']]
 drives=[UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamps/'+name+'/Joints/ScrewTurn'),'angular') for name in ['Left','Right']]
 aviews={name:sv.create_articulation_view(str(next(p for p in Usd.PrimRange(s.GetPrimAtPath('/World/TaskAssets/'+name)) if p.HasAPI(UsdPhysics.ArticulationRootAPI)).GetPath())) for name in ['microwave','airfryer']}
 adrives={name:UsdPhysics.DriveAPI(next(p for p in Usd.PrimRange(s.GetPrimAtPath('/World/TaskAssets/'+name)) if p.IsA(UsdPhysics.Joint) and p.GetName().endswith('opening_joint')),'angular' if name=='microwave' else 'linear') for name in aviews}
 pairs={};samples=[];step=0;finite=True
 while t<args.seconds:
  # Full lift requires 15 seconds each direction at source 40 mm/s.
  control.set_height(1.235 if 2<t<19 else .635)
  command=home.copy()
  if 3<t<17:
   a=(t-3)*np.pi/7
   command[:6]+=np.array([.25,.12,.12,.15,.10,.20])*np.sin(a)
   opening=.0175*(1+np.cos(a));command[6:]=[opening,-opening]
  for k,d in enumerate(drives):
   turn=-2 if 3+k<t<7+k else (.1 if 9+k<t<11+k else 0)
   d.GetTargetPositionAttr().Set(turn*360)
  for name,target in [('microwave',70 if 7<t<14 else 0),('airfryer',.12 if 7<t<14 else 0)]:
   adrives[name].GetTargetPositionAttr().Set(target)
  control.tick(dt);q=v.get_dof_positions().copy();q[0,idx]=command;q[:,control.joints]=(control.command-control.minimum)/2;v.set_dof_position_targets(q,ii)
  mount.before_step(dt);sim.simulate(dt,t);sim.fetch_results();t+=dt;step+=1
  headers,data=sim.get_contact_report()
  for h in headers:
   a=str(PhysicsSchemaTools.intToSdfPath(h.collider0));b=str(PhysicsSchemaTools.intToSdfPath(h.collider1));key=' | '.join(sorted([a,b]));actor=[str(PhysicsSchemaTools.intToSdfPath(h.actor0)),str(PhysicsSchemaTools.intToSdfPath(h.actor1))]
   for c in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
    impulse=float(np.linalg.norm(c.impulse));sep=float(c.separation)
    if key not in pairs:pairs[key]={'actors':actor,'count':0,'min_separation_m':1.,'max_impulse_Ns':0.,'first_t':t}
    r=pairs[key];r['count']+=1;r['last_t']=t;r['max_impulse_Ns']=max(r['max_impulse_Ns'],impulse)
    if sep<r['min_separation_m']:r.update(min_separation_m=sep,worst_t=t,worst_position=list(c.position))
  poses=v.get_link_transforms();positions=v.get_dof_positions();vel=v.get_dof_velocities();finite=finite and bool(np.isfinite(poses).all() and np.isfinite(positions).all() and np.isfinite(vel).all())
  if step%48==0:
   omni.physx.get_physx_interface().update_transformations(False,True,False);mount.after_step()
   samples.append({'t':t,'height':control.measured_height(),'q':positions[0].tolist(),'qd':vel[0].tolist(),'target':command.tolist(),'appliances':{name:av.get_dof_positions().tolist() for name,av in aviews.items()},'clamps':[cv.get_dof_positions()[0].tolist() for cv in cvs]})
  if step%480==0:print('AUDIT_TIME',round(t,2),'pairs',len(pairs),'height',control.measured_height(),flush=True)
 omni.physx.get_physx_interface().update_transformations(False,True,False)
 report={'seconds':t,'steps':step,'dt':dt,'finite':finite,'dof_names':names,'clamp_names':[list(cv.shared_metatype.dof_names) for cv in cvs],'pairs':pairs,'samples':samples,'probe_final_positions':{k:list(UsdGeom.Xformable(s.GetPrimAtPath(path)).ComputeLocalToWorldTransform(0).ExtractTranslation()) for k,path in probes.items()},'runtime_physics_stats':omni.physx.get_physxunittests_interface().get_physics_stats(),'runtime_mass_kg':v.get_masses().tolist(),'runtime_coms':v.get_coms().tolist(),'runtime_inertias':v.get_inertias().tolist(),'link_names':list(v.shared_metatype.link_names)}
 (root/'evidence/contact_audit.json').write_text(json.dumps(report,indent=2));print('CONTACT_AUDIT_SAVED',len(pairs),flush=True);sim.detach_stage()
except BaseException:
 import traceback;traceback.print_exc();print('AUDIT_FAILED',flush=True)
finally:app.close()

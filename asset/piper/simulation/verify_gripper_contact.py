"""A 25 mm kinematic diagnostic coupon tests physical jaw blocking and release, not grasp lifting."""
from pathlib import Path
import json,numpy as np
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960})
try:
 import omni.usd,omni.physx,omni.physics.tensors
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,PhysicsSchemaTools
 root=Path(__file__).resolve().parent;ctx=omni.usd.get_context();ctx.open_stage(str(root/'scene.usda'));s=ctx.get_stage();s.SetEditTarget(s.GetSessionLayer())
 p=UsdGeom.Cube.Define(s,'/World/GripperDiagnostic');p.CreateSizeAttr(.025);p.AddTranslateOp().Set(Gf.Vec3d(3,0,1));UsdPhysics.CollisionAPI.Apply(p.GetPrim());rb=UsdPhysics.RigidBodyAPI.Apply(p.GetPrim());rb.CreateKinematicEnabledAttr(True);UsdPhysics.MassAPI.Apply(p.GetPrim()).CreateMassAttr(.006);PhysxSchema.PhysxContactReportAPI.Apply(p.GetPrim()).CreateThresholdAttr(0)
 for _ in range(3):app.update()
 sim=omni.physx.get_physx_simulation_interface();sim.attach_stage(ctx.get_stage_id());dt=1/480;t=0;sim.simulate(dt,t);sim.fetch_results();t+=dt
 sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=ctx.get_stage_id());v=sv.create_articulation_view('/World/Desk/Joints/Anchor');names=list(v.shared_metatype.dof_names);idx=[names.index('joint'+str(i)) for i in range(1,9)];ii=np.array([0],dtype=np.uint32);home=np.array([0,.65,-.85,0,.2,0,.035,-.035]);target=v.get_dof_positions().copy();target[0,idx]=home;v.set_dof_positions(target,ii);v.set_dof_position_targets(target,ii);pv=sv.create_rigid_body_view('/World/GripperDiagnostic');placed=False;contacts={};records=[]
 while t<4.5:
  if t>.5 and not placed:
   # Link6 and gripper_base are merged at identical origins; test within the finger pad span.
   pose=v.get_link_transforms()[0,list(v.shared_metatype.link_names).index('link6')];q=Gf.Quatd(float(pose[6]),Gf.Vec3d(*[float(x) for x in pose[3:6]]));point=Gf.Rotation(q).TransformDir(Gf.Vec3d(0,0,.095))+Gf.Vec3d(*[float(x) for x in pose[:3]])
   pp=np.array([[*point,*pose[3:7]]],dtype=np.float32);pv.set_transforms(pp,ii);pv.set_kinematic_targets(pp,ii);placed=True
  opening=0 if 1<t<3 else .035;target[0,idx[6]]=opening;target[0,idx[7]]=-opening;v.set_dof_position_targets(target,ii)
  sim.simulate(dt,t);sim.fetch_results();t+=dt
  hh,dd=sim.get_contact_report();force=0
  for h in hh:
   paths=[str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))]
   if not any('GripperDiagnostic' in p for p in paths):continue
   for c in dd[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
    other=next(p for p in paths if 'GripperDiagnostic' not in p);r=contacts.setdefault(other,{'count':0,'min_separation_m':1.,'max_force_N':0});r['count']+=1;r['min_separation_m']=min(r['min_separation_m'],float(c.separation));f=float(np.linalg.norm(c.impulse))/dt;r['max_force_N']=max(r['max_force_N'],f);force+=f
  if int(t/dt)%24==0:records.append({'t':t,'finger_q':v.get_dof_positions()[0,idx[6:]].tolist(),'contact_force_N':force})
 closing=[r for r in records if 2<t and 2<r['t']<2.9];released=[r for r in records if r['t']>4]
 report={'scope':'Kinematic 25 mm coupon: physical finger blocking and release; not a grasp/lift or friction holding test','contacts':contacts,'samples':records,'status':'PASS' if len(contacts)>=2 and all(abs(x)>.005 for r in closing for x in r['finger_q']) and all(r['contact_force_N']<.01 for r in released) else 'FAIL'}
 (root/'evidence/gripper_contact.json').write_text(json.dumps(report,indent=2));print('GRIPPER_RESULT',report['status'],contacts,flush=True);sim.detach_stage()
except BaseException:
 import traceback;traceback.print_exc();print('GRIPPER_FAILED',flush=True)
finally:app.close()

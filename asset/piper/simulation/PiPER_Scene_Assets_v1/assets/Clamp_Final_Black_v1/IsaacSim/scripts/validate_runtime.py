"""Run real Isaac Sim / PhysX checks; all physical values remain estimates."""
import argparse,json,math,traceback,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--rate',type=int,default=240);p.add_argument('--build',action='store_true');p.add_argument('--only',default='motion');a=p.parse_args();root=a.root.resolve();E=root/'evidence';E.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960,'renderer':'RayTracedLighting','limit_cpu_threads':8,'multi_gpu':False,'extra_args':['--/app/renderer/enabled=false','--/renderer/enabled=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
import numpy as np,omni.usd,omni.physx,omni.physics.tensors
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
from author import build
from isaacsim.core.simulation_manager import SimulationManager
SimulationManager.switch_physics_engine('physx')
params=build(root) if a.build else json.loads((root/'parameters.json').read_text());ctx=omni.usd.get_context();sim=omni.physx.get_physx_simulation_interface();dt=1/a.rate;report={'asset_sha256':__import__('hashlib').sha256((root/'Clamp.usdc').read_bytes()).hexdigest(),'status':'RUNNING','rate':a.rate,'physics':'PhysX TGS; GPU dynamics authored; CPU NumPy tensor readback pipeline','tests':[],'physical_parameters_measured':False};out=E/f'{a.only}_{a.rate}.json'
def dump():out.write_text(json.dumps(report,indent=2,default=lambda x:x.item() if isinstance(x,np.generic) else str(x)),encoding='utf8')
def scene(fixed=True,angle=0,height=.015,probe=None,coupon=None):
 ctx.new_stage();s=ctx.get_stage();UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1);w=UsdGeom.Xform.Define(s,'/World');s.SetDefaultPrim(w.GetPrim())
 ps=UsdPhysics.Scene.Define(s,'/World/Physics');ps.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ps.CreateGravityMagnitudeAttr(9.81);px=PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim());px.CreateEnableGPUDynamicsAttr(True);px.CreateBroadphaseTypeAttr('GPU');px.CreateSolverTypeAttr('TGS');px.CreateTimeStepsPerSecondAttr(a.rate);px.CreateEnableCCDAttr(True);px.CreateEnableStabilizationAttr(True);px.CreateEnableExternalForcesEveryIterationAttr(True)
 floor=UsdGeom.Cube.Define(s,'/World/Ground');floor.CreateSizeAttr(1);floor.AddTranslateOp().Set(Gf.Vec3d(0,0,-.0125));floor.AddScaleOp().Set(Gf.Vec3f(1,1,.025));UsdPhysics.CollisionAPI.Apply(floor.GetPrim());co=PhysxSchema.PhysxCollisionAPI.Apply(floor.GetPrim());co.CreateContactOffsetAttr(.00015);co.CreateRestOffsetAttr(0)
 xf=UsdGeom.Xform.Define(s,'/World/Clamp');xf.GetPrim().GetReferences().AddReference(str(root/'Clamp.usdc'));xf.AddTranslateOp().Set(Gf.Vec3d(0,0,height));xf.AddOrientOp().Set(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(0,math.sin(angle/2),0)))
 if fixed:
  j=UsdPhysics.FixedJoint.Define(s,'/World/Fixture');j.CreateBody1Rel().SetTargets(['/World/Clamp/frame']);j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(params['part_properties']['frame']['center'])+[0,0,height])))
 if probe:
  x,z,enabled=probe;sp=UsdGeom.Sphere.Define(s,'/World/Probe');sp.CreateRadiusAttr(.003);sp.AddTranslateOp().Set(Gf.Vec3d(x,-.05,z+height));UsdPhysics.RigidBodyAPI.Apply(sp.GetPrim());UsdPhysics.MassAPI.Apply(sp.GetPrim()).CreateMassAttr(.02);UsdPhysics.CollisionAPI.Apply(sp.GetPrim()).CreateCollisionEnabledAttr(enabled);PhysxSchema.PhysxRigidBodyAPI.Apply(sp.GetPrim()).CreateDisableGravityAttr(True);PhysxSchema.PhysxContactReportAPI.Apply(sp.GetPrim()).CreateThresholdAttr(0);co=PhysxSchema.PhysxCollisionAPI.Apply(sp.GetPrim());co.CreateContactOffsetAttr(.0001);co.CreateRestOffsetAttr(0)
  j=UsdPhysics.PrismaticJoint.Define(s,'/World/ProbeGuide');j.CreateBody1Rel().SetTargets(['/World/Probe']);j.CreateAxisAttr('Y');j.CreateLocalPos0Attr(Gf.Vec3f(x,-.05,z+height));j.CreateLowerLimitAttr(0);j.CreateUpperLimitAttr(.11);drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(30);drive.CreateDampingAttr(1);drive.CreateMaxForceAttr(.08);drive.CreateTargetPositionAttr(0)
 if coupon is not None:
  pos=np.array([-.028,-.005,.1507+height]);c=UsdGeom.Xform.Define(s,'/World/Coupon');c.AddTranslateOp().Set(Gf.Vec3d(*pos));cm=UsdGeom.Cube.Define(s,'/World/Coupon/Shape');cm.CreateSizeAttr(1);cm.AddScaleOp().Set(Gf.Vec3f(.012,.012,.010));UsdPhysics.CollisionAPI.Apply(cm.GetPrim()).CreateCollisionEnabledAttr(coupon);UsdPhysics.RigidBodyAPI.Apply(c.GetPrim());UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(.02);PhysxSchema.PhysxContactReportAPI.Apply(c.GetPrim()).CreateThresholdAttr(0);rc=PhysxSchema.PhysxCollisionAPI.Apply(cm.GetPrim());rc.CreateContactOffsetAttr(.0003);rc.CreateRestOffsetAttr(0)
  j=UsdPhysics.PrismaticJoint.Define(s,'/World/CouponGuide');j.CreateBody1Rel().SetTargets(['/World/Coupon']);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*pos));j.CreateLowerLimitAttr(-.10);j.CreateUpperLimitAttr(.02)
  spin=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/ScrewTurn'),'angular');spin.GetMaxForceAttr().Set(.01)
 for _ in range(3):app.update()
 assert sim.attach_stage(ctx.get_stage_id());sim.simulate(dt,0);sim.fetch_results();sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=ctx.get_stage_id());art=sv.create_articulation_view('/World/Clamp');report.setdefault('runtime_devices',[]).append({'device':str(sv.device),'device_ordinal':int(sv.device_ordinal),'cuda_context_present':sv.cuda_context is not None,'engine':SimulationManager.get_active_physics_engine()});return s,sv,art
try:
 if a.only in ['motion','all']:
  s,sv,art=scene();names=list(art.shared_metatype.dof_names);print('DOFS',names,flush=True);ix={n:names.index(n) for n in names};trace=[];clock=dt
  spin=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/ScrewTurn'),'angular');pad=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/PadSwivel'),'angular');handle=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/HandleSlide'),'linear')
  # Controlled independent pad/handle strokes during qualification, restored to passive in asset.
  pad.GetStiffnessAttr().Set(.02);pad.GetDampingAttr().Set(.002);handle.GetStiffnessAttr().Set(100);handle.GetDampingAttr().Set(1)
  def phase(seconds,mm,swivel,slide,label):
   global clock
   start=spin.GetTargetPositionAttr().Get() or 0;target=mm/params['thread_pitch_estimated']*360;oldp=pad.GetTargetPositionAttr().Get() or 0;oldh=handle.GetTargetPositionAttr().Get() or 0
   for k in range(round(seconds/dt)):
    alpha=min(1,(k*dt)/(seconds*.7));alpha=alpha*alpha*(3-2*alpha);spin.GetTargetPositionAttr().Set(start+(target-start)*alpha);pad.GetTargetPositionAttr().Set(oldp+(swivel-oldp)*alpha);handle.GetTargetPositionAttr().Set(oldh+(slide-oldh)*alpha)
    sim.simulate(dt,clock);sim.fetch_results();clock+=dt;q=art.get_dof_positions()[0].copy();v=art.get_dof_velocities()[0].copy();rootpose=art.get_root_transforms()[0].copy()
    if k%4==0:trace.append({'t':clock,'phase':label,'q':q.tolist(),'velocity':v.tolist(),'root':rootpose.tolist()})
   return {'label':label,'target_feed_m':mm*.001,'requested_pad_deg':swivel,'requested_handle_m':slide,'target_pad_rad':math.radians(float(np.clip(swivel,-180,180))),'target_handle_m':float(np.clip(slide,*params['handle_slide_limits_m'])),'q':q.tolist(),'velocity':v.tolist()}
  endpoints=[phase(1,0,0,0,'settle')]
  for cycle in range(2):
   endpoints.append(phase(6,18,60,.01,'close_'+str(cycle)));endpoints.append(phase(1,18,60,.01,'hold_'+str(cycle)));endpoints.append(phase(6,0,-45,-.002,'open_'+str(cycle)))
  endpoints.append(phase(2,0,220,.05,'upper_limits'));endpoints.append(phase(2,0,-220,-.06,'lower_limits'));endpoints.append(phase(2,0,0,0,'reset'))
  qs=np.array([x['q'] for x in trace]);feed=qs[:,ix['ScrewFeed']];rot=qs[:,ix['ScrewTurn']];err=abs(feed-rot*params['thread_pitch_estimated']*.001/(2*np.pi));ep_errors=[abs(x['q'][ix['ScrewFeed']]-x['target_feed_m']) for x in endpoints];paderrors=[abs(x['q'][ix['PadSwivel']]-x['target_pad_rad']) for x in endpoints];herrors=[abs(x['q'][ix['HandleSlide']]-x['target_handle_m']) for x in endpoints]
  checks={'finite':bool(np.isfinite(qs).all()),'screw_feed_tracking':max(ep_errors)<.0003,'screw_coupling':float(err.max())<.0001,'pad_independent_rotation':max(paderrors)<math.radians(3),'handle_slide':max(herrors)<.0003,'feed_limits':float(feed.min())>=-.0005 and float(feed.max())<=.0205,'pad_limits':bool(qs[:,ix['PadSwivel']].min()>-math.pi-.04 and qs[:,ix['PadSwivel']].max()<math.pi+.04),'handle_limits':bool(qs[:,ix['HandleSlide']].min()>params['handle_slide_limits_m'][0]-.0003 and qs[:,ix['HandleSlide']].max()<params['handle_slide_limits_m'][1]+.0003),'handle_retained':bool(np.all(-40.0781391+qs[:,ix['HandleSlide']]*1000 < -26.1964625-5) and np.all(35.5218609+qs[:,ix['HandleSlide']]*1000 > -16.3673684+5)),'reset':abs(endpoints[-1]['q'][ix['ScrewFeed']])<.0003}
  report['tests'].append({'name':'screw_pad_handle','checks':checks,'pass':all(checks.values()),'dof_names':names,'max_coupling_error_m':float(err.max()),'max_feed_endpoint_error_m':max(ep_errors),'endpoints':endpoints,'trace':trace});sim.detach_stage();art=sv=None
 if a.only in ['drop','all']:
  for run,(angle,height) in enumerate([(0,.03),(.15,.04),(-.15,.05)]):
   s,sv,art=scene(False,angle,height);clock=dt;trace=[];rv=None;cv=sv.create_rigid_contact_view(['/World/Clamp/'+n for n in ['frame','screw','pad','handle']],filter_patterns=[['/World/Ground']]*4,max_contact_data_count=1024);maxdepth=0;maxforce=0;max_geometry_pen=0;support={n:np.load(root/'mesh_data'/f'{n}.npz')['support_vertices'] for n in ['frame','screw','pad','handle']};link_names=list(art.shared_metatype.link_names)
   for k in range(round(11/dt)):
    sim.simulate(dt,clock);sim.fetch_results();clock+=dt
    data=cv.get_contact_data(dt);sep=np.asarray(data[3]).reshape(-1);counts=np.asarray(data[4]);starts=np.asarray(data[5]);active=[float(sep[int(st)+ii]) for count,st in zip(counts.flat,starts.flat) for ii in range(int(count))]
    if active:maxdepth=max(maxdepth,-min(active))
    maxforce=max(maxforce,float(np.linalg.norm(cv.get_net_contact_forces(dt))))
    linkposes=art.get_link_transforms()[0]
    for li,n in enumerate(link_names):
     if n not in support:continue
     po=linkposes[li];x,y,z,w=po[3:];row=np.array([2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]);bottom=float((support[n]@row).min()+po[2]);max_geometry_pen=max(max_geometry_pen,-bottom)
    if k%8==0:
     q=art.get_dof_positions()[0].copy();pos=art.get_root_transforms()[0].copy();vel=art.get_root_velocities()[0].copy();trace.append({'t':clock,'root':pos.tolist(),'velocity':vel.tolist(),'q':q.tolist(),'dof_velocity':art.get_dof_velocities()[0].tolist(),'link_velocity':art.get_link_velocities()[0].tolist()})
   tail=np.array([r['root'][:3] for r in trace if r['t']>10]);speeds=np.array([r['velocity'][:3] for r in trace if r['t']>10]);positions=np.array([r['root'] for r in trace]);drift=float(np.linalg.norm(tail.max(axis=0)-tail.min(axis=0)));speed=float(np.linalg.norm(speeds,axis=1).max());checks={'finite':bool(np.isfinite(positions).all()),'settled_drift':drift<.001,'settled_speed':speed<.005,'no_fallthrough':float(positions[:,2].min())>-.01,'contact_present':maxforce>.01,'geometry_penetration':max_geometry_pen<.0005,'links_settled':max(np.linalg.norm(np.array(r['link_velocity'])[:,:3],axis=1).max() for r in trace if r['t']>10)<.005}
   report['tests'].append({'name':'drop_'+str(run),'angle_rad':angle,'height_m':height,'checks':checks,'pass':all(checks.values()),'last_second_drift_m':drift,'last_second_speed_m_s':speed,'max_contact_separation_negative_m':maxdepth,'max_postsolve_mesh_penetration_m':max_geometry_pen,'max_contact_force_N':maxforce,'trace':trace});print('DROP',run,checks,drift,speed,flush=True);dump();sim.detach_stage();sv=art=rv=None
 if a.only in ['probe','all']:
  for name,x,z,enabled in [('aperture',-.005,.10,True),('wall',.035,.11,True),('wall_disabled',.035,.11,False)]:
   s,sv,art=scene(probe=(x,z,enabled));pv=sv.create_rigid_body_view('/World/Probe');cv=sv.create_rigid_contact_view('/World/Probe',filter_patterns=['/World/Clamp/frame'],max_contact_data_count=256);drive=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/ProbeGuide'),'linear');clock=dt;trace=[];maxf=0;maxpen=0
   for k in range(round(4/dt)):
    drive.GetTargetPositionAttr().Set(min(.10,k*dt*.035));sim.simulate(dt,clock);sim.fetch_results();clock+=dt;pose=pv.get_transforms()[0].copy();force=float(np.linalg.norm(cv.get_net_contact_forces(dt)));maxf=max(maxf,force)
    data=cv.get_contact_data(dt);sep=np.asarray(data[3]).reshape(-1);counts=np.asarray(data[4]);starts=np.asarray(data[5]);active=[float(sep[int(st)+i]) for count,st in zip(counts.flat,starts.flat) for i in range(int(count))]
    if active:maxpen=max(maxpen,-min(active))
    if k%4==0:trace.append({'t':clock,'position':pose[:3].tolist(),'force_N':force})
   traversed=pose[1]>.04;checks={'finite':bool(np.isfinite(pose).all()),'expected_contact_behavior':bool(traversed if name!='wall' else pose[1]<0 and maxf>.005),'penetration':maxpen<.0005};report['tests'].append({'name':name,'checks':checks,'pass':all(checks.values()),'end_y':float(pose[1]),'max_force_N':maxf,'max_penetration_m':maxpen,'trace':trace});print('PROBE',name,checks,pose,maxf,maxpen,flush=True);dump();sim.detach_stage();sv=art=pv=cv=None
 if a.only in ['clamp','all']:
  for enabled in [True,False]:
   s,sv,art=scene(coupon=enabled);cv=sv.create_rigid_contact_view('/World/Coupon',filter_patterns=['/World/Clamp/pad','/World/Clamp/frame'],max_contact_data_count=256);rv=sv.create_rigid_body_view('/World/Coupon');drive=UsdPhysics.DriveAPI(s.GetPrimAtPath('/World/Clamp/Joints/ScrewTurn'),'angular');names=list(art.shared_metatype.dof_names);idx=names.index('ScrewFeed');clock=dt;trace=[];maxpen=0;holdforces=[]
   for k in range(round(10/dt)):
    t=k*dt
    if t<1:goal=0;label='settle'
    elif t<5:goal=min(18,(t-1)*6);label='close'
    elif t<7:goal=18;label='hold'
    else:goal=max(0,18-(t-7)*9);label='release'
    drive.GetTargetPositionAttr().Set(goal/params['thread_pitch_estimated']*360);sim.simulate(dt,clock);sim.fetch_results();clock+=dt;q=art.get_dof_positions()[0].copy();forces=np.asarray(cv.get_contact_force_matrix(dt));fn=np.linalg.norm(forces,axis=-1).reshape(-1);pose=rv.get_transforms()[0].copy()
    if 5.5<t<6.8:holdforces.append(fn.tolist())
    data=cv.get_contact_data(dt);sep=np.asarray(data[3]).reshape(-1);counts=np.asarray(data[4]);starts=np.asarray(data[5]);active=[float(sep[int(st)+i]) for count,st in zip(counts.flat,starts.flat) for i in range(int(count))]
    if active:maxpen=max(maxpen,-min(active))
    if k%4==0:trace.append({'t':clock,'phase':label,'feed':float(q[idx]),'forces_N':fn.tolist(),'coupon':pose[:3].tolist(),'coupon_velocity':rv.get_velocities()[0].tolist(),'joint_q':q.tolist()})
   hold=np.array([r['feed'] for r in trace if r['phase']=='hold']);hf=np.array(holdforces);checks={'finite':bool(np.isfinite(hold).all()),'reset':abs(float(q[idx]))<.0003}
   if enabled:checks.update({'workpiece_stops_feed':.005<float(np.median(hold))<.016,'both_jaws_contact':bool((np.mean(hf,axis=0)>.02).all()),'contact_penetration':maxpen<.0005})
   else:checks.update({'no_contact_control':abs(float(np.median(hold))-.018)<.0003 and float(hf.max())<.001})
   report['tests'].append({'name':'clamping' if enabled else 'clamping_disabled_control','checks':checks,'pass':all(checks.values()),'hold_feed_m':float(np.median(hold)),'mean_hold_forces_N':np.mean(hf,axis=0).tolist(),'max_contact_penetration_m':maxpen,'torque_limit_Nm':.01,'trace':trace});print('CLAMP',enabled,checks,float(np.median(hold)),np.mean(hf,axis=0),maxpen,flush=True);dump();sim.detach_stage();sv=art=cv=rv=None
 report['status']='PASS' if report['tests'] and all(t['pass'] for t in report['tests']) else 'FAIL'
except Exception:report['status']='ERROR';report['error']=traceback.format_exc();print(report['error'],flush=True)
dump();print('RESULT',out,report['status'],flush=True);app.close()


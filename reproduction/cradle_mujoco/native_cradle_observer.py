"""Five native revolute pendulums; source asset SI geometry. No post-init pose forcing."""
import os,sys,json,math,time,traceback,argparse
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--engine',required=True);p.add_argument('--case',required=True);p.add_argument('--out',required=True);a=p.parse_args()
c=json.loads(Path(a.case).read_text());out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
if (out/'result.json').exists():raise RuntimeError('Result exists; preserve old run')
R=c.get('radius_m',.010049897571124499);L=c.get('length_m',.128989013);M=c.get('mass_kg',.03319999948);I=.4*M*R*R;hz=c['hz'];dt=1/hz;steps=round(c['seconds']*hz);iters=c.get('iterations',100);e=c.get('restitution',.95);mu=c.get('friction',.1);damping=c.get('joint_damping_Nms',0.);spacing=2*R+c.get('gap_m',0.);pivots=np.array([[(i-2)*spacing,0,L+.04] for i in range(5)]);angles=np.array(c.get('angles_deg',[0]*(5-c['release_count'])+[c['angle_deg']]*c['release_count']))*np.pi/180
rod_r=c.get('rod_radius_m',.00015);rod_l=L-R;rod_m=c.get('rod_mass_kg',1000*math.pi*rod_r**2*rod_l);rod_z=(L+R)/2;MT=M+rod_m;COM=rod_m*rod_z/MT;IX=I+M*COM**2+rod_m*(rod_l**2/12+rod_r**2/4+(rod_z-COM)**2);IZ=I+rod_m*rod_r**2/2
q0=-angles;poses=pivots+np.array([L*np.sin(angles),np.zeros(5),-L*np.cos(angles)]).T
rows=[];contacts=[];meta={'case':c,'engine':a.engine,'geometry':dict(radius_m=R,length_m=L,mass_kg=M,inertia=I,pivots=pivots.tolist()),'initial_position_requested':poses.tolist(),'joint_model':'5 native Y hinges; finite-mass thin cylinder and sphere as one rigid compound; rod collision excluded, sphere collision native','parameter_provenance':'R and sphere mass inherited; L from video arc/diameter ratio; initial angles from held-position annotations; all estimates and uncertainty recorded in measurement.json','friction':mu,'restitution_requested':e,'iterations_requested':iters,'contact':{},'status':'running'}
meta['rod']=dict(radius_m=rod_r,length_m=rod_l,mass_kg=rod_m,total_mass_kg=MT,com_from_ball_m=COM,inertia_about_com=[IX,IX,IZ],density_assumption_kg_m3=1000,collision=False)
(out/'asset_manifest.json').write_text(json.dumps(dict(geometry=meta['geometry'],rod=meta['rod'],case=c),indent=2))
def row(t,pos,vel=None,extra=None):
 arr=np.array(pos);rows.append(dict(t=float(t),position=arr.tolist(),velocity=None if vel is None else np.asarray(vel).tolist(),extra=extra))
 if not np.isfinite(arr).all() or np.max(abs(arr))>2:raise RuntimeError('Native numerical sanity failed')
def contact(k,items,**extra):
 if items:contacts.append(dict(step=k,t_start=(k-1)*dt,t_end=k*dt,items=items,**extra))
def collision_on(i):return not c.get('no_collision') and i in c.get('collision_subset',list(range(5)))
def mujoco():
 import mujoco as mj
 import xml.etree.ElementTree as E
 root=E.Element('mujoco',model='real_cradle');E.SubElement(root,'compiler',angle='radian');E.SubElement(root,'option',timestep=str(dt),gravity='0 0 -9.80665',iterations=str(iters),integrator='implicitfast');wb=E.SubElement(root,'worldbody')
 for i,pv in enumerate(pivots):
  b=E.SubElement(wb,'body',name=f'ball{i}',pos=' '.join(map(str,pv)));E.SubElement(b,'joint',name=f'hinge{i}',axis='0 1 0',damping=str(damping));E.SubElement(b,'inertial',pos=f'0 0 {-L+COM}',mass=str(MT),diaginertia=f'{IX} {IX} {IZ}');E.SubElement(b,'geom',name=f'sphere{i}',type='sphere',pos=f'0 0 {-L}',size=str(R),mass='0',friction=f'{mu} 0 0',condim='3',solref=f"-{c.get('mj_stiffness',25000)} -{c.get('mj_damping',5.163)}",solimp='.99 .99 .001',contype='1' if collision_on(i) else '0',conaffinity='1' if collision_on(i) else '0')
  E.SubElement(b,'geom',name=f'rod{i}',type='cylinder',pos=f'0 0 {-L+rod_z}',size=f'{rod_r} {rod_l/2}',mass='0',contype='0',conaffinity='0',rgba='.7 .7 .7 1')
 xml=E.tostring(root,encoding='unicode');(out/'scene.xml').write_text(xml);m=mj.MjModel.from_xml_string(xml);d=mj.MjData(m);d.qpos[:]=q0;mj.mj_forward(m,d);ids=np.array([mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,f'sphere{i}') for i in range(5)]);v=np.zeros(6)
 meta.update(version=mj.__version__,native_dt=m.opt.timestep,contact=dict(law='MuJoCo direct-format compliant contact',solref=[-c.get('mj_stiffness',25000),-c.get('mj_damping',5.163)],note='normalized stiffness/damping; restitution not directly available; damping ratio .01633 of an ideal isolated linear oscillator, not measured pair restitution'),native_masses=m.body_mass[1:].tolist())
 def snap(t):
  vs=[]
  for i in ids:mj.mj_objectVelocity(m,d,mj.mjtObj.mjOBJ_GEOM,int(i),v,0);vs.append(v.copy().tolist())
  row(t,d.geom_xpos[ids],vs,dict(q=d.qpos.tolist(),qd=d.qvel.tolist(),solver_niter=d.solver_niter.tolist(),efc_pos=d.efc_pos.tolist()))
 snap(0)
 for k in range(1,steps+1):
  mj.mj_step(m,d);cs=[]
  for j in range(d.ncon):
   cf=np.zeros(6);mj.mj_contactForce(m,d,j,cf);co=d.contact[j];cs.append(dict(pair=[int(co.geom1),int(co.geom2)],position=co.pos.tolist(),distance=float(co.dist),force_contact_frame=cf.tolist(),frame=co.frame.tolist()))
  contact(k,cs,geometry_phase='start of step',force_phase='solver interval');mj.mj_forward(m,d);snap(d.time)
def pybullet():
 import pybullet as pb
 pb.connect(pb.DIRECT);pb.setGravity(0,0,-9.80665);pb.setTimeStep(dt);pb.setPhysicsEngineParameter(numSolverIterations=iters,restitutionVelocityThreshold=0,contactBreakingThreshold=1e-5);ids=[]
 for i in range(5):
  shape=pb.createCollisionShape(pb.GEOM_SPHERE,radius=R,collisionFramePosition=[0,0,-L]) if collision_on(i) else -1
  visual=pb.createVisualShapeArray([pb.GEOM_SPHERE,pb.GEOM_CYLINDER],radii=[R,rod_r],lengths=[0,rod_l],visualFramePositions=[[0,0,-L],[0,0,-L+rod_z]],rgbaColors=[[.6,.6,.65,1],[.7,.7,.7,1]])
  b=pb.createMultiBody(baseMass=0,basePosition=pivots[i],linkMasses=[MT],linkCollisionShapeIndices=[shape],linkVisualShapeIndices=[visual],linkPositions=[[0,0,0]],linkOrientations=[[0,0,0,1]],linkInertialFramePositions=[[0,0,-L+COM]],linkInertialFrameOrientations=[[0,0,0,1]],linkParentIndices=[0],linkJointTypes=[pb.JOINT_REVOLUTE],linkJointAxis=[[0,1,0]])
  pb.changeDynamics(b,0,localInertiaDiagonal=[IX,IX,IZ],lateralFriction=math.sqrt(mu),restitution=math.sqrt(e),linearDamping=0,angularDamping=0,jointDamping=damping,contactProcessingThreshold=0);pb.setJointMotorControl2(b,0,pb.VELOCITY_CONTROL,force=0);pb.resetJointState(b,0,float(q0[i]));pb.enableJointForceTorqueSensor(b,0,True);ids.append(b)
 meta.update(version=pb.getAPIVersion(),contact=dict(law='Bullet impulse restitution',per_body_restitution=math.sqrt(e),pair_restitution=e,pair_mu=mu),native_mass=[pb.getDynamicsInfo(b,0)[0] for b in ids])
 def snap(k):
  ss=[pb.getLinkState(b,0,computeLinkVelocity=1,computeForwardKinematics=1) for b in ids];row(k*dt,[(np.asarray(s[0])-np.asarray(pb.getMatrixFromQuaternion(s[1])).reshape(3,3)@np.array([0,0,COM])).tolist() for s in ss],[[*s[6],*s[7]] for s in ss],dict(joint=[pb.getJointState(b,0) for b in ids]))
 snap(0)
 for k in range(1,steps+1):
  pb.stepSimulation();snap(k);contact(k,[dict(bodyA=x[1],bodyB=x[2],pointA=x[5],pointB=x[6],normalB=x[7],distance=x[8],normal_force=x[9],lateral1=x[10],direction1=x[11],lateral2=x[12],direction2=x[13]) for x in pb.getContactPoints()],geometry_phase='native getContactPoints after step')
 pb.disconnect()
def drake():
 from pydrake.all import DiagramBuilder,AddMultibodyPlantSceneGraph,SpatialInertia,UnitInertia,FixedOffsetFrame,RigidTransform,RevoluteJoint,Sphere,CoulombFriction,Simulator,DiscreteContactApproximation,RotationMatrix,Cylinder
 builder=DiagramBuilder();plant,sg=AddMultibodyPlantSceneGraph(builder,time_step=dt);plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap);plant.mutable_gravity_field().set_gravity_vector([0,0,-9.80665]);bs=[];js=[]
 for i in range(5):
  b=plant.AddRigidBody(f'ball{i}',SpatialInertia(MT,[0,0,COM],UnitInertia(IX/MT+COM**2,IX/MT+COM**2,IZ/MT)));pf=plant.AddFrame(FixedOffsetFrame(f'pivot{i}',plant.world_frame(),RigidTransform(pivots[i])));bf=plant.AddFrame(FixedOffsetFrame(f'attachment{i}',b.body_frame(),RigidTransform([0,0,L])));j=plant.AddJoint(RevoluteJoint(f'hinge{i}',pf,bf,[0,1,0],damping));bs.append(b);js.append(j)
  plant.RegisterVisualGeometry(b,RigidTransform(),Sphere(R),f'sphere_visual{i}',[.6,.6,.65,1]);plant.RegisterVisualGeometry(b,RigidTransform([0,0,rod_z]),Cylinder(rod_r,rod_l),f'rod_visual{i}',[.7,.7,.7,1])
  if collision_on(i):
   from pydrake.geometry import ProximityProperties
   prop=ProximityProperties();prop.AddProperty('material','coulomb_friction',CoulombFriction(mu,mu));prop.AddProperty('material','point_contact_stiffness',c.get('drake_stiffness',1e6));prop.AddProperty('material','relaxation_time',c.get('drake_relaxation',.001));plant.RegisterCollisionGeometry(b,RigidTransform(),Sphere(R),f'sphere{i}',prop)
 plant.Finalize();diagram=builder.Build();ctx=diagram.CreateDefaultContext();pc=plant.GetMyMutableContextFromRoot(ctx)
 for i,j in enumerate(js):j.set_angle(pc,q0[i])
 sim=Simulator(diagram,ctx);sim.Initialize();meta.update(contact=dict(law='SAP Kelvin-Voigt point contact',point_stiffness_N_m=1e6,relaxation_s=.001,restitution_direct=False),native_mass=[b.default_mass() for b in bs],iterations_applied=False)
 def snap(k):row(ctx.get_time(),[plant.EvalBodyPoseInWorld(pc,b).translation() for b in bs],[plant.EvalBodySpatialVelocityInWorld(pc,b).get_coeffs() for b in bs],dict(q=plant.GetPositions(pc).tolist(),qd=plant.GetVelocities(pc).tolist()))
 snap(0)
 for k in range(1,steps+1):
  sim.AdvanceTo(k*dt);snap(k);cr=plant.get_contact_results_output_port().Eval(pc);cs=[]
  for j in range(cr.num_point_pair_contacts()):
   co=cr.point_pair_contact_info(j);cs.append(dict(bodyA=int(co.bodyA_index()),bodyB=int(co.bodyB_index()),point=co.contact_point().tolist(),force_on_B=co.contact_force().tolist(),separation_speed=co.separation_speed(),slip_speed=co.slip_speed()))
  contact(k,cs,phase='native discrete contact results at completed step')
def newton():
 import newton as nw,warp as wp
 nw.use_coord_layout_targets=True;b=nw.ModelBuilder(gravity=wp.vec3(0,0,-9.80665));cfg=nw.ModelBuilder.ShapeConfig(mu=mu,restitution=e,density=0,margin=1e-5,mu_torsional=0,mu_rolling=0)
 for i in range(5):
  body=b.add_link(xform=wp.transform(wp.vec3(*poses[i]),wp.quat_from_axis_angle(wp.vec3(0,1,0),float(q0[i]))),mass=MT,com=wp.vec3(0,0,COM),inertia=wp.mat33(*np.diag([IX,IX,IZ]).flatten()),lock_inertia=True,label=f'ball{i}')
  if collision_on(i):b.add_shape_sphere(body,radius=R,cfg=cfg)
  b.add_shape_cylinder(body,xform=wp.transform(wp.vec3(0,0,rod_z),wp.quat_identity()),radius=rod_r,half_height=rod_l/2,cfg=nw.ModelBuilder.ShapeConfig(density=0,has_shape_collision=False,has_particle_collision=False),label=f'rod{i}')
  joint=b.add_joint_revolute(-1,body,parent_xform=wp.transform(wp.vec3(*pivots[i]),wp.quat_identity()),child_xform=wp.transform(wp.vec3(0,0,L),wp.quat_identity()),axis=wp.vec3(0,1,0),damping=damping,armature=0,friction=0,label=f'hinge{i}');b.add_articulation([joint]);b.joint_q[-1]=float(q0[i])
 model=b.finalize(device='cuda:0');model.request_contact_attributes('force');s0=model.state();s1=model.state();nw.eval_fk(model,model.joint_q,model.joint_qd,s0);control=model.control();solver=nw.solvers.SolverXPBD(model,iterations=iters,enable_restitution=True,angular_damping=0);pipe=nw.CollisionPipeline(model,deterministic=True);ct=pipe.contacts();meta.update(version=nw.__version__,contact=dict(law='native XPBD with restitution',margin_m=1e-5),native_mass=model.body_mass.numpy().tolist())
 row(0,s0.body_q.numpy()[:,:3],s0.body_qd.numpy())
 for k in range(1,steps+1):
  s0.clear_forces();pipe.collide(s0,ct);count=int(ct.rigid_contact_count.numpy()[0]);cs={n:getattr(ct,n).numpy()[:count].copy() for n in ['rigid_contact_shape0','rigid_contact_shape1','rigid_contact_point0','rigid_contact_point1','rigid_contact_normal']};solver.step(s0,s1,control,ct,dt);solver.update_contacts(ct);cs['force']=ct.force.numpy()[:count].copy();contact(k,[{n:v[j].tolist() for n,v in cs.items()} for j in range(count)],geometry_phase='start of step',force_semantics='XPBD native solved contact correction impulse/dt');s0,s1=s1,s0;row(k*dt,s0.body_q.numpy()[:,:3],s0.body_qd.numpy())
def gazebo():
 import xml.etree.ElementTree as E
 import threading
 from gz.transport13 import Node
 from gz.msgs10.contacts_pb2 import Contacts
 from gz.math7 import Pose3d,Vector3d
 from gz.sim8 import TestFixture,World,Model,Link,world_entity
 def txt(parent,path,value):
  nd=parent
  for s in path.split('/'):
   f=nd.find(s);nd=f if f is not None else E.SubElement(nd,s)
  nd.text=str(value)
 root=E.Element('sdf',version='1.9');w=E.SubElement(root,'world',name='cradle');txt(w,'gravity','0 0 -9.80665');ph=E.SubElement(w,'physics',name='physics',type='ignored');txt(ph,'max_step_size',dt);txt(ph,'real_time_factor',0);pl=E.SubElement(w,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics');txt(pl,'engine/filename','gz-physics-dartsim-plugin')
 E.SubElement(w,'plugin',filename='gz-sim-contact-system',name='gz::sim::systems::Contact')
 for i in range(5):
  md=E.SubElement(w,'model',name=f'pendulum{i}');link=E.SubElement(md,'link',name='ball');txt(link,'pose',' '.join(map(str,[*poses[i],0,q0[i],0])));txt(link,'inertial/mass',MT);txt(link,'inertial/pose',f'0 0 {COM} 0 0 0')
  for ax,val in zip(['ixx','iyy','izz'],[IX,IX,IZ]):txt(link,'inertial/inertia/'+ax,val)
  vis=E.SubElement(link,'visual',name='sphere_visual');txt(vis,'geometry/sphere/radius',R);vis=E.SubElement(link,'visual',name='rod_visual');txt(vis,'pose',f'0 0 {rod_z} 0 0 0');txt(vis,'geometry/cylinder/radius',rod_r);txt(vis,'geometry/cylinder/length',rod_l)
  if collision_on(i):
   co=E.SubElement(link,'collision',name='sphere');txt(co,'geometry/sphere/radius',R);txt(co,'surface/friction/ode/mu',mu);txt(co,'surface/friction/ode/mu2',mu);txt(co,'surface/bounce/restitution_coefficient',e);txt(co,'surface/bounce/threshold',0)
  sensor=E.SubElement(link,'sensor',name='contacts',type='contact');txt(sensor,'always_on','true');txt(sensor,'update_rate',0);txt(sensor,'contact/collision','sphere');txt(sensor,'contact/topic',f'/cradle/ball{i}/contacts')
  j=E.SubElement(md,'joint',name='hinge',type='revolute');txt(j,'parent','world');txt(j,'child','ball');txt(j,'pose',f'0 0 {L} 0 0 0');txt(j,'axis/xyz','0 1 0');txt(j,'axis/dynamics/damping',damping);txt(j,'axis/limit/lower',-1000);txt(j,'axis/limit/upper',1000)
 path=out/'scene.sdf';E.ElementTree(root).write(path,encoding='unicode');fixture=TestFixture(str(path));links=[];node=Node();raw=[];callbacks=[];mutex=threading.Lock()
 def receive(msg,body):
  stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9;items=[]
  for pair in msg.contact:
   for k,p in enumerate(pair.position):
    item=dict(sensor_body=body,collision1=str(pair.collision1),collision2=str(pair.collision2),point=[p.x,p.y,p.z])
    if k<len(pair.normal):v=pair.normal[k];item['normal']=[v.x,v.y,v.z]
    if k<len(pair.depth):item['depth']=pair.depth[k]
    if k<len(pair.wrench):
     w=pair.wrench[k]
     for label in ['body_1_wrench','body_2_wrench']:
      v=getattr(w,label);item[label]=dict(force=[v.force.x,v.force.y,v.force.z],torque=[v.torque.x,v.torque.y,v.torque.z])
    items.append(item)
  with mutex:raw.append(dict(native_timestamp_s=stamp,sensor_body=body,items=items))
 for i in range(5):
  cb=lambda msg,i=i:receive(msg,i);callbacks.append(cb);node.subscribe(Contacts,f'/cradle/ball{i}/contacts',cb)

 def pre(info,ecm):
  if links:return
  world=World(world_entity(ecm))
  for i in range(5):
   md=Model(world.model_by_name(ecm,f'pendulum{i}'));l=Link(md.link_by_name(ecm,'ball'));l.enable_velocity_checks(ecm,True);links.append(l)
  row(0,poses,extra={'phase':'authored initial pose; first native post-update follows'})
 def post(info,ecm):
  if info.paused:return
  pos=[];vs=[]
  for l in links:
   tf=l.world_pose(ecm);v=l.world_linear_velocity(ecm);pos.append([tf.pos().x(),tf.pos().y(),tf.pos().z()]);vs.append([v.x(),v.y(),v.z()] if v is not None else [None]*3)
  row(info.sim_time.total_seconds(),pos,vs)
 fixture.on_pre_update(pre);fixture.on_post_update(post);fixture.finalize();time.sleep(.3);fixture.server().run(True,steps,False);time.sleep(.4)
 with mutex:(out/'contact_observer.json').write_text(json.dumps(raw))
 meta['contact_observer']=dict(messages=len(raw),semantics='native Contacts point/wrench and native timestamps; per-body sensors may duplicate pairs; not summed')
 meta.update(contact=dict(law='Gazebo DART native SDF sphere contact',SDF_restitution=e,backend_parameter_readback_available=False),iterations_applied=False,contact_force_availability='not instrumented in this adapter; position/velocity retained')
def isaac():
 os.environ['OMNI_KIT_ACCEPT_EULA']='YES'
 from isaacsim import SimulationApp
 sys.argv=sys.argv[:1];app=SimulationApp({'headless':True,'multi_gpu':False,'active_gpu':0,'physics_gpu':0})
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf
 import omni.usd,omni.physics.tensors as tensors
 from isaacsim.core.api import World
 world=World(physics_dt=dt,rendering_dt=dt,stage_units_in_meters=1.,backend='numpy',device='cpu');world.get_physics_context().set_gravity(-9.80665);stage=omni.usd.get_context().get_stage();mat=UsdShade.Material.Define(stage,'/World/steel');ma=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());ma.CreateStaticFrictionAttr(mu);ma.CreateDynamicFrictionAttr(mu);ma.CreateRestitutionAttr(e);pm=PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim());pm.CreateRestitutionCombineModeAttr('average');pm.CreateFrictionCombineModeAttr('average')
 for i in range(5):
  path=f'/World/ball{i}';xf=UsdGeom.Xform.Define(stage,path);xf.AddTranslateOp().Set(Gf.Vec3d(*poses[i]));xf.AddOrientOp().Set(Gf.Quatf(math.cos(q0[i]/2),Gf.Vec3f(0,math.sin(q0[i]/2),0)));UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim());mass=UsdPhysics.MassAPI.Apply(xf.GetPrim());mass.CreateMassAttr(MT);mass.CreateCenterOfMassAttr(Gf.Vec3f(0,0,COM));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(IX,IX,IZ));pr=PhysxSchema.PhysxRigidBodyAPI.Apply(xf.GetPrim());pr.CreateLinearDampingAttr(0.);pr.CreateAngularDampingAttr(0.);pr.CreateSleepThresholdAttr(0.);pr.CreateSolverPositionIterationCountAttr(iters);pr.CreateSolverVelocityIterationCountAttr(8)
  g=UsdGeom.Sphere.Define(stage,path+'/Geom');g.CreateRadiusAttr(R);rod=UsdGeom.Cylinder.Define(stage,path+'/Rod');rod.CreateRadiusAttr(rod_r);rod.CreateHeightAttr(rod_l);rod.CreateAxisAttr('Z');rod.AddTranslateOp().Set(Gf.Vec3d(0,0,rod_z))
  if collision_on(i):UsdPhysics.CollisionAPI.Apply(g.GetPrim());ca=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim());ca.CreateContactOffsetAttr(1e-5);ca.CreateRestOffsetAttr(0.);UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat,materialPurpose='physics')
  joint=UsdPhysics.RevoluteJoint.Define(stage,f'/World/hinge{i}');joint.CreateBody1Rel().SetTargets([path]);joint.CreateAxisAttr('Y');joint.CreateLocalPos0Attr(Gf.Vec3f(*pivots[i]));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,L));joint.CreateLocalRot0Attr(Gf.Quatf(1));joint.CreateLocalRot1Attr(Gf.Quatf(1))
  if damping:
   drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(damping*math.pi/180);drive.CreateTargetVelocityAttr(0.)
 from isaacsim.core.prims import RigidPrim
 cv=RigidPrim('/World/ball*',name='contact_observer',track_contact_forces=True,contact_filter_prim_paths_expr=[f'/World/ball{i}/Geom' for i in range(5)],max_contact_count=4096,prepare_contact_sensors=True);world.scene.add(cv);observer=[]
 stage.GetRootLayer().Export(str(out/'scene.usda'));world.reset();cv.initialize();view=tensors.create_simulation_view('numpy');view.set_subspace_roots('/');bv=view.create_rigid_body_view('/World/ball*');paths=[str(x) for x in bv.prim_paths];order=[paths.index(f'/World/ball{i}') for i in range(5)];trans=bv.get_transforms().copy();vel=bv.get_velocities().copy()
 for i,k in enumerate(order):trans[k]=[*poses[i],0,math.sin(q0[i]/2),0,math.cos(q0[i]/2)];vel[k]=0
 bv.set_transforms(trans,np.arange(5,dtype=np.uint32));bv.set_velocities(vel,np.arange(5,dtype=np.uint32));origin=world.current_time;meta.update(contact=dict(law='PhysX native sphere restitution',restitution=e,combine='average'),native_mass=bv.get_masses().tolist(),contact_force_availability='not instrumented in this adapter; native positions/velocities retained',initialization='one-time pose/velocity initialization after world reset')
 row(0,bv.get_transforms()[order,:3],bv.get_velocities()[order])
 for k in range(1,steps+1):
  world.step(render=False);row(world.current_time-origin,bv.get_transforms()[order,:3],bv.get_velocities()[order])
  normal=cv.get_contact_force_data(dt=dt);friction=cv.get_friction_data(dt=dt)
  def packed(tup):
   aa=[np.asarray(v) for v in tup];counts,starts=aa[-2:];used=int(np.max(np.where(counts>0,starts+counts,0))) if counts.size else 0
   return [v[:used].tolist() for v in aa[:-2]]+[counts.tolist(),starts.tolist()]
  observer.append(dict(step=k,native_time_s=float(world.current_time),time_s=float(world.current_time-origin),normal_tuple=packed(normal),friction_tuple=packed(friction),normal_pair_force_world=cv.get_contact_force_matrix(dt=dt).tolist()))
 (out/'contact_observer.json').write_text(json.dumps(observer));meta['contact_observer']=dict(samples=len(observer),semantics='RigidPrim native normal tuples force/point/normal/distance/count/start and separate friction patch tuple; dt gives interval-average force; pair matrix normal only',body_paths=[str(v) for v in cv.prim_paths] if hasattr(cv,'prim_paths') else paths)
 return app
start=time.time();app=None
try:
 app=globals()[a.engine]();meta['status']='completed';meta['observed_steps']=len(rows)-1
except Exception:meta['status']='failed';meta['error']=traceback.format_exc()
meta['wall_seconds']=time.time()-start
if rows:
 pos=np.array([r['position'] for r in rows]);vec=pos-pivots[None,:,:];theta=np.arctan2(vec[:,:,0],-vec[:,:,2]);meta['max_hinge_radius_error_m']=float(abs(np.linalg.norm(vec,axis=2)-L).max());meta['initial_position_error_m']=float(np.linalg.norm(pos[0]-poses,axis=1).max());meta['max_angles_deg']=np.rad2deg(abs(theta).max(axis=0)).tolist()
(out/'states.json').write_text(json.dumps(rows));(out/'contacts.json').write_text(json.dumps(contacts));(out/'result.json').write_text(json.dumps(meta,indent=2));print(json.dumps(meta),flush=True)
if app is not None:sys.stdout.flush();os._exit(0)
if meta['status']=='failed':sys.exit(1)

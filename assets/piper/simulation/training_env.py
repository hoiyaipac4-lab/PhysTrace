"""Isaac Sim 5.1 scene adapter. Import only after creating SimulationApp.
Actions: six arm target angles [rad], full gripper opening [m]. No real robot IO.
"""
from pathlib import Path
import json,sys,numpy as np,xml.etree.ElementTree as ET
from PIL import Image
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysicsSchemaTools,Gf,Sdf
import omni.usd,omni.physx,omni.physics.tensors
import omni.replicator.core as rep

ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'PiPER_Scene_Assets_v1/scene'),str(ROOT/'摄像头支架_统一版_v1/runtime')]
from lift_controller import LiftController
from camera_mount_runtime import CameraMountRuntime

class TrainingScene:
 def __init__(self,app,config=None):
  self.app=app;self.cfg=json.loads((ROOT/'training_config.json').read_text())
  if config:self.cfg.update(config)
  joints={j.get('name'):j for j in ET.parse(ROOT/'robot_description/piper.urdf').getroot().findall('joint')}
  self.bounds=np.array([[float(joints['joint'+str(i)].find('limit').get(k)) for k in ['lower','upper']] for i in range(1,9)])
  hz=self.cfg['physics_hz'];chz=self.cfg['control_hz']
  if hz<=0 or chz<=0 or hz%chz:raise ValueError('physics_hz must be an integer multiple of control_hz')
  self.dt=1/hz;self.substeps=int(hz/chz);self.ctx=omni.usd.get_context();self.products={};self.attached=False;self.episode=-1
  self.reset(self.cfg['seed'])

 def reset(self,seed=0):
  """Hard reset rebuilds the native scene/solver, not just qpos; slower but no hidden warm-start state."""
  self.close();self.episode+=1;self.seed=int(seed);self.rng=np.random.default_rng(seed)
  self.ctx.open_stage(str(ROOT/'scene_training.usda'));self.stage=self.ctx.get_stage();self.stage.SetEditTarget(self.stage.GetSessionLayer())
  assert UsdGeom.GetStageMetersPerUnit(self.stage)==1. and UsdGeom.GetStageUpAxis(self.stage)=='Z','Invalid world units/up axis'
  from scene_config import apply_config
  apply_config(self.stage,self.cfg)
  if self.stage.GetPrimAtPath('/World/TaskAssets/plate').IsActive():
   UsdPhysics.Joint(self.stage.GetPrimAtPath('/World/TaskAssets/plate/TableAttachment')).GetJointEnabledAttr().Set(bool(self.cfg['plate_fixed']))
  # Optional measured pair-specific values; no automatic material-name inference.
  for name,values in self.cfg.get('material_overrides',{}).items():
   prim=self.stage.GetPrimAtPath('/World/TaskAssets/'+name+'/Contact')
   if not prim:raise ValueError('Unknown material object '+name)
   mat=UsdPhysics.MaterialAPI(prim)
   for key,api in [('static_friction',mat.GetStaticFrictionAttr()),('dynamic_friction',mat.GetDynamicFrictionAttr()),('restitution',mat.GetRestitutionAttr())]:
    if key in values:
     value=float(values[key]);assert np.isfinite(value) and value>=0 and (key!='restitution' or value<=1);api.Set(value)
  hand=UsdGeom.Xformable(self.stage.GetPrimAtPath(self.cfg['hand_camera']));hand.MakeMatrixXform().Set(Gf.Matrix4d(*np.asarray(self.cfg['wrist_local_matrix']).reshape(-1).tolist()))
  for p in self.stage.Traverse():
   if p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get():
    p.AddAppliedSchema('PhysxContactReportAPI');p.CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0.)
  self.mount=CameraMountRuntime(self.stage,json.loads((ROOT/'摄像头支架_统一版_v1/runtime/spring_settings.json').read_text()),root_path='/World/CameraMount')
  for _ in range(3):self.app.update()
  self.sim=omni.physx.get_physx_simulation_interface();assert self.sim.attach_stage(self.ctx.get_stage_id());self.attached=True;self.sim_time=0.
  self.sim.simulate(self.dt,self.sim_time);self.sim.fetch_results();self.sim_time+=self.dt
  self.sv=omni.physics.tensors.create_simulation_view('numpy',stage_id=self.ctx.get_stage_id())
  self.arm=self.sv.create_articulation_view('/World/Desk/Joints/Anchor');self.names=list(self.arm.shared_metatype.dof_names);self.idx=[self.names.index('joint'+str(i)) for i in range(1,9)];self.ids=np.array([0],dtype=np.uint32)
  self.command=np.array(self.cfg['home'],dtype=float);q=self.arm.get_dof_positions().copy();q[0,self.idx]=self.command;self.arm.set_dof_positions(q,self.ids);self.arm.set_dof_velocities(np.zeros_like(q),self.ids);self.arm.set_dof_position_targets(q,self.ids)
  self.lift=LiftController(self.arm);self.lift.set_height(.635)
  self.object_views={}
  for name in ['cube_wood','cube_aluminum','cube_iron','cube_brass','plate']:
   if not self.stage.GetPrimAtPath('/World/TaskAssets/'+name).IsActive(): continue
   body=next(p for p in Usd.PrimRange(self.stage.GetPrimAtPath('/World/TaskAssets/'+name)) if p.HasAPI(UsdPhysics.RigidBodyAPI))
   self.object_views[name]=self.sv.create_rigid_body_view(str(body.GetPath()))
  self.appliances={}
  for name in ['microwave','airfryer']:
   if not self.stage.GetPrimAtPath('/World/TaskAssets/'+name).IsActive(): continue
   p=next(p for p in Usd.PrimRange(self.stage.GetPrimAtPath('/World/TaskAssets/'+name)) if p.IsA(UsdPhysics.Joint) and p.GetName().endswith('opening_joint'))
   self.appliances[name]=UsdPhysics.DriveAPI(p,'angular' if name=='microwave' else 'linear')
   self.appliances[name].GetTargetPositionAttr().Set(0)
  self.frames=[];self.products={};rep.orchestrator.set_capture_on_play(False)
  for name,path in [('head',self.cfg['head_camera']),('hand',self.cfg['hand_camera'])]:
   assert self.stage.GetPrimAtPath(path).IsA(UsdGeom.Camera),path
   product=rep.create.render_product(path,tuple(self.cfg['resolution']));rgb=rep.AnnotatorRegistry.get_annotator('rgb');depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');rgb.attach([product]);depth.attach([product]);self.products[name]=(product,rgb,depth,path)
  # Settle contact at a known initial state; the policy clock begins afterwards.
  for _ in range(int(.25/self.dt)):self._physics_step(record=False)
  self.control_step=0;self.episode_start=self.sim_time;return self.state()

 def _physics_step(self,record):
  self.lift.tick(self.dt);q=self.arm.get_dof_positions().copy();q[0,self.idx]=self.command;q[:,self.lift.joints]=(self.lift.command-self.lift.minimum)/2;self.arm.set_dof_position_targets(q,self.ids)
  self.mount.before_step(self.dt);self.sim.simulate(self.dt,self.sim_time);self.sim.fetch_results();self.sim_time+=self.dt
  if not np.isfinite(self.arm.get_dof_positions()).all():raise RuntimeError('Non-finite physics state: stop collecting this episode')
  if record:
   contacts=[];headers,data=self.sim.get_contact_report()
   for h in headers:
    paths=[str(PhysicsSchemaTools.intToSdfPath(h.collider0)),str(PhysicsSchemaTools.intToSdfPath(h.collider1))]
    if self.cfg.get('contact_scope','task')=='task' and not any('/Piper/' in p or '/TaskAssets/' in p for p in paths):continue
    for c in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
     contacts.append({'colliders':paths,'position_m':list(c.position),'normal':list(c.normal),'impulse_Ns':list(c.impulse),'separation_m':float(c.separation)})
   self.frames.append({'time_s':self.sim_time-self.episode_start,'dt_s':self.dt,'command':self.command.tolist(),'q':self.arm.get_dof_positions()[0,self.idx].tolist(),'qd':self.arm.get_dof_velocities()[0,self.idx].tolist(),'contacts':contacts})

 def step(self,action,appliances=None,record_contacts=False):
  a=np.asarray(action,dtype=float)
  if a.shape!=(7,) or not np.isfinite(a).all():raise ValueError('Action requires 7 finite numbers: six radian angles and full gripper width in metres')
  command=np.r_[a[:6],a[6]/2,-a[6]/2]
  if np.any(command<self.bounds[:,0]) or np.any(command>self.bounds[:,1]):raise ValueError('Action outside PiPER URDF joint/gripper limits')
  self.command=command
  if appliances:
   for name,value in appliances.items():
    limit=110 if name=='microwave' else .16
    if name not in self.appliances or not np.isfinite(value) or not 0<=value<=limit:raise ValueError('Invalid appliance target')
    self.appliances[name].GetTargetPositionAttr().Set(float(value))
  self.frames=[]
  for _ in range(self.substeps):self._physics_step(record_contacts)
  self.control_step+=1
  omni.physx.get_physx_interface().update_transformations(False,True,False);self.mount.after_step()
  return self.state()

 def state(self):
  q=self.arm.get_dof_positions()[0,self.idx].copy();qd=self.arm.get_dof_velocities()[0,self.idx].copy();assert np.isfinite(q).all() and np.isfinite(qd).all()
  return {'episode':self.episode,'seed':self.seed,'time_s':getattr(self,'control_step',0)/self.cfg['control_hz'],'physics_time_s':self.sim_time,'arm_positions_rad':q[:6].tolist(),'gripper_width_m':float(q[6]-q[7]),'joint_q':q.tolist(),'joint_qd':qd.tolist(),'command':self.command.tolist(),'table_height_m':self.lift.measured_height(),'objects':{name:{'xyz_xyzw':v.get_transforms()[0].tolist(),'linear_angular_velocity':v.get_velocities()[0].tolist()} for name,v in self.object_views.items()}}

 def observe(self):
  # No physics is advanced for rendering. Fail if a future API change violates this.
  omni.physx.get_physx_interface().update_transformations(False,True,False);self.mount.after_step()
  before=self.arm.get_dof_positions().copy();rep.orchestrator.step(delta_time=0.,pause_timeline=False)
  for _ in range(3):self.app.update()
  after=self.arm.get_dof_positions();assert np.max(np.abs(after-before))<1e-6,'Rendering advanced physics unexpectedly'
  obs={'state':self.state(),'cameras':{}}
  for name,(product,rgb,depth,path) in self.products.items():
   image=np.asarray(rgb.get_data())[...,:3].copy();d=np.asarray(depth.get_data()).copy();w,h=self.cfg['resolution']
   assert image.shape==(h,w,3) and image.dtype==np.uint8 and image.std()>2,(name,image.shape)
   assert d.shape==(h,w) and np.isfinite(d).any(),(name,'depth unavailable')
   camera=UsdGeom.Camera(self.stage.GetPrimAtPath(path));fx=w*camera.GetFocalLengthAttr().Get()/camera.GetHorizontalApertureAttr().Get();fy=h*camera.GetFocalLengthAttr().Get()/camera.GetVerticalApertureAttr().Get();K=[[fx,0,w/2],[0,fy,h/2],[0,0,1]]
   mw,mh=self.cfg['model_resolution'];scale=min(mw/w,mh/h);rw,rh=round(w*scale),round(h*scale);ox,oy=(mw-rw)//2,(mh-rh)//2;model=Image.new('RGB',(mw,mh));model.paste(Image.fromarray(image).resize((rw,rh),Image.Resampling.BILINEAR),(ox,oy))
   obs['cameras'][name]={'rgb':image,'depth_m':d,'depth_valid':np.isfinite(d)&(d>0),'model_rgb':np.asarray(model),'intrinsics':K,'world_from_camera_row_matrix':np.array(UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(0)).tolist(),'time_s':obs['state']['time_s'],'resize_and_pad':{'scale':scale,'offset_xy':[ox,oy]},'depth_convention':'distance to image plane, metres; invalid values masked','camera_axes':'USD +X right, +Y up, -Z forward'}
  return obs

 def close(self):
  # This adapter owns the entire stage. Closing it removes the render graphs;
  # recursive annotator detach can traverse stale Replicator nodes after capture.
  products=list(self.products.values());self.products={}
  if self.attached:
   self.mount.reset();self.sim.detach_stage();self.attached=False
   if hasattr(self,'sv'):self.sv.invalidate()
  if self.ctx.get_stage():self.ctx.close_stage()
  from omni.syntheticdata import SyntheticData
  SyntheticData.Get().reset(usd=False)
  for product,rgb,depth,path in products:product.destroy()
  for _ in range(2):self.app.update()

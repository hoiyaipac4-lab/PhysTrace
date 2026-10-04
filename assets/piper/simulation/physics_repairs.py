"""Evidence-based corrections; preserves supplied source USD and URDF bytes."""
import json,xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,Sdf

def rotation(q):return np.array(Gf.Matrix3d(q)).T

def set_inertia(api,m,c,I):
 values,vectors=np.linalg.eigh(I)
 if np.linalg.det(vectors)<0:vectors[:,0]*=-1
 q=Rotation.from_matrix(vectors).as_quat()
 api.CreateMassAttr().Set(float(m));api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*c));api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*values));api.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))

def apply_repairs(stage,base):
 changes=[];xml=E.parse(base/'robot_description/piper.urdf').getroot()
 for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Piper')):
  if p.HasAPI(UsdPhysics.RigidBodyAPI):
   a=UsdPhysics.MassAPI(p);q=a.GetPrincipalAxesAttr().Get()
   # Baseline reconstructed I=R D R^T disagrees with source; conjugate exactly restores all 9 source tensors.
   a.GetPrincipalAxesAttr().Set(q.GetConjugate())
   api=PhysxSchema.PhysxRigidBodyAPI.Apply(p);api.CreateEnableCCDAttr().Set(True);api.CreateSolverVelocityIterationCountAttr().Set(4)
   changes.append({'prim':str(p.GetPath()),'repair':'URDF inertia principal-axis quaternion conjugated to correct column/row convention'})
 for name in ['joint7','joint8']:
  p=stage.GetPrimAtPath('/World/Piper/Physics/'+name);v=float(xml.find("joint[@name='%s']/limit"%name).get('velocity'))
  PhysxSchema.PhysxJointAPI.Apply(p).CreateMaxJointVelocityAttr().Set(v)
  changes.append({'prim':str(p.GetPath()),'repair':'Prismatic velocity restored from URDF in m/s','value':v})
 art=PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath('/World/Desk/Joints/Anchor'));art.CreateEnabledSelfCollisionsAttr().Set(True);art.CreateSolverVelocityIterationCountAttr().Set(4)
 # Static environment retains source geometry; no dynamic furniture is implied.
 pm=UsdShade.Material.Define(stage,'/World/EnvironmentContact');a=UsdPhysics.MaterialAPI.Apply(pm.GetPrim());a.CreateStaticFrictionAttr().Set(.5);a.CreateDynamicFrictionAttr().Set(.4);a.CreateRestitutionAttr().Set(0)
 count=0
 for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Room')):
  if p.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Set(False)
  if p.IsA(UsdPhysics.Joint):UsdPhysics.Joint(p).CreateJointEnabledAttr().Set(False)
  if p.HasAPI(UsdPhysics.CollisionAPI):
   UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(True)
   if p.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set('none')
   a=PhysxSchema.PhysxCollisionAPI.Apply(p);a.CreateContactOffsetAttr().Set(.001);a.CreateRestOffsetAttr().Set(0)
   UsdShade.MaterialBindingAPI.Apply(p).Bind(pm,materialPurpose='physics');count+=1
 changes.append({'repair':'Enable static triangle-mesh environment collision preserving openings','count':count})
 # Telescoping legs are hollow: solid box proxies incorrectly fill their bores.
 for group,names in [('BaseLink',['Base_002','Base_005']),('MiddleLink',['Middle','Middle_001']),('TopLink',['Top_014','Top_015'])]:
  for name in names:
   old=stage.GetPrimAtPath('/World/Desk/'+group+'/Colliders/'+name)
   parent=stage.GetPrimAtPath('/World/Desk/'+group)
   candidates=[p for p in Usd.PrimRange(parent) if p.IsA(UsdGeom.Mesh) and p.GetParent().GetName()==name]
   assert len(candidates)==1,(group,name,[str(p.GetPath()) for p in candidates])
   actual=candidates[0];UsdPhysics.CollisionAPI.Apply(actual).CreateCollisionEnabledAttr().Set(True)
   UsdPhysics.MeshCollisionAPI.Apply(actual).CreateApproximationAttr().Set('sdf');PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(actual).CreateSdfResolutionAttr().Set(256)
   c=PhysxSchema.PhysxCollisionAPI.Apply(actual);c.CreateContactOffsetAttr().Set(.0005);c.CreateRestOffsetAttr().Set(0)
   UsdPhysics.CollisionAPI(old).GetCollisionEnabledAttr().Set(False)
   changes.append({'repair':'Replace solid telescoping-leg box with source hollow tube SDF','old':str(old.GetPath()),'new':str(actual.GetPath())})
 # Recompose top assembly mass properties: source bare desk plus moved plate, no removed clamps.
 source=Usd.Stage.Open(str(base/'PiPER_Scene_Assets_v1/scene/desk/lift_table.usda'))
 bare=next(p for p in source.Traverse() if p.GetName()=='TopLink' and p.HasAPI(UsdPhysics.RigidBodyAPI));a=UsdPhysics.MassAPI(bare)
 m=float(a.GetMassAttr().Get());c=np.array(a.GetCenterOfMassAttr().Get());r=rotation(a.GetPrincipalAxesAttr().Get());I=r@np.diag(a.GetDiagonalInertiaAttr().Get())@r.T
 bp=stage.GetPrimAtPath('/World/Desk/TopLink/Board');points=np.array(UsdGeom.Mesh(bp).GetPointsAttr().Get());mat=np.array(UsdGeom.Xformable(bp).ComputeLocalToWorldTransform(0));points=points@mat[:3,:3]+mat[3,:3]
 # Explicit inherited nominal steel mass with uniform bounding-box tensor; this is an estimate, not metrology.
 bm=float(json.loads((base/'PiPER_Scene_Assets_v1/scene/asset/plate_parameters.json').read_text())['mass_estimate_kg']);lo=points.min(0);hi=points.max(0);bc=(lo+hi)/2;d=hi-lo;bi=bm/12*np.diag([d[1]**2+d[2]**2,d[0]**2+d[2]**2,d[0]**2+d[1]**2]);cc=(m*c+bm*bc)/(m+bm)
 def parallel(m,d):return m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
 set_inertia(UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/Desk/TopLink')),m+bm,cc,I+bi+parallel(m,c-cc)+parallel(bm,bc-cc))
 changes.append({'repair':'Recompose top mass, COM and inertia after removal of embedded clamps and plate displacement','mass_kg':m+bm,'com_m':cc.tolist(),'basis':'Bare-desk inherited estimate + plate nominal mass and bbox inertia; not calibrated'})
 (base/'evidence/physics_repairs.json').write_text(json.dumps(changes,indent=2))
 return changes

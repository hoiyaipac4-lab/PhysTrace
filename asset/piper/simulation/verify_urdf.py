from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np
from scipy.spatial.transform import Rotation as R
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960})
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf
root=Path(__file__).resolve().parent;s=Usd.Stage.Open(str(root/'scene.usda'));xml=E.parse(root/'robot_description/piper.urdf').getroot()
def rot(q):return np.array(Gf.Matrix3d(q)).T
def origin(e):
 o=e.find('origin');return (np.fromstring(o.get('xyz','0 0 0'),sep=' '),R.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()) if o is not None else (np.zeros(3),np.eye(3))
def path(n):return '/World/Piper/Geometry/dummy_link'+('' if n in ['dummy_link','base_link'] else '/'+ '/'.join('link'+str(i) for i in range(1,(6 if n=='gripper_base' else int(n[4:]))+1)) if n not in ['link8'] else '/link1/link2/link3/link4/link5/link6/link8')
report={'joints':[],'inertias':[],'collision_meshes':[],'visual_meshes':[]}
for x in xml.findall('joint'):
 if x.get('type')=='fixed':continue
 p=s.GetPrimAtPath('/World/Piper/Physics/'+x.get('name'));j=UsdPhysics.Joint(p);pos,rr=origin(x);a=np.fromstring(x.find('axis').get('xyz'),sep=' ')
 r0=rot(j.GetLocalRot0Attr().Get());r1=rot(j.GetLocalRot1Attr().Get());p0=np.array(j.GetLocalPos0Attr().Get());p1=np.array(j.GetLocalPos1Attr().Get());axis=np.eye(3)[:,['X','Y','Z'].index(p.GetAttribute('physics:axis').Get())]
 lim=x.find('limit');scale=180/np.pi if x.get('type')=='revolute' else 1
 drive=UsdPhysics.DriveAPI(p,'angular' if scale!=1 else 'linear')
 report['joints'].append({'name':x.get('name'),'frame_position_error_m':float(np.linalg.norm(p0-r0@r1.T@p1-pos)),'frame_rotation_error_rad':float(R.from_matrix(r0@r1.T@rr.T).magnitude()),'axis_error':float(np.linalg.norm(r0@axis-rr@a)),'limit_error':max(abs(p.GetAttribute('physics:'+k+'Limit').Get()/scale-float(lim.get(k))) for k in ['lower','upper']),'velocity_source':float(lim.get('velocity')),'velocity_usd':p.GetAttribute('physxJoint:maxJointVelocity').Get()/scale,'effort_source':float(lim.get('effort')),'drive_max_force':drive.GetMaxForceAttr().Get()})
# Merge fixed inertials using parallel-axis theorem, matching importer topology.
for n in ['base_link']+['link'+str(i) for i in range(1,9)]:
 members=[n]+(['gripper_base'] if n=='link6' else []);parts=[]
 for name in members:
  l=xml.find("link[@name='%s']"%name);ie=l.find('inertial')
  if ie is None:continue
  m=float(ie.find('mass').get('value'));c,rr=origin(ie);a=ie.find('inertia').attrib;ii=np.array([[float(a['i'+u+v] if 'i'+u+v in a else a['i'+v+u]) for v in 'xyz'] for u in 'xyz']);parts.append((m,c,rr@ii@rr.T))
 mass=sum(t[0] for t in parts);com=sum(m*c for m,c,_ in parts)/mass
 inertia=sum(ii+m*(np.dot(c-com,c-com)*np.eye(3)-np.outer(c-com,c-com)) for m,c,ii in parts)
 p=s.GetPrimAtPath(path(n));api=UsdPhysics.MassAPI(p);rr=rot(api.GetPrincipalAxesAttr().Get());ui=rr@np.diag(api.GetDiagonalInertiaAttr().Get())@rr.T
 report['inertias'].append({'link':n,'mass_error_kg':abs(mass-api.GetMassAttr().Get()),'com_error_m':float(np.linalg.norm(com-np.array(api.GetCenterOfMassAttr().Get()))),'inertia_max_abs_error_kgm2':float(np.max(np.abs(inertia-ui))),'eigenvalues':np.linalg.eigvalsh(inertia).tolist(),'triangle_inequality':bool(2*max(np.linalg.eigvalsh(inertia))<=np.trace(inertia)+1e-9)})
for l in xml.findall('link'):
 if l.get('name') in ['dummy_link','piper_tcp']:continue
 for kind in ['collision','visual']:
  for elem in l.findall(kind):
   mesh=elem.find('geometry/mesh');f=Path(mesh.get('filename'));name=f.stem
   candidates=[p for p in Usd.PrimRange(s.GetPrimAtPath(path(l.get('name')))) if p.IsA(UsdGeom.Mesh) and (p.GetParent().GetName()==name or (name=='gripper_base' and p.GetParent().GetName()=='gripper_base_1')) and (p.HasAPI(UsdPhysics.CollisionAPI)==(kind=='collision'))]
   pts=[]
   for line in (root/'robot_description'/f).open():
    if line.startswith('v '):pts.append([float(v) for v in line.split()[1:4]])
   if len(candidates)!=1:report[kind+'_meshes'].append({'name':name,'candidate_count':len(candidates)});continue
   p=candidates[0];up=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get());sp=np.array(pts);err=float(np.max(np.abs(up-sp))) if up.shape==sp.shape else None
   report[kind+'_meshes'].append({'name':name,'usd':str(p.GetPath()),'source_vertices':len(sp),'usd_vertices':len(up),'point_error_m':err})
assert all(j['frame_position_error_m']<1e-6 and j['frame_rotation_error_rad']<1e-6 and j['axis_error']<1e-6 and j['limit_error']<1e-6 and abs(j['velocity_usd']-j['velocity_source'])<1e-5 for j in report['joints'])
assert all(j['mass_error_kg']<1e-6 and j['com_error_m']<1e-6 and j['inertia_max_abs_error_kgm2']<1e-7 and j['triangle_inequality'] for j in report['inertias'])
assert all(j.get('point_error_m',1) is not None and j.get('point_error_m',1)<1e-6 for k in ['collision_meshes','visual_meshes'] for j in report[k])
report['status']='PASS'
(root/'evidence/urdf_comparison.json').write_text(json.dumps(report,indent=2));print('URDF_REPORT',json.dumps({k:v if k in ['joints','inertias'] else {'count':len(v),'bad':[i for i in v if i.get('point_error_m',1) is None or i.get('point_error_m',1)>1e-6]} for k,v in report.items() if k!='status'}),flush=True)
app.close()

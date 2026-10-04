"""Compare native link readback against independent URDF forward kinematics."""
from pathlib import Path
import json,xml.etree.ElementTree as E,numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parent;r=json.loads((root/'evidence/runtime.json').read_text());xml=E.parse(root/'robot_description/piper.urdf').getroot();nj={j.get('name'):j for j in xml.findall('joint')}
def origin(j):
 o=j.find('origin');M=np.eye(4)
 if o is not None:M[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');M[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix()
 return M
def pose(p):
 M=np.eye(4);M[:3,:3]=Rotation.from_quat(p[3:7]).as_matrix();M[:3,3]=p[:3];return M
stats={};records=[]
for sample in r['samples']:
 native={name:pose(p) for name,p in zip(r['link_names'],sample['link_poses'])};q=dict(zip(r['dof_names'],sample['q']));fk={'dummy_link':native['dummy_link']}
 pending=list(xml.findall('joint'))
 while pending:
  progress=False
  for j in pending[:]:
   parent=j.find('parent').get('link');child=j.find('child').get('link')
   if parent not in fk:continue
   M=origin(j);motion=np.eye(4);ax=j.find('axis')
   if ax is not None:
    axis=np.fromstring(ax.get('xyz'),sep=' ');angle=q.get(j.get('name'),0)
    if j.get('type')=='revolute':motion[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
    elif j.get('type')=='prismatic':motion[:3,3]=axis*angle
   fk[child]=fk[parent]@M@motion;pending.remove(j);progress=True
  assert progress,'URDF graph is disconnected or cyclic'
 for name in ['link'+str(i) for i in range(1,9)]:
  a=fk[name];b=native[name];pe=float(np.linalg.norm(a[:3,3]-b[:3,3]));re=float(Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude());st=stats.setdefault(name,{'max_position_error_m':0.,'max_rotation_error_rad':0.});st['max_position_error_m']=max(st['max_position_error_m'],pe);st['max_rotation_error_rad']=max(st['max_rotation_error_rad'],re)
 records.append({'t':sample['t'],'urdf_tcp_world_position':fk['piper_tcp'][:3,3].tolist()})
result={'status':'PASS' if all(v['max_position_error_m']<.0002 and v['max_rotation_error_rad']<np.radians(.05) for v in stats.values()) else 'FAIL','scope':'Independent URDF FK vs native Isaac link poses at measured q; relative to native base. Not real-arm calibration.','sample_count':len(r['samples']),'links':stats,'tcp_samples':records,'thresholds':{'position_m':.0002,'rotation_deg':.05}}
(root/'evidence/runtime_fk_comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='tcp_samples'},indent=2));assert result['status']=='PASS'

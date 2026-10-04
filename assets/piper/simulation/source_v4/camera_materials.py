"""Bind the four original wrist-camera textures; never change geometry or physics."""
from pathlib import Path
import xml.etree.ElementTree as E
R=Path(__file__).resolve().parent
PARTS=('camera_body','mount_bracket','mount_screw_01','mount_screw_02')
def texture(part):
 p=R/'assets/piper/visual_refresh'/(part+'_base_color.png')
 if not p.is_file():raise FileNotFoundError(p)
 return str(p)
def mujoco(root):
 a=root.find('asset');report=[]
 for mesh in a.findall('mesh'):
  part=Path(mesh.get('file','')).stem
  if part not in PARTS:continue
  name='original_camera_'+part
  if a.find(f"texture[@name='{name}']") is None:E.SubElement(a,'texture',name=name,type='2d',file=texture(part))
  if a.find(f"material[@name='{name}']") is None:E.SubElement(a,'material',name=name,texture=name,texuniform='false',rgba='1 1 1 1',specular='.1',shininess='.2')
  matches=[g for g in root.findall('.//geom') if g.get('mesh')==mesh.get('name') and g.get('contype','1')=='0' and g.get('conaffinity','1')=='0']
  for g in matches:g.set('material',name);g.set('rgba','1 1 1 1')
  report.append({'part':part,'visual_geoms':len(matches),'texture':texture(part)})
 return report
def gazebo(root):
 report=[]
 for visual in root.findall('.//visual'):
  mesh=visual.find('geometry/mesh/uri')
  if mesh is None:continue
  part=Path(mesh.text).stem
  if part not in PARTS:continue
  old=visual.find('material')
  if old is not None:visual.remove(old)
  mat=E.SubElement(visual,'material');E.SubElement(mat,'ambient').text='1 1 1 1';E.SubElement(mat,'diffuse').text='1 1 1 1'
  metal=E.SubElement(E.SubElement(mat,'pbr'),'metal');E.SubElement(metal,'albedo_map').text=texture(part);E.SubElement(metal,'roughness').text='.6';E.SubElement(metal,'metalness').text='0'
  report.append({'part':part,'texture':texture(part)})
 return report

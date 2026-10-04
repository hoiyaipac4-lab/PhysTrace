"""Reuse measured-asset atlas via UV coordinates, without modifying its pixels."""
from pathlib import Path
import json
ROOT=Path(__file__).parent
ATLAS='T_Box25_C.png'
def box_mesh(size,rect):
    import numpy as np
    vertices=[];normals=[];uv=[];indices=[]
    u0,v0,u1,v1=rect
    for axis,sign in [(0,1),(0,-1),(1,1),(1,-1),(2,1),(2,-1)]:
        n=np.zeros(3);n[axis]=sign
        a=np.zeros(3);a[(axis+1)%3]=1;b=np.cross(n,a)
        offset=len(vertices)
        for aa,bb in [(-1,-1),(1,-1),(1,1),(-1,1)]:vertices.append(((n+a*aa+b*bb)*np.asarray(size)/2).tolist());normals.append(n.tolist())
        uv.extend([(u0,v0),(u1,v0),(u1,v1),(u0,v1)]);indices.extend([offset,offset+1,offset+2,offset,offset+2,offset+3])
    return dict(vertices=vertices,normals=normals,uvs=uv,indices=indices)
def build():
    for name,size,rect in [('table',[.9,.7,.05],[.04,.445,.19,.56])]:
        data=box_mesh(size,rect);(ROOT/(name+'_visual.json')).write_text(json.dumps(data))
        lines=['mtllib '+name+'_visual.mtl','usemtl surface']
        lines += ['v '+' '.join(map(str,v)) for v in data['vertices']]
        lines += ['vt '+' '.join(map(str,v)) for v in data['uvs']]
        lines += ['vn '+' '.join(map(str,v)) for v in data['normals']]
        for j in range(0,len(data['indices']),3):lines.append('f '+' '.join(f'{i+1}/{i+1}/{i+1}' for i in data['indices'][j:j+3]))
        (ROOT/(name+'_visual.obj')).write_text('\n'.join(lines)+'\n')
        (ROOT/(name+'_visual.mtl')).write_text('newmtl surface\nKa 1 1 1\nKd 1 1 1\nKs .3 .3 .3\nNs 60\nmap_Kd '+ATLAS+'\n')
if __name__=='__main__':build()

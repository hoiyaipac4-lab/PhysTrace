"""Shared measured plate, native visual and collision adapters. No dynamics replay."""
from pathlib import Path
import json
import numpy as np
import xml.etree.ElementTree as E
R=Path(__file__).resolve().parent
A=R/'assets/plate'
def vals(v):return ' '.join(map(str,v))
def pose(c):return c['plate_position']
def parts():return sorted(A.glob('plate_col_*.obj'))
def enabled(c):return c.get('target_kind')=='plate'

def bullet(p,c,m):
    if not enabled(c):return None
    b=p.loadURDF(str(A/'plate.urdf'),basePosition=pose(c),useFixedBase=True,flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    p.changeDynamics(b,-1,lateralFriction=m['floor_mu'],restitution=0,rollingFriction=0,spinningFriction=0)
    return b

def mujoco(root,c,m):
    if not enabled(c):return
    a=root.find('asset');w=root.find('worldbody')
    E.SubElement(a,'texture',name='plate_tex',type='2d',file=str(A/'base_color.png'))
    E.SubElement(a,'material',name='plate_mat',texture='plate_tex',texuniform='false',specular='.35',shininess='.4')
    E.SubElement(a,'mesh',name='plate_visual',file=str(A/'plate_visual.obj'))
    E.SubElement(w,'geom',name='plate_skin',type='mesh',mesh='plate_visual',pos=vals(pose(c)),material='plate_mat',contype='0',conaffinity='0',group='2')
    for f in parts():
        E.SubElement(a,'mesh',name=f.stem,file=str(f))
        E.SubElement(w,'geom',name=f.stem,type='mesh',mesh=f.stem,pos=vals(pose(c)),friction=f"{m['floor_mu']} 0 0",condim='3',rgba='0 0 0 0',group='3')

def drake(p,sg,c,m,props):
    if not enabled(c):return []
    from pydrake.all import RigidTransform,Mesh,Convex
    X=RigidTransform(pose(c));w=p.world_body()
    p.RegisterVisualGeometry(w,X,Mesh(str(A/'plate_visual.obj')),'plate_skin',[1,1,1,1])
    return [p.RegisterCollisionGeometry(w,X,Convex(str(f)),f.stem,props(m['floor_mu'])) for f in parts()]

def newton(b,n,wp,c,m):
    if not enabled(c):return
    first=b.shape_count
    # Use the official OBJ/URDF UV loader shared with the other textured assets.
    b.add_urdf(str(A/'plate.urdf'),xform=wp.transform(pose(c),wp.quat_identity()),floating=False,collapse_fixed_joints=True,ignore_inertial_definitions=False,enable_self_collisions=False,joint_ordering=None)
    for i in range(first,b.shape_count):
        if int(b.shape_flags[i])&int(n.ShapeFlags.VISIBLE):
            b.shape_source[i].texture=str(A/'base_color.png');b.shape_source[i].color=(1.,1.,1.)
        if int(b.shape_flags[i])&int(n.ShapeFlags.COLLIDE_SHAPES):
            b.shape_type[i]=int(n.GeoType.CONVEX_MESH);b.shape_material_mu[i]=m['floor_mu'];b.shape_material_mu_torsional[i]=0.;b.shape_material_mu_rolling[i]=0.

def gazebo(root,c,m):
    if not enabled(c):return
    model=E.SubElement(root.find('world'),'model',name='plate');E.SubElement(model,'static').text='true';E.SubElement(model,'pose').text=vals(pose(c))+' 0 0 0';link=E.SubElement(model,'link',name='plate')
    for f in [A/'plate_visual.obj',*parts()]:
        visual=f.name=='plate_visual.obj';node=E.SubElement(link,'visual' if visual else 'collision',name=f.stem)
        E.SubElement(E.SubElement(E.SubElement(node,'geometry'),'mesh'),'uri').text=str(f)
        if visual:
            mat=E.SubElement(node,'material');E.SubElement(mat,'diffuse').text='1 1 1 1';E.SubElement(mat,'ambient').text='1 1 1 1'
            pbr=E.SubElement(E.SubElement(mat,'pbr'),'metal');E.SubElement(pbr,'albedo_map').text=str(A/'base_color.png');E.SubElement(pbr,'roughness').text='.4'
        else:
            ode=E.SubElement(E.SubElement(E.SubElement(node,'surface'),'friction'),'ode')
            E.SubElement(ode,'mu').text=str(m['floor_mu']);E.SubElement(ode,'mu2').text=str(m['floor_mu'])

def isaac(stage,c,material):
    if not enabled(c):return
    from pxr import UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
    root=UsdGeom.Xform.Define(stage,'/World/Plate');root.AddTranslateOp().Set(Gf.Vec3d(*pose(c)))
    z=np.load(A/'visual.npz');mesh=UsdGeom.Mesh.Define(stage,'/World/Plate/Visual')
    mesh.CreatePointsAttr(z['vertices'].tolist());mesh.CreateFaceVertexCountsAttr([3]*len(z['faces']));mesh.CreateFaceVertexIndicesAttr(z['faces'].ravel().tolist());mesh.CreateSubdivisionSchemeAttr('none')
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set(z['uv'].reshape(-1,2).tolist())
    mat=UsdShade.Material.Define(stage,'/World/Plate/Material');shader=UsdShade.Shader.Define(stage,'/World/Plate/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.4)
    tex=UsdShade.Shader.Define(stage,'/World/Plate/Material/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(A/'base_color.png'));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
    uv=UsdShade.Shader.Define(stage,'/World/Plate/Material/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2);tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    z=np.load(A/'collision_parts.npz')
    for i in range(len(parts())):
        g=UsdGeom.Mesh.Define(stage,f'/World/Plate/Collision_{i:03d}');f=z[f'f{i}'];g.CreatePointsAttr(z[f'v{i}'].tolist());g.CreateFaceVertexCountsAttr([3]*len(f));g.CreateFaceVertexIndicesAttr(f.ravel().tolist());g.CreateSubdivisionSchemeAttr('none');g.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(g.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr().Set('convexHull');UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(material,materialPurpose='physics')

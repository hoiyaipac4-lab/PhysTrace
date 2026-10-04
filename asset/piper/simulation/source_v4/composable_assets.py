"""Small native URDF composition interface. Asset poses in metres and xyzw."""
from pathlib import Path
import xml.etree.ElementTree as E
import json,copy,tempfile
import numpy as np
R=Path(__file__).resolve().parent
def yaw(a):return float(a.get('yaw_deg',0))*np.pi/180
def quaternion(a):return [0.,0.,float(np.sin(yaw(a)/2)),float(np.cos(yaw(a)/2))]
def assets(c):return c.get('assets',[])
def vals(x):return ' '.join(map(str,x))
def path(a):return str(R/a['urdf']) if not Path(a['urdf']).is_absolute() else a['urdf']
def root_link(a):
    r=E.parse(path(a)).getroot();children={n.get('link') for n in r.findall('joint/child')};return next(n.get('name') for n in r.findall('link') if n.get('name') not in children)
def mesh_texture(file):
    file=Path(file)
    if not file.is_file():return None
    for line in file.read_text().splitlines():
        if line.startswith('mtllib '):
            mtl=file.parent/line[7:].strip()
            if mtl.exists():
                for m in mtl.read_text().splitlines():
                    if m.startswith('map_Kd '):
                        t=mtl.parent/m[7:].strip()
                        if t.exists():return str(t)
    return None
def original_cube(a):
    return Path(path(a)).parent.name.startswith('cube_')
def original_color(a):
    return (.588,.588,.588) if original_cube(a) else (1.,1.,1.)
def asset_texture(a):
    for el in E.parse(path(a)).findall('.//visual/geometry/mesh'):
        t=mesh_texture(el.get('filename'))
        if t:return t
    return None
def mu(engine,c,a):
    t=float(c.get('mu',.4));g=float(c.get('grip_mu',.8));cube=a['name'].startswith('cube_')
    if engine=='pybullet':v=t**.5
    elif engine in ['mujoco','isaacsim']:v=min(t,g) if cube else t
    elif engine=='drake':v=max(t,g) if cube else t*max(t,g)/(2*max(t,g)-t)
    elif engine=='gazebo':v=max(t,g) if cube else t
    else:v=min(g,2*t) if cube else 2*t-min(g,2*t)
    return v
def bullet(p,c):
    ids=[]
    for a in assets(c):
        bid=p.loadURDF(path(a),basePosition=a['position'],baseOrientation=quaternion(a),useFixedBase=a.get('fixed',False),flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        for j in range(-1,p.getNumJoints(bid)):p.changeDynamics(bid,j,lateralFriction=mu('pybullet',c,a),rollingFriction=0,spinningFriction=0,restitution=0,linearDamping=0,angularDamping=0)
        for j in range(p.getNumJoints(bid)):
            if p.getJointInfo(bid,j)[2] in [p.JOINT_REVOLUTE,p.JOINT_PRISMATIC]:p.setJointMotorControl2(bid,j,p.VELOCITY_CONTROL,force=0)
        ids.append(bid)
    return ids
def mujoco(root,c):
    import mujoco as mj
    asset=root.find('asset');wb=root.find('worldbody');contact=root.find('contact')
    if contact is None:contact=E.SubElement(root,'contact')
    for a in assets(c):
        pre=a['name']+'__';urdf=E.parse(path(a));compiler=E.SubElement(E.SubElement(urdf.getroot(),'mujoco'),'compiler',discardvisual='false',fusestatic='false',balanceinertia='true')
        with tempfile.TemporaryDirectory() as temp:
            source=Path(temp)/'asset.urdf';out=Path(temp)/'asset.xml';urdf.write(source);model=mj.MjModel.from_xml_path(str(source));mj.mj_saveLastXML(str(out),model);x=E.parse(out).getroot()
        # Prefix every reference symbol, while retaining file paths and literal settings.
        refs=['name','mesh','material','texture','joint','body1','body2','geom1','geom2','site','target']
        for el in x.iter():
            for key in refs:
                if key in el.attrib:el.set(key,pre+el.get(key))
        xa=x.find('asset')
        if xa is not None:
            for el in xa:asset.append(el)
            for el in xa.findall('mesh'):
                texture=mesh_texture(el.get('file','')) or asset_texture(a)
                if texture:
                    name=el.get('name')+'_texture';E.SubElement(asset,'texture',name=name,type='2d',file=texture);E.SubElement(asset,'material',name=name,texture=name,texuniform='false',rgba=vals([*original_color(a),1]))
                    for g in x.findall('.//geom'):
                        if g.get('mesh')==el.get('name') and g.get('contype','1')=='0':g.set('material',name);g.set('rgba','1 1 1 1')
        body=E.SubElement(wb,'body',name=pre+'placement',pos=vals(a['position']),quat=vals([quaternion(a)[3],*quaternion(a)[:3]]))
        if not a.get('fixed',False):E.SubElement(body,'freejoint',name=pre+'free')
        for el in x.find('worldbody'):body.append(el)
        # Internal filter matches fixture URDF tests; external robot/object contact retained.
        names=[b.get('name') for b in body.iter('body')]
        if a.get('fixed',False):
            for i,n in enumerate(names):
                for n2 in names[i+1:]:E.SubElement(contact,'exclude',body1=n,body2=n2)
        for geom in body.iter('geom'):
            if geom.get('contype','1')!='0':geom.set('rgba','0 0 0 0');geom.set('group','3');geom.set('friction',f"{mu('mujoco',c,a)} 0 0");geom.set('condim','3')
def drake(p,sg,c):
    from pydrake.all import Parser,RigidTransform,RotationMatrix,ProximityProperties,AddContactMaterial,CoulombFriction,RoleAssign
    ids=[]
    for a in assets(c):
        parser=Parser(p,sg,a['name']);model=parser.AddModels(path(a))[0]
        root=p.GetBodyByName(root_link(a),model)
        if a.get('fixed',False):p.WeldFrames(p.world_frame(),root.body_frame(),RigidTransform(RotationMatrix.MakeZRotation(yaw(a)),a['position']))
        else:p.SetDefaultFreeBodyPose(root,RigidTransform(RotationMatrix.MakeZRotation(yaw(a)),a['position']))
        for bi in p.GetBodyIndices(model):
            for gid in p.GetCollisionGeometriesForBody(p.get_body(bi)):
                prop=ProximityProperties();v=mu('drake',c,a);AddContactMaterial(0.,1e7,CoulombFriction(v,v),prop);sg.AssignRole(p.get_source_id(),gid,prop,RoleAssign.kReplace)
        ids.append(model)
    return ids
def newton(b,n,wp,c):
    for a in assets(c):
        first=b.shape_count
        b.add_urdf(path(a),xform=wp.transform(a['position'],wp.quat(*quaternion(a))),floating=not a.get('fixed',False),collapse_fixed_joints=True,force_position_velocity_actuation=True,ignore_inertial_definitions=False,enable_self_collisions=False,joint_ordering=None)
        for i in range(first,b.shape_count):
            b.shape_material_mu[i]=mu('newton',c,a);b.shape_material_mu_torsional[i]=0.;b.shape_material_mu_rolling[i]=0.
            if int(b.shape_flags[i])&int(n.ShapeFlags.VISIBLE) and isinstance(b.shape_source[i],n.Mesh):
                b.shape_source[i].texture=asset_texture(a);b.shape_source[i].color=original_color(a)
            if int(b.shape_type[i])==int(n.GeoType.MESH) and int(b.shape_flags[i])&int(n.ShapeFlags.COLLIDE_SHAPES):b.shape_type[i]=int(n.GeoType.CONVEX_MESH)
        for i in range(len(b.joint_target_ke)):b.joint_target_ke[i]=0.;b.joint_target_kd[i]=0.
def gazebo(root,c):
    import subprocess
    for a in assets(c):
        model=E.fromstring(subprocess.check_output(['gz','sdf','-p',path(a)],text=True)).find('model');model.set('name',a['name']);pose=model.find('pose')
        if pose is None:pose=E.SubElement(model,'pose')
        pose.text=vals(a['position'])+' 0 0 '+str(yaw(a))
        if a.get('fixed',False):
            j=E.SubElement(model,'joint',name='asset_world_fixture',type='fixed');E.SubElement(j,'parent').text='world';E.SubElement(j,'child').text=root_link(a)
        for col in model.findall('.//collision'):
            old=col.find('surface')
            if old is not None:col.remove(old)
            ode=E.SubElement(E.SubElement(E.SubElement(col,'surface'),'friction'),'ode');E.SubElement(ode,'mu').text=str(mu('gazebo',c,a));E.SubElement(ode,'mu2').text=str(mu('gazebo',c,a))
        for v in model.findall('.//visual'):
            mesh=v.find('geometry/mesh/uri');texture=mesh_texture(mesh.text) if mesh is not None else None
            if texture:
                mat=v.find('material')
                if mat is not None:v.remove(mat)
                mat=E.SubElement(v,'material');E.SubElement(mat,'ambient').text=vals([*original_color(a),1]);E.SubElement(mat,'diffuse').text=vals([*original_color(a),1]);metal=E.SubElement(E.SubElement(mat,'pbr'),'metal');E.SubElement(metal,'albedo_map').text=texture;E.SubElement(metal,'roughness').text='.4'
        root.find('world').append(model)
def isaac(stage,c):
    import omni.kit.commands
    from pxr import UsdGeom,UsdPhysics,Gf,PhysxSchema,UsdShade,Sdf
    for a in assets(c):
        before={str(x.GetPath()) for x in stage.GetPseudoRoot().GetChildren()}
        stage.ClearDefaultPrim()
        _,cfg=omni.kit.commands.execute('URDFCreateImportConfig');cfg.fix_base=a.get('fixed',False);cfg.merge_fixed_joints=True;cfg.import_inertia_tensor=True;cfg.create_physics_scene=False
        if hasattr(cfg,'make_default_prim'):cfg.make_default_prim=False
        ok,res=omni.kit.commands.execute('URDFParseAndImportFile',urdf_path=path(a),import_config=cfg)
        if not ok:raise RuntimeError(str(res))
        added=[x for x in stage.GetPseudoRoot().GetChildren() if str(x.GetPath()) not in before]
        mat=UsdShade.Material.Define(stage,'/World/AssetMaterials/'+a['name']);phys=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());phys.CreateStaticFrictionAttr(mu('isaacsim',c,a));phys.CreateDynamicFrictionAttr(mu('isaacsim',c,a));phys.CreateRestitutionAttr(0.);PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr('max')
        texpath=asset_texture(a);look=None
        if texpath:
            look=UsdShade.Material.Define(stage,'/World/AssetLooks/'+a['name']);s=UsdShade.Shader.Define(stage,str(look.GetPath())+'/Surface');s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.4)
            tex=UsdShade.Shader.Define(stage,str(look.GetPath())+'/Albedo');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(texpath);tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3);tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*original_color(a),1))
            uv=UsdShade.Shader.Define(stage,str(look.GetPath())+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2);tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');look.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface')
        for prim in added:
            if prim.IsA(UsdGeom.Xform):
                xf=UsdGeom.Xformable(prim);xf.AddTranslateOp(opSuffix='placement').Set(Gf.Vec3d(*a['position']));xf.AddRotateZOp(opSuffix='placement').Set(float(a.get('yaw_deg',0)))
            # URDF primitives are nested instance proxies. Author collision schemas on
            # editable instances rather than silently skipping proxy descendants.
            Usd=__import__('pxr.Usd',fromlist=['PrimRange'])
            for _ in range(8):
                instances=[p for p in Usd.PrimRange(prim) if p.IsInstance() and '/collisions' in str(p.GetPath())]
                if not instances:break
                for p in instances:p.SetInstanceable(False)
            for p in __import__('pxr.Usd',fromlist=['PrimRange']).PrimRange(prim):
                if '/collisions/' in str(p.GetPath()) and p.IsA(UsdGeom.Gprim):
                    UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True)
                    if p.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexHull')
                    collision=PhysxSchema.PhysxCollisionAPI.Apply(p);collision.CreateContactOffsetAttr(.001);collision.CreateRestOffsetAttr(0.)
                    UsdGeom.Imageable(p).CreateVisibilityAttr('invisible')
                if p.HasAPI(UsdPhysics.CollisionAPI):UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,materialPurpose='physics')
                elif (p.IsA(UsdGeom.Mesh) or p.GetName()=='visuals') and look:UsdShade.MaterialBindingAPI.Apply(p).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
                if p.IsA(UsdPhysics.FixedJoint):
                    joint=UsdPhysics.Joint(p)
                    if not joint.GetBody0Rel().GetTargets():
                        joint.CreateLocalPos0Attr(Gf.Vec3f(*a['position']));q=quaternion(a);joint.CreateLocalRot0Attr(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
                for kind in ['angular','linear']:
                    if p.HasAPI(UsdPhysics.DriveAPI,kind):d=UsdPhysics.DriveAPI(p,kind);d.CreateStiffnessAttr(0);d.CreateDampingAttr(0)
                if p.HasAPI(UsdPhysics.RigidBodyAPI):rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p);rb.CreateLinearDampingAttr(0);rb.CreateAngularDampingAttr(0)
        if a['name'].startswith('cube_'):
            collision_count=sum(p.HasAPI(UsdPhysics.CollisionAPI) for root in added for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies()))
            if not collision_count:raise RuntimeError('No authored native collider for '+a['name'])

def read_native(w,engine,c):
    """Direct native body/joint readback for composed assets, with explicit missing channels."""
    if not assets(c):return {}
    if engine=='pybullet':
        return {a['name']:dict(pose=[*w.p.getBasePositionAndOrientation(b)[0],*w.p.getBasePositionAndOrientation(b)[1]],joints=[dict(name=w.p.getJointInfo(b,j)[1].decode(),q=w.p.getJointState(b,j)[0],v=w.p.getJointState(b,j)[1]) for j in range(w.p.getNumJoints(b))]) for a,b in zip(assets(c),w.additional_assets)}
    if engine=='mujoco':
        return {'body_names':[w.mj.mj_id2name(w.m,w.mj.mjtObj.mjOBJ_BODY,i) for i in range(w.m.nbody)],'position_world':w.d.xpos.tolist(),'quaternion_wxyz':w.d.xquat.tolist(),'qpos':w.d.qpos.tolist(),'qvel':w.d.qvel.tolist()}
    if engine=='drake':
        return {a['name']:dict(q=w.p.GetPositions(w.pc,model).tolist(),v=w.p.GetVelocities(w.pc,model).tolist(),bodies={w.p.get_body(i).name():w.p.get_body(i).EvalPoseInWorld(w.pc).translation().tolist() for i in w.p.GetBodyIndices(model)}) for a,model in zip(assets(c),w.additional_assets)}
    if engine=='newton':return dict(labels=w.m.body_label,pose=w.s.body_q.numpy().tolist(),joint_q=w.s.joint_q.numpy().tolist(),joint_qd=w.s.joint_qd.numpy().tolist())
    if engine=='gazebo':return w.cache.get('asset_states',{'missing_reason':'additional asset state unavailable in this callback'})
    if engine=='isaacsim':
        if not hasattr(w,'asset_readback'):
            from isaacsim.core.prims import RigidPrim
            from pxr import UsdPhysics
            paths=[str(p.GetPath()) for p in w.w.stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and not str(p.GetPath()).startswith(('/piper','/World/'))]
            w.asset_paths=paths;w.asset_readback=RigidPrim(paths);w.asset_readback.initialize()
        pos,quat=w.asset_readback.get_world_poses();return dict(paths=w.asset_paths,position_world=pos.tolist(),quaternion_wxyz=quat.tolist())

def gz_read(ecm,c):
    from gz.sim8 import World,Model,Link,Joint,world_entity
    out={};world=World(world_entity(ecm))
    for a in assets(c):
        m=Model(world.model_by_name(ecm,a['name']));link=Link(m.link_by_name(ecm,root_link(a)));X=link.world_pose(ecm)
        out[a['name']]={'position_world':[X.x(),X.y(),X.z()] if X else None}
    return out

def validate_dynamic_assets(state,engine,c):
    data=state.get('asset_states_native',{});result=[]
    for a in assets(c):
        if a.get('fixed',False):continue
        name=a['name'];body=root_link(a);position=None
        if engine=='pybullet':position=data[name]['pose'][:3]
        elif engine=='drake':position=data[name]['bodies'][body]
        elif engine=='gazebo':position=data[name]['position_world']
        elif engine=='mujoco':position=next(p for n,p in zip(data['body_names'],data['position_world']) if n.endswith(body))
        elif engine=='newton':position=next(p[:3] for n,p in zip(data['labels'],data['pose']) if n.endswith(body))
        elif engine=='isaacsim':position=next(p for n,p in zip(data['paths'],data['position_world']) if n.endswith('/'+body))
        valid=position is not None and bool(np.isfinite(position).all()) and .008<position[2]<.035 and np.linalg.norm(np.asarray(position[:2])-a['position'][:2])<.03
        result.append(dict(name=name,position_native=position,table_support_pass=bool(valid)))
    return result

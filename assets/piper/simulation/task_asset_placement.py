"""Add all foreground task categories from piper_scene_v4; source files remain separate."""
import json,xml.etree.ElementTree as E
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf,Sdf,Vt

def texture_material(stage,path,texture,color=(1,1,1)):
 mat=UsdShade.Material.Define(stage,path);sh=UsdShade.Shader.Define(stage,path+'/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.4)
 tex=UsdShade.Shader.Define(stage,path+'/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(texture);tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*color,1));tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
 uv=UsdShade.Shader.Define(stage,path+'/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');uv.CreateOutput('result',Sdf.ValueTypeNames.Float2);tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return mat

def restore_obj(mesh,file):
 points=[];uv=[];norm=[];indices=[];counts=[];corneruv=[];cornernorm=[]
 for line in file.open():
  a=line.split()
  if not a:continue
  if a[0]=='v':points.append(list(map(float,a[1:4])))
  elif a[0]=='vt':uv.append(list(map(float,a[1:3])))
  elif a[0]=='vn':norm.append(list(map(float,a[1:4])))
  elif a[0]=='f':
   counts.append(len(a)-1)
   for vertex in a[1:]:
    tokens=vertex.split('/');indices.append(int(tokens[0])-1)
    if len(tokens)>1 and tokens[1]:corneruv.append(uv[int(tokens[1])-1])
    if len(tokens)>2 and tokens[2]:cornernorm.append(norm[int(tokens[2])-1])
 mesh.GetPointsAttr().Set(points);mesh.GetFaceVertexIndicesAttr().Set(indices);mesh.GetFaceVertexCountsAttr().Set(counts);mesh.CreateSubdivisionSchemeAttr().Set('none');mesh.CreateExtentAttr().Set([Gf.Vec3f(*np.min(points,axis=0)),Gf.Vec3f(*np.max(points,axis=0))])
 if len(corneruv)==len(indices):UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set(corneruv)
 if len(cornernorm)==len(indices):mesh.GetNormalsAttr().Set(cornernorm);mesh.SetNormalsInterpolation('faceVarying')
 return {'vertices':len(points),'faces':len(counts),'uv_corners':len(corneruv)}

def place_task_assets(stage,base):
 catalog=json.loads((base/'source_v4/asset_catalog.json').read_text());records=[]
 specs=[('support_microwave',(-.41,.13),180,.635),('support_airfryer',(.40,.22),0,.635),('microwave',(-.41,.13),180,.735),('airfryer',(.40,.22),0,.735),('plate',(-.005,.13),0,.635)]
 specs += [('cube_'+name,(x,-.15),0,.637) for name,x in zip(['wood','aluminum','iron','brass'],[-.38,-.29,-.20,.14])]
 for name,xy,yaw,bottom in specs:
  path='/World/TaskAssets/'+name;root=UsdGeom.Xform.Define(stage,path);root.GetPrim().GetReferences().AddReference('./task_assets/'+name+'.usd')
  # Expand instances before local collision/material edits.
  for _ in range(8):
   inst=[p for p in Usd.PrimRange(root.GetPrim()) if p.IsInstance()]
   if not inst:break
   for p in inst:p.SetInstanceable(False)
  bodies=[p for p in Usd.PrimRange(root.GetPrim()) if p.HasAPI(UsdPhysics.RigidBodyAPI)];rootbody=stage.GetPrimAtPath(path+'/'+catalog[name].get('root_link',name+'__base'))
  assert rootbody,(name,[str(p.GetPath()) for p in bodies])
  # URDF importer applies box dimensions twice in this installed version. Reauthor exact URDF boxes.
  if name.startswith(('support_','cube_')):
   size=[.025]*3 if name.startswith('cube_') else catalog[name]['size_m'];bp=str(rootbody.GetPath())
   for p in list(Usd.PrimRange(rootbody)):
    if p.HasAPI(UsdPhysics.CollisionAPI):UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr().Set(False)
   if name.startswith('support_'):stage.GetPrimAtPath(bp+'/visuals').SetActive(False)
   c=UsdGeom.Cube.Define(stage,bp+'/ExactURDFBox');c.CreateSizeAttr(1);c.AddScaleOp().Set(Gf.Vec3f(*size));c.CreateExtentAttr([Gf.Vec3f(-.5),Gf.Vec3f(.5)]);UsdPhysics.CollisionAPI.Apply(c.GetPrim())
   if name.startswith('cube_'):c.CreateVisibilityAttr('invisible')
   else:c.CreateDisplayColorAttr([Gf.Vec3f(.3,.32,.35)])
  b=UsdGeom.BBoxCache(0,['default','render'],False,True).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange();z=bottom-b.GetMin()[2]
  root.AddTranslateOp(opSuffix='taskPlacement').Set(Gf.Vec3d(*xy,z));root.AddRotateZOp(opSuffix='taskPlacement').Set(yaw)
  # Every imported rigid body stays in world coordinates for nested-body compatibility.
  poses={str(p.GetPath()):UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0) for p in bodies}
  for p in bodies:
   xf=UsdGeom.Xformable(p);xf.ClearXformOpOrder();xf.AddTransformOp(opSuffix='installed').Set(poses[str(p.GetPath())]);xf.SetResetXformStack(True)
   rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p);rb.CreateEnableCCDAttr().Set(True);rb.CreateSolverPositionIterationCountAttr().Set(64);rb.CreateSolverVelocityIterationCountAttr().Set(4)
  if not name.startswith('cube_'):
   j=UsdPhysics.FixedJoint.Define(stage,path+'/TableAttachment');parent_path='/World/TaskAssets/support_'+name+'/support_'+name+'__base' if name in ['microwave','airfryer'] else '/World/Desk/TopLink';j.CreateBody0Rel().SetTargets([parent_path]);j.CreateBody1Rel().SetTargets([rootbody.GetPath()]);m=UsdGeom.Xformable(rootbody).ComputeLocalToWorldTransform(0)*UsdGeom.Xformable(stage.GetPrimAtPath(parent_path)).ComputeLocalToWorldTransform(0).GetInverse();j.CreateLocalPos0Attr(Gf.Vec3f(m.ExtractTranslation()));j.CreateLocalRot0Attr(Gf.Quatf(m.ExtractRotationQuat()));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLocalRot1Attr(Gf.Quatf(1));j.CreateExcludeFromArticulationAttr(True)
  physics=UsdShade.Material.Define(stage,path+'/Contact');pm=UsdPhysics.MaterialAPI.Apply(physics.GetPrim());pm.CreateStaticFrictionAttr(.5);pm.CreateDynamicFrictionAttr(.4);pm.CreateRestitutionAttr(0)
  for p in Usd.PrimRange(root.GetPrim()):
   if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():
    UsdShade.MaterialBindingAPI.Apply(p).Bind(physics,materialPurpose='physics');c=PhysxSchema.PhysxCollisionAPI.Apply(p);c.CreateContactOffsetAttr(.0005);c.CreateRestOffsetAttr(0)
    if p.IsA(UsdGeom.Mesh):UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexHull')
   if p.HasAPI(UsdPhysics.ArticulationRootAPI):
    art=PhysxSchema.PhysxArticulationAPI.Apply(p);art.CreateEnabledSelfCollisionsAttr(True);art.CreateSolverPositionIterationCountAttr(64);art.CreateSolverVelocityIterationCountAttr(4)
   if p.IsA(UsdPhysics.Joint) and p.GetName().endswith('opening_joint'):
    angular=p.IsA(UsdPhysics.RevoluteJoint);drive=UsdPhysics.DriveAPI.Apply(p,'angular' if angular else 'linear');drive.CreateStiffnessAttr(5 if angular else 100);drive.CreateDampingAttr(1 if angular else 10);drive.CreateMaxForceAttr(2 if angular else 10);drive.CreateTargetPositionAttr(0)
    PhysxSchema.PhysxJointAPI.Apply(p).CreateMaxJointVelocityAttr(573 if angular else 10)
  if name.startswith('cube_'):
   material=name[5:];look=texture_material(stage,path+'/Look','./source_v4/material_visual/'+material+'/T_Box25_C.png',(.588,.588,.588))
   for p in Usd.PrimRange(root.GetPrim()):
    if p.IsA(UsdGeom.Mesh) and '/visuals/' in str(p.GetPath()):UsdShade.MaterialBindingAPI.Apply(p).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
  if name in ['microwave','airfryer']:
   tex='./source_v4/assets/'+name+'/meshes/'+('T_WBL_C.png' if name=='microwave' else 'kqzg_D.png')
   look=texture_material(stage,path+'/OriginalAtlas',tex)
   # Imported ancestor bindings are stronger than descendant bindings.
   UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
   for p in Usd.PrimRange(root.GetPrim()):
    if p.IsA(UsdGeom.Mesh) and '/visuals/' in str(p.GetPath()):UsdShade.MaterialBindingAPI.Apply(p).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
  if name=='plate':
   look=texture_material(stage,path+'/Look','./source_v4/assets/plate/base_color.png')
   for p in Usd.PrimRange(root.GetPrim()):
    if p.IsA(UsdGeom.Mesh) and '/visuals/' in str(p.GetPath()):UsdShade.MaterialBindingAPI.Apply(p).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants)
  bb=UsdGeom.BBoxCache(0,['default','render'],False,True).ComputeWorldBound(root.GetPrim()).ComputeAlignedRange();records.append({'name':name,'path':path,'bounds':[list(bb.GetMin()),list(bb.GetMax())],'rigid_bodies':[str(p.GetPath()) for p in bodies],'support_model':'free dynamic' if name.startswith('cube_') else 'explicit tabletop fixture; not friction holding certification'})
 # Noncolliding green placement marker rides with the tabletop.
 marker=UsdGeom.Cube.Define(stage,'/World/Desk/TopLink/GreenTarget');marker.CreateSizeAttr(1);marker.AddTranslateOp().Set(Gf.Vec3d(.34,-.15,.63515));marker.AddScaleOp().Set(Gf.Vec3f(.06,.06,.0002));marker.CreateDisplayColorAttr([Gf.Vec3f(.02,.55,.12)])
 records.append({'name':'green_region','collision':False,'purpose':'visual target marker'})
 # V4 restores original camera PNG materials and original face-corner UVs.
 wrist='/World/Piper/Geometry/dummy_link/link1/link2/link3/link4/link5/link6'
 for name in ['camera_body','mount_bracket','mount_screw_01','mount_screw_02']:
  mesh=UsdGeom.Mesh(stage.GetPrimAtPath(wrist+'/'+name+'/SourceMesh'));r=restore_obj(mesh,base/'source_v4/assets/piper/visual_refresh'/f'{name}.obj');look=texture_material(stage,'/World/V4Looks/'+name,'./source_v4/assets/piper/visual_refresh/'+name+'_base_color.png');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(look,bindingStrength=UsdShade.Tokens.strongerThanDescendants);records.append({'name':'v4_'+name,'visual_restore':r})
 tcp=UsdGeom.Xform.Define(stage,wrist+'/piper_tcp');tcp.ClearXformOpOrder();tcp.AddTranslateOp(opSuffix='urdfTcp').Set(Gf.Vec3d(0,0,.09))
 (base/'evidence/task_asset_placement.json').write_text(json.dumps(records,indent=2))

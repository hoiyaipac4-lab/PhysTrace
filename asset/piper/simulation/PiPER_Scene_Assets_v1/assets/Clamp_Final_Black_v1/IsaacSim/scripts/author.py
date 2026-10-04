"""Author a scan-based C-clamp with explicit estimated physics and screw coupling."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,PhysxSchema,Gf,Sdf,Vt

def build(root):
 root=Path(root);p=json.loads((root/'parameters.json').read_text());props=p['part_properties'];raw=np.array(p['sim_source_origin_raw']);axis=np.array(p['axis_direction']);anchor=(np.array(p['axis_origin'])-raw)*.001;ha=np.array(p['handle_axis']);handle_anchor=(np.array(p['handle_origin'])-raw)*.001
 s=Usd.Stage.CreateNew(str(root/'Clamp.usdc'));rt=UsdGeom.Xform.Define(s,'/Clamp');s.SetDefaultPrim(rt.GetPrim());UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1);UsdPhysics.SetStageKilogramsPerUnit(s,1)
 rt.GetPrim().SetCustomData({'asset_id':'robot_arm_part_c_clamp','color':'black: user request','units':'meters; source millimeters assumed','physics':'estimated steel density 7800 kg/m3, friction .5; equivalent ideal screw joint','source_origin_raw':str(raw.tolist()),'source_scale':.001,'training_ready':False})
 art=UsdPhysics.ArticulationRootAPI.Apply(rt.GetPrim());pa=PhysxSchema.PhysxArticulationAPI.Apply(rt.GetPrim());pa.CreateEnabledSelfCollisionsAttr(True);pa.CreateSolverPositionIterationCountAttr(64);pa.CreateSolverVelocityIterationCountAttr(16)
 mat=UsdShade.Material.Define(s,'/Clamp/Black');shader=UsdShade.Shader.Define(s,'/Clamp/Black/Shader');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.006));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.4);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.15);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
 pm=UsdShade.Material.Define(s,'/Clamp/Contact');api=UsdPhysics.MaterialAPI.Apply(pm.GetPrim());api.CreateStaticFrictionAttr(.5);api.CreateDynamicFrictionAttr(.45);api.CreateRestitutionAttr(.02)
 centers={n:np.array(v['center']) for n,v in props.items()};centers['carriage']=anchor
 for n in ['frame','carriage','screw','pad','handle']:
  path='/Clamp/'+n;body=UsdGeom.Xform.Define(s,path);body.AddTranslateOp().Set(Gf.Vec3d(*centers[n]));pr=body.GetPrim();UsdPhysics.RigidBodyAPI.Apply(pr);mass=UsdPhysics.MassAPI.Apply(pr)
  mass.CreateMassAttr(.001 if n=='carriage' else props[n]['mass']);mass.CreateCenterOfMassAttr(Gf.Vec3f(0));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*([1e-7]*3 if n=='carriage' else props[n]['inertia'])))
  rb=PhysxSchema.PhysxRigidBodyAPI.Apply(pr);rb.CreateEnableCCDAttr(True);rb.CreateEnableSpeculativeCCDAttr(True);rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(16)
  if n=='carriage':pr.SetCustomData({'purpose':'virtual helper for independent prismatic/revolute DOF; not an extra physical part'});continue
  d=np.load(root/'mesh_data'/f'{n}.npz');mesh=UsdGeom.Mesh.Define(s,path+'/Mesh');mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(d['vertices']));mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(d['faces'].ravel()));mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(d['faces']),3,dtype='i4')));mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(d['normals']));mesh.SetNormalsInterpolation('vertex');mesh.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
  UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('sdf');sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(mesh.GetPrim());sdf.CreateSdfResolutionAttr(512 if n in ['frame','screw'] else 256);sdf.CreateSdfSubgridResolutionAttr(6);sdf.CreateSdfBitsPerSubgridPixelAttr('BitsPerPixel16');sdf.CreateSdfEnableRemeshingAttr(False)
  col=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim());col.CreateContactOffsetAttr(.002);col.CreateRestOffsetAttr(.00035);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(pm,materialPurpose='physics');PhysxSchema.PhysxContactReportAPI.Apply(pr).CreateThresholdAttr(0)
 def joint(name,kind,b0,b1,point,direction,lo,hi):
  j=kind.Define(s,'/Clamp/Joints/'+name);j.CreateBody0Rel().SetTargets(['/Clamp/'+b0]);j.CreateBody1Rel().SetTargets(['/Clamp/'+b1]);j.CreateLocalPos0Attr(Gf.Vec3f(*(point-centers[b0])));j.CreateLocalPos1Attr(Gf.Vec3f(*(point-centers[b1])));q=Gf.Rotation(Gf.Vec3d(1,0,0),Gf.Vec3d(*direction)).GetQuat();q=Gf.Quatf(q.GetReal(),Gf.Vec3f(q.GetImaginary()));j.CreateLocalRot0Attr(q);j.CreateLocalRot1Attr(q);j.CreateAxisAttr('X');j.CreateLowerLimitAttr(lo);j.CreateUpperLimitAttr(hi);j.CreateCollisionEnabledAttr(False);return j
 slide=joint('ScrewFeed',UsdPhysics.PrismaticJoint,'frame','carriage',anchor,axis,-.0003,.020)
 spin=joint('ScrewTurn',UsdPhysics.RevoluteJoint,'carriage','screw',anchor,axis,-40,2500)
 mimic=PhysxSchema.PhysxMimicJointAPI.Apply(slide.GetPrim(),'rotX');mimic.CreateReferenceJointRel().SetTargets([spin.GetPath()]);mimic.CreateGearingAttr(-p['thread_pitch_estimated']*.001/360);mimic.CreateOffsetAttr(0)
 d=UsdPhysics.DriveAPI.Apply(spin.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(2);d.CreateDampingAttr(.08);d.CreateMaxForceAttr(.4);d.CreateTargetPositionAttr(0)
 pad=joint('PadSwivel',UsdPhysics.RevoluteJoint,'screw','pad',anchor+axis*.0333,axis,-180,180)
 handle=joint('HandleSlide',UsdPhysics.PrismaticJoint,'screw','handle',handle_anchor,ha,*p['handle_slide_limits_m'])
 for j,kind in [(pad,'angular'),(handle,'linear')]:
  d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),kind);d.CreateTypeAttr('force');d.CreateStiffnessAttr(0);d.CreateDampingAttr(.00002 if kind=='angular' else .1);d.CreateMaxForceAttr(.02 if kind=='angular' else 2)
 # The equivalent screw pair carries internal thread reactions; avoid duplicate mesh reactions.
 UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/Clamp/frame')).CreateFilteredPairsRel().SetTargets(['/Clamp/screw'])
 s.GetRootLayer().Save();return p

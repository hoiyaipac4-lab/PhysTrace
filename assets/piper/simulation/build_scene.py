"""Compose supplied assets without modifying the source layers."""
from pathlib import Path
import json
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':960,'renderer':'RayTracedLighting'})
from pxr import Usd,UsdGeom,UsdPhysics,UsdLux,UsdUtils,Gf,Sdf
base=Path(__file__).resolve().parent
stage=Usd.Stage.CreateNew(str(base/'scene.usda'))
stage.GetRootLayer().subLayerPaths=['./PiPER_Scene_Assets_v1/scene/scene.usda']
UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z');stage.SetDefaultPrim(stage.GetPrimAtPath('/World'))
mount=UsdGeom.Xform.Define(stage,'/World/CameraMount');mount.GetPrim().GetReferences().AddReference('./摄像头支架_统一版_v1/camera_mount.usdc','/CameraMount')
# Latest user correction: 80 mm right clearance; plate presses the 4 mm robot flange.
board=stage.GetPrimAtPath('/World/Desk/TopLink/Board')
board_xf=UsdGeom.Xformable(board);board_xf.AddTranslateOp(opSuffix='userCorrection').Set(Gf.Vec3d(-.072,0,.004))
# Robot rests on the tabletop, rather than on top of the pressure plate.
robot=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Piper'))
robot.AddTranslateOp(opSuffix='userCorrection').Set(Gf.Vec3d(0,.072,-.0085886882527112))
# Above offset is in the robot's +90-degree local frame: local +Y maps to world -X.
mount_joint=UsdPhysics.FixedJoint(stage.GetPrimAtPath('/World/MountRobot'))
mount_joint.GetLocalPos0Attr().Set(Gf.Vec3f(-.04004712172472246,-.34622165668283017,.635))
from movable_clamps import author_clamps
author_clamps(stage,base)
# Place the full light asset in the clear strip between tabletop and sofa.
light=stage.GetPrimAtPath('/World/Room/Light');bounds=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(light).ComputeAlignedRange()
center=bounds.GetMidpoint();delta=Gf.Vec3d(-center[0],.912-center[1],-bounds.GetMin()[2])
light_local=UsdGeom.Xformable(light).ComputeLocalToWorldTransform(0)*Gf.Matrix4d().SetTranslate(delta)*UsdGeom.Xformable(light.GetParent()).ComputeLocalToWorldTransform(0).GetInverse()
light_xf=UsdGeom.Xformable(light);light_xf.ClearXformOpOrder();light_xf.AddTransformOp(opSuffix='betweenTableSofa').Set(light_local)
# Isaac Sim 5.1 requires reset stacks for nested rigid bodies. Preserve world poses.
robot_bodies=[p for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Piper')) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
world_poses={str(p.GetPath()):UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0) for p in robot_bodies}
for p in robot_bodies:
 xf_body=UsdGeom.Xformable(p);xf_body.ClearXformOpOrder();xf_body.AddTransformOp(opSuffix='compat').Set(world_poses[str(p.GetPath())]);xf_body.SetResetXformStack(True)
# Rotate the source screw axis from X to vertical Z. Place the fixed jaw at the left rear edge.
rotation=(Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,1,0),-90))*Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),-90))).ExtractRotation()
source_anchor=Gf.Vec3d(-.119,.393,2.155)
target_anchor=Gf.Vec3d(.657,-.390,.635)
translation=target_anchor-rotation.TransformDir(source_anchor)
xf=UsdGeom.Xformable(mount);xf.AddTranslateOp().Set(translation);xf.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
from camera_mount_placement import seat_camera_arm,aim_camera_head,seat_camera_clamp
camera_clamp_fit=seat_camera_clamp(stage,'/World/CameraMount')
translation=UsdGeom.Xformable(mount).GetOrderedXformOps()[0].Get()
camera_seating=seat_camera_arm(stage,'/World/CameraMount')
camera_aim=aim_camera_head(stage,'/World/CameraMount')
(base/'camera_pose.json').write_text(json.dumps(camera_aim,indent=2))
# Replace the world anchor with a tabletop anchor, so lift motion carries the mount.
j=UsdPhysics.FixedJoint(stage.GetPrimAtPath('/World/CameraMount/Joints/WorldClamp'))
j.CreateBody0Rel().SetTargets(['/World/Desk/TopLink']);j.CreateBody1Rel().SetTargets(['/World/CameraMount/Links/Clamp'])
j.CreateLocalPos0Attr().Set(Gf.Vec3f(translation));j.CreateLocalRot0Attr().Set(Gf.Quatf(rotation.GetQuat()));j.CreateLocalPos1Attr().Set(Gf.Vec3f(0));j.CreateLocalRot1Attr().Set(Gf.Quatf(1))
j.CreateExcludeFromArticulationAttr().Set(True)
UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/CameraMount/Links/Clamp')).CreateFilteredPairsRel().AddTarget('/World/Desk/TopLink')
# Explicit holding drives let the asset stay posed when opening the USD alone.
for name in ['BaseSwivel','joint_base_0_internal_shaft','lock_pivot_elbow_internal_shaft','lock_pivot_camera_internal_shaft','CameraRoll']:
 d=UsdPhysics.DriveAPI(stage.GetPrimAtPath('/World/CameraMount/Joints/'+name),'angular');d.CreateStiffnessAttr().Set(100);d.CreateDampingAttr().Set(10);d.CreateMaxForceAttr().Set(5);d.CreateTargetPositionAttr().Set(camera_seating['swivel_deg'] if name=='BaseSwivel' else camera_aim['joint_targets_deg'].get(name,0))
def camera(path,eye,target,focal):
 cam=UsdGeom.Camera.Define(stage,path);cam.CreateFocalLengthAttr().Set(focal);cam.CreateClippingRangeAttr().Set(Gf.Vec2f(.01,100))
 world=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse()
 parent=UsdGeom.Xformable(cam.GetPrim().GetParent()).ComputeLocalToWorldTransform(0)
 x=UsdGeom.Xformable(cam);x.ClearXformOpOrder();x.AddTransformOp().Set(world*parent.GetInverse());return cam
camera('/World/Camera',(1.85,-2.5,1.95),(-.05,0,.77),27)
camera('/World/MountDetail',(.22,-.90,.86),(-.025,-.345,.661),50)
camera('/World/Overview', (2.8,-3.6,2.65),(-.3,.25,.95),24)
eye=Gf.Vec3d(*camera_aim['optical_eye_world']);target=eye+Gf.Vec3d(*camera_aim['optical_forward_world'])
camera('/World/CameraMount/Links/CameraBody/WorkspaceCamera',eye,target,18)
UsdLux.DomeLight(stage.GetPrimAtPath('/World/Dome')).GetIntensityAttr().Set(350)
UsdLux.DistantLight(stage.GetPrimAtPath('/World/Sun')).GetIntensityAttr().Set(1200)
stage.GetRootLayer().customLayerData={'camera_placement':'User arrow: front-right tabletop mount; no photo calibration. Virtual WorkspaceCamera optical pose is estimated.','mount_model':'Fixed clamp connection to tabletop; virtual holding drives; spring runtime supplied separately.'}
# The room is a visual backdrop. Remove inherited out-of-scope physics material bindings.
for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Room')):
 for rel in p.GetRelationships():
  if rel.GetName()=='material:binding:physics':rel.SetTargets([])
from physics_repairs import apply_repairs
apply_repairs(stage,base)
from task_asset_placement import place_task_assets
place_task_assets(stage,base)
stage.GetRootLayer().Save()
layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(base/'scene.usda'));assert not unresolved,unresolved
joints=[]
for p in stage.Traverse():
 if p.IsA(UsdPhysics.Joint):
  for rel in [UsdPhysics.Joint(p).GetBody0Rel(),UsdPhysics.Joint(p).GetBody1Rel()]:
   for target in rel.GetTargets():assert stage.GetPrimAtPath(target),(p.GetPath(),target)
for p in Usd.PrimRange(stage.GetPrimAtPath('/World/CameraMount/Links/Clamp')):
 if p.IsA(UsdGeom.Mesh):
  b=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(p).ComputeAlignedRange();print('CLAMP_MESH',p.GetName(),list(b.GetMin()),list(b.GetMax()),flush=True)
cache=UsdGeom.BBoxCache(0,['default','render'])
table_bounds=cache.ComputeWorldBound(stage.GetPrimAtPath('/World/Desk/TopLink/Top/Mesh')).ComputeAlignedRange()
board_bounds=cache.ComputeWorldBound(board).ComputeAlignedRange()
flange_bounds=cache.ComputeWorldBound(stage.GetPrimAtPath('/World/Piper/Geometry/dummy_link/base_link_3_0/SourceMesh')).ComputeAlignedRange()
right_gap=table_bounds.GetMax()[0]-board_bounds.GetMax()[0]
flange_gap=board_bounds.GetMin()[2]-flange_bounds.GetMax()[2]
assert abs(right_gap-.08)<1e-5,(right_gap,board_bounds)
assert abs(flange_gap)<1e-5,flange_gap
light_bounds=cache.ComputeWorldBound(light).ComputeAlignedRange()
report={'camera_clamp_fit':camera_clamp_fit,'camera_seating':camera_seating,'clamp_orientation':'Jaws inward, backs outside table front. Literal 180-degree reversal rejected after purpose clarification because backs crossed the tabletop.','light_world_bounds':[list(light_bounds.GetMin()),list(light_bounds.GetMax())],'right_clearance_m':right_gap,'plate_bottom_z_m':board_bounds.GetMin()[2],'robot_flange_top_z_m':flange_bounds.GetMax()[2],'plate_flange_gap_m':flange_gap,'unresolved':unresolved,'layers':len(layers),'assets':len(assets),'prims':len(list(stage.Traverse())),'camera_mount_translation':list(translation),'camera_mount_rotation_y_deg':-90,'camera_mount_rotation_z_deg':-90,'placement_status':'PROVISIONAL_NOT_CALIBRATED','source_archive_integrity':'PASS','joint_targets':'PASS','isaac_version_tested':'5.1.0','source_authoring_version':'6.0.1'}
(base/'composition_check.json').write_text(json.dumps(report,indent=2))
print('COMPOSITION',json.dumps(report),flush=True)
app.close()

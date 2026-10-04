"""Choose a swivel pose with the clamp seated over the front tabletop edge."""
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,Gf

def seat_camera_arm(stage,root_path):
    root=stage.GetPrimAtPath(root_path)
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(root_path+'/Joints/BaseSwivel'))
    anchor=Gf.Vec3d(joint.GetLocalPos0Attr().Get())
    axis=Gf.Rotation(Gf.Quatd(joint.GetLocalRot0Attr().Get())).TransformDir(Gf.Vec3d(1,0,0))
    root_world=UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)
    bodies=[p for p in stage.GetPrimAtPath(root_path+'/Links').GetChildren() if p.GetName() not in ['Clamp','Pad']]
    points=[]
    for body in bodies:
        for p in Usd.PrimRange(body):
            if p.IsA(UsdGeom.Mesh):
                a=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(),dtype=float)
                mat=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)*root_world.GetInverse())
                points.append(a@mat[:3,:3]+mat[3,:3])
    pts=np.concatenate(points);trials=[];chosen=None
    for angle in [0,-30,30,-60,60,-90,90,-120,120,-150,150,175]:
        twist=Gf.Matrix4d().SetTranslate(-anchor)*Gf.Matrix4d().SetRotate(Gf.Rotation(axis,angle))*Gf.Matrix4d().SetTranslate(anchor)
        mat=np.array(twist*root_world);world=pts@mat[:3,:3]+mat[3,:3]
        # 1 mm clearance margin for arm members; fixture clamp and pad excluded.
        mask=(world[:,0]>-.701)&(world[:,0]<.701)&(world[:,1]>-.40965288)&(world[:,1]<.39234712)&(world[:,2]>.609)&(world[:,2]<.636)
        count=int(mask.sum());trials.append({'angle_deg':angle,'arm_vertices_in_table_clearance_box':count})
        if count==0:chosen=(angle,twist);break
    assert chosen is not None,trials
    angle,twist=chosen
    for body in bodies:
        x=UsdGeom.Xformable(body);local=x.GetLocalTransformation();x.ClearXformOpOrder();x.AddTransformOp(opSuffix='seatedSwivel').Set(local*twist)
    return {'swivel_deg':angle,'trials':trials,'scope':'All visible arm vertices versus expanded tabletop box; clamp fixture and pad handled separately; not full triangle collision certification.'}

def aim_camera_head(stage,root_path):
    """Aim the source optical-face normal using the existing neck and roll joints."""
    from scipy.optimize import least_squares
    body=stage.GetPrimAtPath(root_path+'/Links/CameraBody')
    neck=stage.GetPrimAtPath(root_path+'/Links/CameraNeck')
    body_world=UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
    front=[];back=[]
    for p in Usd.PrimRange(body):
        if not p.IsA(UsdGeom.Mesh):continue
        name=p.GetName()
        if '镜片' not in name and '后螺钉' not in name:continue
        pts=np.asarray(UsdGeom.Mesh(p).GetPointsAttr().Get(),dtype=float)
        mat=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0));pts=pts@mat[:3,:3]+mat[3,:3]
        (front if '镜片' in name else back).append(pts)
    front=np.concatenate(front);back=np.concatenate(back)
    center=front.mean(0);_,_,vh=np.linalg.svd(front-center,full_matrices=False);normal=vh[-1]
    if normal@(center-back.mean(0))<0:normal=-normal
    normal/=np.linalg.norm(normal)
    joints=[UsdPhysics.Joint(stage.GetPrimAtPath(root_path+'/Joints/'+n)) for n in ['lock_pivot_camera_internal_shaft','CameraRoll']]
    anchors=[];axes=[]
    for j in joints:
        mat=UsdGeom.Xformable(stage.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0])).ComputeLocalToWorldTransform(0)
        anchors.append(mat.Transform(Gf.Vec3d(j.GetLocalPos0Attr().Get())))
        axis=mat.TransformDir(Gf.Rotation(Gf.Quatd(j.GetLocalRot0Attr().Get())).TransformDir(Gf.Vec3d(1,0,0)))
        axes.append(np.array(axis.GetNormalized()))
    target=np.array([-.75,np.sqrt(3)/4,-.5])
    def rotate(v,a,t):
        t=np.deg2rad(t);return v*np.cos(t)+np.cross(a,v)*np.sin(t)+a*(a@v)*(1-np.cos(t))
    def residual(q):return rotate(rotate(normal,axes[1],q[1]),axes[0],q[0])-target
    solutions=[least_squares(residual,[a,b],bounds=(-179.9,179.9)) for a in [-90,0,90] for b in [-90,0,90]]
    solution=min(solutions,key=lambda r:(np.linalg.norm(r.fun)>1e-4,np.linalg.norm(r.x)))
    assert np.linalg.norm(solution.fun)<1e-4,solution.fun
    alpha,beta=map(float,solution.x)
    t1=Gf.Matrix4d().SetTranslate(-anchors[0])*Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*axes[0]),alpha))*Gf.Matrix4d().SetTranslate(anchors[0])
    anchor2=t1.Transform(anchors[1]);axis2=t1.TransformDir(Gf.Vec3d(*axes[1]))
    t2=Gf.Matrix4d().SetTranslate(-anchor2)*Gf.Matrix4d().SetRotate(Gf.Rotation(axis2,beta))*Gf.Matrix4d().SetTranslate(anchor2)
    for prim,twist in [(neck,t1),(body,t1*t2)]:
        x=UsdGeom.Xformable(prim);world=x.ComputeLocalToWorldTransform(0)*twist
        parent=UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(0)
        x.ClearXformOpOrder();x.AddTransformOp(opSuffix='cameraAim').Set(world*parent.GetInverse())
    eye=(t1*t2).Transform(Gf.Vec3d(*center))+Gf.Vec3d(*target)*.004
    return {'heading_to_table_edge_deg':30.,'downward_pitch_deg':30.,'optical_forward_world':target.tolist(),'optical_eye_world':list(eye),'joint_targets_deg':{'lock_pivot_camera_internal_shaft':alpha,'CameraRoll':beta},'direction_fit_error':float(np.linalg.norm(solution.fun)),'basis':'User requested 30-degree heading inward and 30-degree downward pitch. Optical normal estimated from supplied lens geometry; intrinsics and real extrinsics uncalibrated.'}

def seat_camera_clamp(stage,root_path):
    """Seat the fixed upper jaw and close the screw pad on the table underside."""
    root=UsdGeom.Xformable(stage.GetPrimAtPath(root_path))
    frame_mesh=next(p for p in Usd.PrimRange(stage.GetPrimAtPath(root_path+'/Links/Clamp')) if p.IsA(UsdGeom.Mesh) and '夹板' in p.GetName())
    pts=np.asarray(UsdGeom.Mesh(frame_mesh).GetPointsAttr().Get(),dtype=float)
    mat=np.array(UsdGeom.Xformable(frame_mesh).ComputeLocalToWorldTransform(0));world=pts@mat[:3,:3]+mat[3,:3]
    jaw=world[(world[:,1]>-.40865288)&(world[:,0]<.7)&(world[:,2]>.620)]
    assert len(jaw)>0
    raise_z=.635-float(jaw[:,2].min())
    translate=root.GetOrderedXformOps()[0];position=Gf.Vec3d(translate.Get());position[2]+=raise_z;translate.Set(position)
    pad=stage.GetPrimAtPath(root_path+'/Links/Pad')
    box=UsdGeom.BBoxCache(0,['default','render']).ComputeWorldBound(pad).ComputeAlignedRange()
    screw_delta=.610-float(box.GetMax()[2])
    local_delta=Gf.Vec3d(screw_delta,0,0)
    UsdGeom.Xformable(pad).AddTranslateOp(opSuffix='tableClamping').Set(local_delta)
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(root_path+'/Joints/PadSwingAndSpin'))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(Gf.Vec3d(joint.GetLocalPos0Attr().Get())+local_delta))
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path+'/Links/Clamp')):
        if p.IsA(UsdGeom.Mesh) and any(n in p.GetName() for n in ['螺杆','横杆']):UsdGeom.Xformable(p).AddTranslateOp(opSuffix='tableClamping').Set(local_delta)
    return {'upper_jaw_z_offset_m':raise_z,'screw_pad_advance_m':screw_delta,'jaw_table_overlap_depth_m':float(jaw[:,1].max()+.40865288),'pad_target_z_m':.610,'upper_jaw_target_z_m':.635,'scope':'Visible jaw and pad positioned against the 25 mm tabletop; fixed mounting joint remains the support model, not friction-load certification.'}

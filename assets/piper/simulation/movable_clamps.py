"""Install supplied screw-coupled clamps with an assembly-specific neutral opening."""
from pathlib import Path
import json
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,Gf

CLAMP_PATHS=['/World/Clamps/Left','/World/Clamps/Right']
PITCH_M=.003005

def author_clamps(stage,base):
    source='./PiPER_Scene_Assets_v1/assets/Clamp_Final_Black_v1/IsaacSim/Clamp.usdc'
    params=json.loads((Path(base)/'PiPER_Scene_Assets_v1/assets/Clamp_Final_Black_v1/IsaacSim/parameters.json').read_text())
    axis=Gf.Vec3d(*params['axis_direction'])
    # Upper fixed jaw is seated at plate top. Retract the screw to tabletop underside.
    root_z=.4922490882527112
    feed_shift=(.610-root_z-params['part_properties']['pad']['bounds'][1][2])/axis[2]
    shift=axis*feed_shift
    records=[]
    for prim in list(stage.GetPrimAtPath('/World/Desk/TopLink').GetChildren()):
        if prim.GetName().startswith('Clamp'):prim.SetActive(False)
    for i,path in enumerate(CLAMP_PATHS):
        root=UsdGeom.Xform.Define(stage,path)
        root.GetPrim().GetReferences().AddReference(source,'/Clamp')
        cx=-.04004712172472246+(-.055 if i==0 else .055)+.0050782621
        root.AddTranslateOp().Set(Gf.Vec3d(cx,-.394,root_z))
        root.AddRotateZOp().Set(-90.)
        # Shift the joint neutral frame and its entire downstream chain together.
        for name in ['carriage','screw','pad','handle']:
            body=UsdGeom.Xformable(stage.GetPrimAtPath(path+'/'+name))
            body.AddTranslateOp(opSuffix='installationOpening').Set(shift)
        feed=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(path+'/Joints/ScrewFeed'))
        feed.GetLocalPos0Attr().Set(Gf.Vec3f(Gf.Vec3d(feed.GetLocalPos0Attr().Get())+shift))
        feed.GetLowerLimitAttr().Set(-.00601);feed.GetUpperLimitAttr().Set(.00075125)
        spin=UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(path+'/Joints/ScrewTurn'))
        spin.GetLowerLimitAttr().Set(-720.);spin.GetUpperLimitAttr().Set(90.)
        drive=UsdPhysics.DriveAPI(spin.GetPrim(),'angular');drive.GetTargetPositionAttr().Set(0);drive.GetMaxForceAttr().Set(.03)
        # The frame is an explicit fixture; pad/table contacts remain enabled.
        frame=stage.GetPrimAtPath(path+'/frame')
        world=UsdGeom.Xformable(frame).ComputeLocalToWorldTransform(0)
        fix=UsdPhysics.FixedJoint.Define(stage,path+'/Joints/TableAttachment')
        fix.CreateBody0Rel().SetTargets(['/World/Desk/TopLink']);fix.CreateBody1Rel().SetTargets([frame.GetPath()])
        fix.CreateLocalPos0Attr().Set(Gf.Vec3f(world.ExtractTranslation()));fix.CreateLocalRot0Attr().Set(Gf.Quatf(world.ExtractRotationQuat()))
        fix.CreateLocalPos1Attr().Set(Gf.Vec3f(0));fix.CreateLocalRot1Attr().Set(Gf.Quatf(1));fix.CreateExcludeFromArticulationAttr().Set(True)
        UsdPhysics.FilteredPairsAPI.Apply(frame).CreateFilteredPairsRel().AddTarget('/World/Desk/TopLink')
        for name in ['frame','screw','pad','handle']:
            mesh=stage.GetPrimAtPath(path+'/'+name+'/Mesh')
            api=PhysxSchema.PhysxCollisionAPI.Apply(mesh);api.CreateRestOffsetAttr().Set(0.);api.CreateContactOffsetAttr().Set(.0003)
        cache=UsdGeom.BBoxCache(0,['default','render'])
        pad_box=cache.ComputeWorldBound(stage.GetPrimAtPath(path+'/pad')).ComputeAlignedRange()
        points=np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath(path+'/frame/Mesh')).GetPointsAttr().Get(),dtype=float)
        matrix=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(path+'/frame/Mesh')).ComputeLocalToWorldTransform(0))
        worldpoints=points@matrix[:3,:3]+matrix[3,:3]
        forbidden=(worldpoints[:,2]>.61001)&(worldpoints[:,2]<.63499)&(worldpoints[:,1]>-.40864288)&(worldpoints[:,1]<.39133712)&(worldpoints[:,0]>-.69999)&(worldpoints[:,0]<.69999)
        assert not forbidden.any(),int(forbidden.sum())
        records.append({'path':path,'neutral_feed_offset_m':feed_shift,'pad_top_z_m':float(pad_box.GetMax()[2]),'table_bottom_z_m':.610,'pad_gap_m':.610-float(pad_box.GetMax()[2]),'frame_vertices_inside_table_slab':int(forbidden.sum()),'screw_pitch_m_per_turn':PITCH_M,'turn_range':[-2,.25],'fixture':'Frame fixed to tabletop; screw, pad and handle are dynamic; pad-table contacts enabled. Board and robot remain fixed installation; friction holding capacity not validated.'})
    mass=UsdPhysics.MassAPI(stage.GetPrimAtPath('/World/Desk/TopLink')).GetMassAttr()
    mass.Set(float(mass.Get())-2*sum(params['part_properties'][x]['mass'] for x in ['frame','screw','pad','handle']))
    (Path(base)/'clamp_assembly_check.json').write_text(json.dumps({'scope':'Static sampled visible geometry plus joint definition; dynamic check separate','clamps':records},indent=2))
    return records

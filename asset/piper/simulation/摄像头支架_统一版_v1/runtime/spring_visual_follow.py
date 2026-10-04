"""Two-anchor visual deformation for the scan-constrained spring candidates.

This updates appearance only. It does not claim measured elastic forces.
Pass the original, unmodified stage to construct the updater, then call update
after physics steps. Original vertex arrays are retained, preventing drift.
"""
import numpy as np
from pxr import Usd,UsdGeom,Vt

class SpringVisualFollow:
    def __init__(self,stage,anchor_data):
        self.stage=stage;self.rows=[]
        for spec in anchor_data:
            mesh=UsdGeom.Mesh(stage.GetPrimAtPath(spec['mesh']))
            points=np.asarray(mesh.GetPointsAttr().Get(),dtype=np.float64)
            a=np.asarray(spec['anchor_start_m']);b=np.asarray(spec['anchor_end_m'])
            direction=b-a;length=np.linalg.norm(direction);direction/=length
            # Preserve the hook ends on their respective rigid attachments.
            t=np.clip(((points-a)@direction-.010)/max(length-.020,.001),0,1)
            indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get());faces=[];offset=0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                polygon=indices[offset:offset+count];offset+=count
                faces.extend((polygon[0],polygon[j],polygon[j+1]) for j in range(1,count-1))
            self.rows.append(dict(spec,original=points,weight=t[:,None],mesh_api=mesh,faces=np.array(faces),original_normals=mesh.GetNormalsAttr().Get(),original_normals_interpolation=mesh.GetNormalsInterpolation()))

    def update(self):
        results=[]
        for row in self.rows:
            points=row['original'];t=row['weight']
            matrices=[np.array(UsdGeom.Xformable(self.stage.GetPrimAtPath(row[k])).ComputeLocalToWorldTransform(Usd.TimeCode.Default())) for k in ['body_start','body_end','mesh']]
            homogeneous=np.column_stack([points,np.ones(len(points))])
            world=(1-t)*(homogeneous@matrices[0])[:,:3]+t*(homogeneous@matrices[1])[:,:3]
            local=(np.column_stack([world,np.ones(len(world))])@np.linalg.inv(matrices[2]))[:,:3]
            row['mesh_api'].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(local.astype(np.float32)))
            row['mesh_api'].GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([local.min(0),local.max(0)],dtype=np.float32)))
            faces=row['faces'];tri=local[faces];face_normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals=np.zeros_like(local)
            for column in range(3):np.add.at(normals,faces[:,column],face_normals)
            normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-20)
            row['mesh_api'].SetNormalsInterpolation('vertex');row['mesh_api'].GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
            results.append({'mesh':row['mesh'],'finite':bool(np.isfinite(local).all()),'vertices':len(local)})
        return results

    def restore(self):
        for row in self.rows:
            points=row['original'];row['mesh_api'].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
            row['mesh_api'].GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(np.array([points.min(0),points.max(0)],dtype=np.float32)))
            row['mesh_api'].SetNormalsInterpolation(row['original_normals_interpolation'])
            if row['original_normals'] is not None:row['mesh_api'].GetNormalsAttr().Set(row['original_normals'])

"""Save a NEW mapping-only stage with central rooms and reception excluded.

Run with a Python environment containing pxr (e.g. Blender bundled Python).
Never saves source layers, never flattens external references.
"""
from pathlib import Path
import hashlib
import json
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'isaacpjt/assets'
SOURCE=ASSETS/'hospital_integration_human.usd'
DEST=ASSETS/'hospital_integration_human_map_excluded.usd'
REPORT=Path(__file__).parent/'exclusion_report.json'

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not DEST.exists(), f'Refusing to overwrite {DEST}'
    source_hash=sha(SOURCE)
    source_layer=Sdf.Layer.FindOrOpen(str(SOURCE))
    source_layer.Export(str(DEST))
    s=Usd.Stage.Open(str(DEST),load=Usd.Stage.LoadNone)
    bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'])
    room_names=['Geo_M2_Floor8','Geo_M2_Floor7','Geo_M2_Floor6','Geo_M2_Floor5',
                'Geo_M2_Floor4','Geo_M2_Floor3','Geo_M2_Floor2','Geo_M2_Floor_582']
    floors=[s.GetPrimAtPath('/World/hospital/'+name) for name in room_names]
    assert all(floors)
    rr=Gf.Range3d()
    for p in floors: rr.UnionWith(bbox.ComputeWorldBound(p).ComputeAlignedRange())
    desk=s.GetPrimAtPath('/World/hospital/ReceptionDesk')
    assert desk and desk.IsActive()
    dr=bbox.ComputeWorldBound(desk).ComputeAlignedRange()
    # Extents hug the existing walls. Desk gets only 5 cm safety padding.
    rooms=[rr.GetMin()[0],rr.GetMin()[1],rr.GetMax()[0],rr.GetMax()[1]]
    reception=[dr.GetMin()[0]-.05,dr.GetMin()[1]-.05,dr.GetMax()[0]+.05,dr.GetMax()[1]+.05]
    desk.SetActive(False)
    # Leave floor tiles and all other visible room geometry untouched.
    root=UsdGeom.Xform.Define(s,'/World/OccupancyMapExclusions')
    root.AddTransformOp().Set(UsdGeom.XformCache().GetLocalToWorldTransform(s.GetPrimAtPath('/World')).GetInverse())
    root.GetPrim().SetCustomDataByKey('mappingOnly',True)
    root.GetPrim().SetDocumentation('Mapping-only invisible collision curtains. Keep PhysX geometry ON. Do not use this copy as the live robot simulation stage.')
    root.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    regions=[]
    for name,rect in [('CentralRooms',rooms),('ReceptionDesk',reception)]:
        x0,y0,x1,y1=rect
        group=UsdGeom.Xform.Define(s,root.GetPath().AppendChild(name))
        group.GetPrim().CreateAttribute('mapping:unknownBoundsXY',Sdf.ValueTypeNames.Double4,custom=True).Set(Gf.Vec4d(*rect))
        group.GetPrim().CreateAttribute('mapping:unknownPixel',Sdf.ValueTypeNames.Int,custom=True).Set(127)
        # Hollow boundary, not solid box: a filled box would be black occupied.
        thickness=.10
        walls=[('South',((x0+x1)/2,y0+thickness/2,1.5),(x1-x0,thickness,3.0)),
               ('North',((x0+x1)/2,y1-thickness/2,1.5),(x1-x0,thickness,3.0)),
               ('West',(x0+thickness/2,(y0+y1)/2,1.5),(thickness,y1-y0,3.0)),
               ('East',(x1-thickness/2,(y0+y1)/2,1.5),(thickness,y1-y0,3.0))]
        for wall,pos,size in walls:
            cube=UsdGeom.Cube.Define(s,group.GetPath().AppendChild(wall))
            cube.CreateSizeAttr(1)
            cube.AddTranslateOp().Set(pos)
            cube.AddScaleOp().Set(size)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
        regions.append({'name':name,'bounds_xy':rect})
    s.GetRootLayer().Save()
    assert sha(SOURCE)==source_hash
    # A standalone static-collision extract lets us validate without downloading
    # character, robot, material or lighting assets and without touching the UI.
    proxy=Usd.Stage.CreateNew(str(Path(__file__).parent/'static_collision_validation.usdc'))
    UsdGeom.SetStageMetersPerUnit(proxy,1)
    UsdGeom.SetStageUpAxis(proxy,UsdGeom.Tokens.z)
    world=UsdGeom.Xform.Define(proxy,'/World')
    proxy.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(proxy,'/World/PhysicsScene')
    xf=UsdGeom.XformCache()
    copied=[]
    excluded_roots=['/World/robot_nova','/World/Characters','/World/tray','/World/S_TrafficCone']
    for p in Usd.PrimRange(s.GetPrimAtPath('/World'),Usd.TraverseInstanceProxies()):
        path=str(p.GetPath())
        if any(path.startswith(prefix) for prefix in excluded_roots):continue
        if not p.IsA(UsdGeom.Gprim) or not p.HasAPI(UsdPhysics.CollisionAPI):continue
        if UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
        q=proxy.DefinePrim('/World/Collider_%04d'%len(copied),p.GetTypeName())
        for attr in p.GetAttributes():
            n=attr.GetName()
            if n.startswith(('xformOp','visibility','primvars:','material:')):continue
            value=attr.Get()
            if value is not None:
                q.CreateAttribute(n,attr.GetTypeName(),custom=attr.IsCustom()).Set(value)
        UsdGeom.Xformable(q).AddTransformOp().Set(xf.GetLocalToWorldTransform(p))
        UsdPhysics.CollisionAPI.Apply(q)
        if p.IsA(UsdGeom.Mesh):
            approx=UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() or 'none'
            UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr(approx)
        q.SetCustomDataByKey('sourcePrim',path)
        copied.append(path)
    proxy.GetRootLayer().Save()
    report={'source':str(SOURCE),'source_sha256':source_hash,'output':str(DEST),
            'regions':regions,'disabled_prims':[str(desk.GetPath())],
            'floor_prims_unchanged':[str(p.GetPath()) for p in floors],
            'collision_validation_prims':len(copied),
            'validation_omissions':excluded_roots,
            'panel':{'origin':[0,0,0],'lower':[-49.50,-9.10,.05],
                     'upper':[24.95,23.15,1.0],'cell_size':.05,'physx_geometry':True}}
    REPORT.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()

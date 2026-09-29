"""Local regression: occupancy unknown depends on reachability, not floor."""
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':64,'height':64,'disable_viewport_updates':True})
import omni.usd, omni.timeline, omni.physx
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.asset.gen.omap')
from isaacsim.asset.gen.omap.bindings import _omap
from pxr import UsdGeom, UsdPhysics
ctx=omni.usd.get_context()
s=ctx.get_stage()
assert s
UsdGeom.SetStageMetersPerUnit(s,1)
UsdPhysics.Scene.Define(s,'/physics')
for name,pos,scale in [('a',(1,0,1),(.1,2,2)),('b',(3,0,1),(.1,2,2)),('c',(2,1,1),(2.1,.1,2)),('d',(2,-1,1),(2.1,.1,2))]:
    g=UsdGeom.Cube.Define(s,'/'+name)
    g.CreateSizeAttr(1)
    g.AddTranslateOp().Set(pos)
    g.AddScaleOp().Set(scale)
    UsdPhysics.CollisionAPI.Apply(g.GetPrim())
for _ in range(5):app.update()
print('SCENE_READY',flush=True)
tl=omni.timeline.get_timeline_interface()
tl.play()
for _ in range(3):app.update()
g=_omap.Generator(omni.physx.get_physx_interface(),ctx.get_stage_id())
g.update_settings(.05,1,0,-1)
g.set_transform((0,0,.5),(-4,-3,0),(4,3,0))
g.generate2d()
from collections import Counter
print('UNKNOWN_TEST',g.get_dimensions(),Counter(g.get_buffer()),flush=True)
print('BOUNDS',g.get_min_bound(),g.get_max_bound(),flush=True)
tl.stop()
app.close()

"""Reference-inspired specimen collection workstation. Blender 5.2, metres.
No third-party assets, textures, medical functionality or physics.
Run: blender --background --factory-startup --python this_file.py
"""
from pathlib import Path
import math
import json
import bpy
from mathutils import Vector
from pxr import Usd, UsdGeom, UsdShade, UsdUtils

OUT = Path(__file__).resolve().parent
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1
bpy.context.preferences.filepaths.save_version = 0
asset = bpy.data.collections.get('Collection')
asset.name = 'SpecimenCollectionWorkstation'
studio = bpy.data.collections.new('PreviewStudio_NOT_EXPORTED')
scene.collection.children.link(studio)

def move(obj, coll=asset):
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    coll.objects.link(obj)

def group(name, parent=None, loc=(0,0,0)):
    o = bpy.data.objects.new(name, None)
    asset.objects.link(o)
    o.parent = parent
    o.location = loc
    o.empty_display_size = .08
    return o

root = group('SpecimenCollectionWorkstation')
root['description'] = 'Three-bay specimen collection room visual prop'
root['dimensions'] = 'Estimated from reference photograph, not manufacturer specifications'
root['front'] = '-Y; floor at Z=0; metres'
root['physics'] = 'None; visual only'

def mat(name, color, metal=0, rough=.4, alpha=1, emit=0):
    m = bpy.data.materials.new(name)
    m.diffuse_color = (*color,alpha)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*color,1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = rough
    p.inputs['Alpha'].default_value = alpha
    if emit:
        p.inputs['Emission Color'].default_value = (*color,1)
        p.inputs['Emission Strength'].default_value = emit
    return m

white = mat('WarmWhitePowderCoat',(.77,.80,.80),.12,.32)
porcelain = mat('WorktopPorcelain',(.90,.92,.91),.05,.28)
grey = mat('InnerCabinetGrey',(.30,.34,.36),.2,.42)
silver = mat('BrushedAluminium',(.52,.59,.62),.8,.29)
graphite = mat('GraphiteHardware',(.035,.045,.052),.25,.38)
rubber = mat('RubberWheelsAndBelt',(.012,.017,.019),0,.67)
teal = mat('InterfaceTeal',(.045,.37,.38),.05,.4,emit=.12)
blue = mat('TubeCodeBlue',(.05,.35,.57))
red = mat('TubeCodeRed',(.63,.09,.075))
purple = mat('TubeCodeLavender',(.42,.22,.52))
green = mat('ReadyLED',(.1,.65,.32),emit=.3)
glass = mat('ClearDividerAcrylic',(.79,.86,.87),.02,.2,alpha=.18)
screen = mat('ScreenSoftWhite',(.70,.79,.80),0,.4,emit=.25)
line = mat('ScreenGreyText',(.19,.28,.32),0,.5)
orange = mat('InterfaceOrange',(.8,.28,.1))

def finish(o,name,material,parent,loc):
    o.name = name
    move(o)
    o.parent = parent
    o.location = loc
    o.data.materials.append(material)
    return o

def box(name,loc,size,material,parent,r=.002):
    bpy.ops.mesh.primitive_cube_add(size=1)
    o = bpy.context.object
    o.dimensions = size
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    finish(o,name,material,parent,loc)
    if r:
        m = o.modifiers.new('SoftManufacturedEdges','BEVEL')
        m.width = min(r,min(size)*.4)
        m.segments = 3
        o.modifiers.new('WeightedNormals','WEIGHTED_NORMAL')
    return o

def cyl(name,loc,r,depth,material,parent,axis='Z',vertices=20):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices,radius=r,depth=depth)
    o = finish(bpy.context.object,name,material,parent,loc)
    if axis=='Y': o.rotation_euler.x = math.pi/2
    if axis=='X': o.rotation_euler.y = math.pi/2
    for f in o.data.polygons:
        f.use_smooth = len(f.vertices)==4
    return o

def bar(name,a,b,r,material,parent):
    a,b = Vector(a),Vector(b)
    o = cyl(name,(a+b)/2,r,(b-a).length,material,parent)
    o.rotation_euler = (b-a).to_track_quat('Z','Y').to_euler()
    return o

def label(name,body,loc,size,parent,material=graphite):
    data=bpy.data.curves.new(name,'FONT')
    data.body=body
    data.size=size
    data.align_x='CENTER'
    data.align_y='CENTER'
    data.resolution_u=2
    o=bpy.data.objects.new(name,data)
    asset.objects.link(o)
    o.parent=parent
    o.location=loc
    o.rotation_euler=(math.pi/2,0,0)
    data.materials.append(material)
    return o

# A thin-walled supply tower with eight coded drawers and four upper doors.
cab = group('01_SupplyCabinet',root,(-2.73,0,0))
box('BottomChassis',(0,0,.15),(1.22,.73,.07),grey,cab)
box('RearSkin',(0,.351,1.01),(1.22,.023,1.67),white,cab)
for x in [-.6,.6]:
    box('SideSkin',(x,0,1.01),(.025,.73,1.68),white,cab)
box('CabinetTop',(0,0,1.856),(1.23,.75,.035),porcelain,cab)
box('DarkDrawerReveal',(0,-.34,.85),(1.18,.018,1.31),graphite,cab)
for row in range(4):
    z=.335+row*.284
    for col in [-1,1]:
        x=col*.295
        box('DrawerFront',(x,-.373,z),(.572,.036,.268),white,cab,.007)
        box('RecessedFingerPull',(x+.13,-.394,z+.107),(.132,.009,.030),graphite,cab,.005)
        box('DrawerLabelHolder',(x-.16,-.395,z+.106),(.125,.008,.024),silver,cab)
        box('TubeColourCode',(x-.205,-.401,z+.106),(.027,.006,.019),[purple,purple,red,blue][row],cab,.001)
        box('DrawerIDPaper',(x-.148,-.401,z+.106),(.064,.003,.013),porcelain,cab,0)
        label('DrawerNumber',f'{row+1}{"A" if col<0 else "B"}',(x-.149,-.404,z+.106),.009,cab)
for j in range(4):
    x=(j-1.5)*.295
    box('UpperDoor',(x,-.374,1.613),(.282,.038,.439),white,cab,.006)
    if j%2:
        cyl('RoundLatch',(x+.07,-.397,1.758),.021,.009,silver,cab,'Y')
        cyl('LatchCentre',(x+.07,-.403,1.758),.016,.004,white,cab,'Y')
for x in [-.49,.49]:
    for y in [-.24,.24]:
        cyl('CasterWheel',(x,y,.044),.044,.034,rubber,cab,'X')
        box('CasterFork',(x,y,.095),(.05,.044,.047),silver,cab)
        cyl('LevellingFoot',(x+.07,y,.014),.032,.027,rubber,cab)
for j in range(9):
    box('SideVent',(-.614,.08+j*.021,1.62),(.003,.010,.117),graphite,cab,.001)

# Slim automated dispensing terminal alongside the supply drawers.
tower=group('02_DispensingTerminal',root,(-1.91,0,0))
box('TowerFoot',(0,0,.088),(.39,.68,.075),graphite,tower)
box('LowerTowerShell',(0,0,.38),(.39,.69,.52),white,tower,.009)
box('UpperTowerShell',(0,0,1.243),(.39,.69,1.08),white,tower,.009)
box('BlackDispenseSlot',(0,-.354,.680),(.31,.017,.083),graphite,tower)
box('DispenseTray',(0,-.382,.658),(.32,.13,.018),silver,tower)
box('LowerServiceHandle',(-.12,-.351,.505),(.04,.02,.126),graphite,tower,.01)
console=group('TiltedTouchConsole',tower,(0,-.29,1.797))
console.rotation_euler.x=math.radians(-24)
box('ConsoleFrame',(0,0,0),(.36,.045,.17),white,console,.012)
box('ConsoleScreen',(-.035,-.025,.004),(.253,.004,.121),screen,console)
for n in range(3): box('ConsoleKey',(.135,-.026,.04-n*.039),(.033,.004,.025),grey,console)
for n in range(3): box('TouchRow',(-.045,-.028,.039-n*.031),(.196,.002,.005),line,console,0)
# Large supply status display, looking toward the operator.
bar('StatusMonitorStand',(0,.15,1.80),(0,.15,2.03),.031,silver,tower)
box('StatusDisplayBody',(-.29,.13,2.235),(.86,.073,.51),graphite,tower,.016)
box('StatusDisplayGlass',(-.29,.090,2.24),(.815,.008,.452),rubber,tower,.002)
cyl('MonitorPowerLED',(.067,.082,2.011),.004,.003,green,tower,'Y')

# Long continuous worktop and open knee space, split into three stations.
bench=group('03_ContinuousBench',root)
box('Worktop',(.85,0,1.035),(5.12,.88,.039),porcelain,bench,.009)
box('RearApron',(.85,.399,.582),(5.03,.04,.83),grey,bench)
box('RearTopRail',(.85,.36,.969),(5.02,.09,.087),silver,bench)
box('LowerRearRail',(.85,.355,.18),(5.02,.075,.065),silver,bench)
for x in [-1.665,-.085,1.58,3.355]:
    box('BenchUpright',(x,.10,.585),(.047,.59,.86),white,bench,.005)
    for y in [-.12,.31]:
        cyl('AdjustableBenchFoot',(x,y,.065),.035,.13,rubber,bench)
for j in range(3):
    x=-.86+j*1.65
    box('UnderCounterPanel',(x,-.411,.943),(1.60,.038,.139),white,bench,.005)
    box('DrawerLatch',(x-.49,-.434,.943),(.028,.009,.048),graphite,bench)
    box('RearServicePanel',(x,.374,.595),(1.55,.012,.704),grey,bench)
    for dx in [-.66,.66]:
        box('RearPanelHinge',(x+dx,.362,.78),(.025,.012,.039),graphite,bench)
# Conveyance track lies behind clear work area and passes through all stations.
box('TrackHousing',(.85,.155,1.132),(5.02,.30,.152),white,bench,.006)
box('TrackFrontDarkBand',(.85,-.004,1.136),(5.0,.009,.078),graphite,bench,.002)
box('ContinuousBelt',(.85,.156,1.212),(4.99,.19,.008),rubber,bench,.001)
for y in [.04,.272]: box('TrackGuideRail',(.85,y,1.224),(5.04,.023,.023),silver,bench)
for j in range(11):
    box('ConveyorBeltJoint',(-1.51+j*.465,.156,1.217),(.003,.18,.002),grey,bench,0)

def screen_ui(parent):
    """Mesh-only example interface; contains no patient data or external image."""
    box('LCDGlass',(0,-.025,0),(.430,.006,.255),screen,parent,.002)
    box('InterfaceHeader',(0,-.029,.114),(.427,.002,.022),teal,parent,0)
    box('InterfaceSidebar',(-.191,-.030,-.006),(.041,.002,.216),graphite,parent,0)
    label('UIHeader','COLLECTION  /  READY',(-.026,-.032,.114),.008,parent,porcelain)
    for r in range(7):
        z=.077-r*.025
        box('MenuMark',(-.191,-.032,z),(.020,.001,.003),screen,parent,0)
        for c in range(3):
            box('ExampleTableCell',(-.122+c*.086,-.032,z),(.051 if c!=1 else .033,.001,.0035),line,parent,0)
    for r,m in enumerate([orange,teal,grey]):
        box('UIStatusBlock',(.162,-.032,.042-r*.052),(.071,.002,.040),m,parent,0)
    label('DemoLabel','DEMO',(.162,-.034,.042),.009,parent,porcelain)

for j,x in enumerate([-.46,1.18,2.82],1):
    station=group(f'04_Station_{j:02}',root,(x,0,0))
    # White fin behind terminal; transparent return panel shows depth.
    box('StationVerticalHousing',(.203,.188,1.467),(.103,.52,.60),white,station,.021)
    box('TopHousingLip',(.02,.405,1.75),(.44,.045,.034),white,station,.009)
    box('AcrylicSideDivider',(.271,.17,1.473),(.009,.50,.53),glass,station,.004)
    box('AcrylicBackDivider',(.02,.416,1.473),(.49,.006,.53),glass,station,.002)
    box('LowerIntakeFrame',(0,-.006,1.142),(.294,.025,.149),silver,station,.008)
    box('DarkIntakeOpening',(0,-.023,1.14),(.240,.013,.092),rubber,station,.006)
    box('IntakeBelt',(0,-.061,1.099),(.230,.141,.007),rubber,station)
    box('IntakeGreenStrip',(0,-.034,1.118),(.16,.005,.005),green,station,0)
    # Open laptop-like terminal supported by an angled shelf.
    laptop=group('OperatorTerminal',station,(-.087,-.125,1.41))
    box('TerminalBase',(0,-.125,0),(.482,.302,.022),silver,laptop,.007)
    box('KeyboardBed',(0,-.080,.013),(.438,.172,.004),graphite,laptop,.002)
    for row in range(5):
        for col in range(13):
            box('KeyboardKey',((col-6)*.032,-.017-row*.031,.017),(.027,.023,.004),grey,laptop,.001)
    box('Touchpad',(0,-.231,.013),(.122,.063,.003),grey,laptop,.002)
    lid=group('Display',laptop,(0,.017,.164))
    lid.rotation_euler.x=math.radians(-10)
    box('DisplayBezel',(0,0,0),(.482,.044,.303),graphite,lid,.009)
    screen_ui(lid)
    box('TerminalSupport',(.026,-.11,-.045),(.16,.22,.061),white,laptop,.005)
    # Tall polished pole and articulated overhead monitor, rear visible as in photo.
    cyl('MonitorPole',(.155,.22,1.936),.019,1.51,silver,station)
    cyl('PoleBaseCollar',(.155,.22,1.202),.043,.029,grey,station)
    cyl('ArmClamp',(.155,.22,2.646),.030,.07,silver,station)
    bar('MonitorArm',(.155,.22,2.66),(-.058,.27,2.75),.023,silver,station)
    bar('MonitorElbow',(-.058,.27,2.75),(-.125,.34,2.75),.023,silver,station)
    box('OverheadDisplayRear',(-.18,.373,2.775),(.48,.050,.322),graphite,station,.016)
    box('OverheadDisplayScreen',(-.18,.401,2.775),(.446,.005,.281),rubber,station,.002)
    cyl('VESAMount',(-.18,.337,2.775),.056,.023,grey,station,'Y')
    for sign in [-1,1]:
        bar('VESADiagonal',(-.18-sign*.063,.328,2.775-.063),(-.18+sign*.063,.328,2.775+.063),.009,graphite,station)
    for k in range(8): box('MonitorRearVent',(-.32+k*.024,.346,2.866),(.013,.003,.006),rubber,station,0)
    label('StationNumber',f'{j:02}',(.176,-.078,1.657),.033,station,teal)
    # Freestanding two-shelf trolley in front, with an open front and thin skins.
    cart=group(f'05_Trolley_{j:02}',root,(x+.31,-.80,0))
    for xx in [-.263,.263]:
        box('RoundedTrolleySide',(xx,0,.414),(.030,.44,.736),white,cart,.011)
    box('TrolleyBack',(0,.21,.401),(.50,.017,.645),grey,cart,.004)
    for z in [.077,.351,.661]:
        box('OpenShelf',(0,-.003,z),(.513,.424,.019),grey,cart,.005)
        box('ShelfFrontEdge',(0,-.215,z),(.512,.016,.030),graphite,cart,.003)
    box('TrolleyTopTray',(0,0,.728),(.512,.40,.020),porcelain,cart,.006)
    for xx in [-.224,.224]:
        for yy in [-.151,.151]:
            cyl('CartWheel',(xx,yy,.027),.027,.028,rubber,cart,'X')
            box('WheelFork',(xx,yy,.053),(.030,.028,.023),silver,cart)
    bar('TrolleyRearHandle',(-.20,.196,.767),(.20,.196,.767),.009,silver,cart)

print('GEOMETRY_BUILT',len(asset.objects),flush=True)
# Portable export: text and all edge modifiers are baked into geometry.
bpy.ops.object.select_all(action='DESELECT')
for o in list(asset.objects):
    bpy.context.view_layer.objects.active=o
    if o.type=='FONT':
        o.select_set(True)
        bpy.ops.object.convert(target='MESH')
        o.select_set(False)
    elif o.type=='MESH':
        for m in list(o.modifiers): bpy.ops.object.modifier_apply(modifier=m.name)
for o in asset.objects: o.select_set(True)
bpy.context.view_layer.objects.active=root
usd=OUT/'specimen_collection_workstation.usd'
bpy.ops.wm.usd_export(filepath=str(usd),selected_objects_only=True,export_materials=True,
    generate_preview_surface=True,export_textures_mode='KEEP',export_animation=False,
    export_lights=False,export_cameras=False,convert_world_material=False,
    root_prim_path='/CollectionWorkstation',convert_scene_units='METERS',meters_per_unit=1)
s=Usd.Stage.Open(str(usd))
s.SetDefaultPrim(s.GetPrimAtPath('/CollectionWorkstation'))
UsdGeom.SetStageMetersPerUnit(s,1)
UsdGeom.SetStageUpAxis(s,UsdGeom.Tokens.z)
for p in s.Traverse():
    if p.IsA(UsdShade.Shader) and 'ClearDividerAcrylic' in str(p.GetPath()):
        UsdShade.Shader(p).GetInput('opacity').Set(.18)
s.GetRootLayer().Save()
meshes=[UsdGeom.Mesh(p) for p in s.Traverse() if p.IsA(UsdGeom.Mesh)]
bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]).ComputeWorldBound(s.GetDefaultPrim()).ComputeAlignedRange()
_,deps,missing=UsdUtils.ComputeAllDependencies(str(usd))
assert not deps and not missing
stats={'default_prim':'/CollectionWorkstation','units':'metres','up_axis':'Z','front':'-Y',
       'size_m':list(bounds.GetSize()),'bounds_min':list(bounds.GetMin()),'bounds_max':list(bounds.GetMax()),
       'workstations':3,'trolleys':3,'mesh_count':len(meshes),
       'triangles_equivalent':sum(sum(n-2 for n in m.GetFaceVertexCountsAttr().Get()) for m in meshes),
       'external_assets':list(deps),'unresolved_assets':list(missing),'physics':False}
(OUT/'model_info.json').write_text(json.dumps(stats,indent=2)+'\n')
print('USD_VALIDATED',json.dumps(stats),flush=True)

# A separate presentation studio excluded from the USD.
floor=box('PreviewFloor',(0,0,-.037),(200,200,.06),mat('StudioFloor',(.55,.59,.61),rough=.8),None,0)
move(floor,studio)
scene.world.use_nodes=True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value=(.66,.73,.80,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value=.35
def aim(o,target): o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler()
for name,loc,power,size,sy in [('FrontSoftbox',(-3,-5,7),1800,7,4),('RightFill',(5,-2,5),1200,4,4),('BackSoftbox',(0,3,6),1600,6,3)]:
    data=bpy.data.lights.new(name,'AREA')
    data.energy=power
    data.shape='RECTANGLE'
    data.size=size
    data.size_y=sy
    o=bpy.data.objects.new(name,data)
    studio.objects.link(o)
    o.location=loc
    aim(o,(0,0,1))
data=bpy.data.cameras.new('HeroCamera')
camera=bpy.data.objects.new('HeroCamera',data)
studio.objects.link(camera)
camera.location=(-4.3,-11,4.1)
aim(camera,(0,-.10,1.34))
data.type='ORTHO'
data.ortho_scale=8.0
scene.camera=camera
scene.render.engine='CYCLES'
scene.cycles.samples=64
scene.cycles.use_denoising=True
scene.cycles.transparent_max_bounces=12
try:
    prefs=bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type='OPTIX'
    prefs.get_devices()
    if any(d.type=='OPTIX' for d in prefs.devices):
        for d in prefs.devices: d.use=d.type=='OPTIX'
        scene.cycles.device='GPU'
except Exception as exc: print('CPU render fallback',exc)
scene.render.resolution_x=2400
scene.render.resolution_y=1400
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.view_settings.view_transform='AgX'
bpy.ops.object.select_all(action='DESELECT')
root.select_set(True)
bpy.context.view_layer.objects.active=root
for ui in bpy.data.screens:
    for area in ui.areas:
        if area.type=='VIEW_3D':
            area.spaces.active.region_3d.view_perspective='CAMERA'
            area.spaces.active.shading.type='MATERIAL'
            area.spaces.active.overlay.show_overlays=False
scene.render.filepath=str(OUT/'preview.png')
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'specimen_collection_workstation.blend'))
bpy.ops.render.render(write_still=True)
camera.location=(0,-12,3.1)
aim(camera,(0,-.05,1.4))
scene.render.filepath=str(OUT/'preview_front.png')
bpy.ops.render.render(write_still=True)
print('BUILD_COMPLETE',str(OUT),flush=True)

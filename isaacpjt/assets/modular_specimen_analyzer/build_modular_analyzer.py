"""Build a reference-inspired modular clinical laboratory analyzer in Blender.

Standalone asset, metres, Z up, front -Y. No external models or textures.
Run with Blender --background --factory-startup --python this_file.py.
"""

from pathlib import Path
import json
import math

import bpy
from mathutils import Vector
from pxr import Usd, UsdGeom, UsdShade, UsdUtils


OUT = Path(__file__).resolve().parent
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
asset = bpy.data.collections.get('Collection')
asset.name = 'ModularSpecimenAnalyzer'
studio = bpy.data.collections.new('PreviewStudio_NOT_EXPORTED')
bpy.context.scene.collection.children.link(studio)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0
bpy.context.preferences.filepaths.save_version = 0


def move(obj, collection=asset):
    for coll in list(obj.users_collection):
        coll.objects.unlink(obj)
    collection.objects.link(obj)


def group(name, parent=None, loc=(0, 0, 0)):
    obj = bpy.data.objects.new(name, None)
    asset.objects.link(obj)
    obj.parent = parent
    obj.location = loc
    obj.empty_display_size = 0.07
    return obj


root = group('ModularSpecimenAnalyzer')
root['description'] = 'Reference-inspired 10-module laboratory analyzer visual prop'
root['front_axis'] = '-Y'
root['dimensions_are'] = 'Photo-estimated, not manufacturer dimensions'
root['physics'] = 'Visual only, no rigid bodies or collision shapes'


def mat(name, color, metal=0, roughness=0.4, alpha=1, emit=0):
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color, alpha)
    material.use_nodes = True
    bsdf = material.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Metallic'].default_value = metal
    bsdf.inputs['Roughness'].default_value = roughness
    bsdf.inputs['Alpha'].default_value = alpha
    if emit:
        bsdf.inputs['Emission Color'].default_value = (*color, 1)
        bsdf.inputs['Emission Strength'].default_value = emit
    return material


white = mat('CabinetPorcelainWhite', (0.72, 0.77, 0.78), 0.1, 0.32)
trim = mat('IvoryHoodFrames', (0.84, 0.87, 0.85), 0.1, 0.27)
navy = mat('DeepNavyPlinth', (0.014, 0.021, 0.074), 0.16, 0.37)
steel = mat('SatinStainless', (0.44, 0.52, 0.56), 0.85, 0.30)
alum = mat('BrightAluminium', (0.68, 0.74, 0.76), 0.85, 0.22)
dark = mat('InteriorGraphite', (0.021, 0.027, 0.033), 0.2, 0.44)
rubber = mat('RubberAndBelt', (0.013, 0.018, 0.020), 0, 0.66)
smoke = mat('SmokedAcrylic_Upper', (0.075, 0.065, 0.062), 0.07, 0.19, alpha=0.40)
smoke_dark = mat('SmokedAcrylic_Lower', (0.037, 0.026, 0.025), 0.07, 0.22, alpha=0.54)
green = mat('GreenReadyIndicators', (0.065, 0.57, 0.18), 0.12, 0.27, emit=0.25)
cap_green = mat('GreenRackCaps', (0.22, 0.52, 0.06), 0, 0.4)
cap_purple = mat('PurpleRackCaps', (0.35, 0.17, 0.40), 0, 0.43)
amber = mat('AmberStatusLamp', (0.95, 0.32, 0.02), 0, 0.32)
red = mat('ProbeCarriageRed', (0.46, 0.065, 0.09), 0.34, 0.32)
ink = mat('PrintedGraphite', (0.095, 0.13, 0.15), 0, 0.5)
label = mat('LabelAndTubeIvory', (0.85, 0.89, 0.85), 0, 0.46)
tube_mat = mat('FrostedSpecimenTubes', (0.52, 0.64, 0.63), 0.05, 0.28)
screen = mat('ScreenBackground', (0.045, 0.09, 0.12), 0, 0.35, emit=0.12)
screen_blue = mat('ScreenBlue', (0.12, 0.49, 0.64), 0, 0.4, emit=0.1)


def finish(obj, name, material, parent):
    obj.name = name
    move(obj)
    obj.parent = parent
    obj.data.materials.append(material)
    return obj


def bevel(obj, width=0.002, segments=2):
    if width > 0:
        mod = obj.modifiers.new('ManufacturedEdgeRounding', 'BEVEL')
        mod.width = width
        mod.segments = segments
        mod.limit_method = 'ANGLE'
        n = obj.modifiers.new('WeightedNormals', 'WEIGHTED_NORMAL')
        n.keep_sharp = True
    return obj


def box(name, loc, size, material, parent, radius=0.002):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = bpy.context.object
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    finish(obj, name, material, parent)
    obj.location = loc
    return bevel(obj, min(radius, min(size) * 0.4))


def cyl(name, loc, radius, depth, material, parent, axis='Z', vertices=20):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth)
    obj = finish(bpy.context.object, name, material, parent)
    obj.location = loc
    if axis == 'X':
        obj.rotation_euler.y = math.pi / 2
    elif axis == 'Y':
        obj.rotation_euler.x = math.pi / 2
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return bevel(obj, min(0.0008, depth * 0.1))


def text(name, content, loc, size, parent, material=ink, rotation=(math.pi / 2, 0, 0), align='CENTER'):
    data = bpy.data.curves.new(name, 'FONT')
    data.body = content
    data.size = size
    data.align_x = align
    data.align_y = 'CENTER'
    data.resolution_u = 2
    data.extrude = 0
    obj = bpy.data.objects.new(name, data)
    asset.objects.link(obj)
    obj.parent = parent
    obj.location = loc
    obj.rotation_euler = rotation
    data.materials.append(material)
    return obj


def extrude_yz(name, polygon, x, width, material, parent):
    count = len(polygon)
    points = [(xx, y, z) for xx in [x - width / 2, x + width / 2] for y, z in polygon]
    faces = [tuple(reversed(range(count))), tuple(range(count, count * 2))]
    faces += [(i, (i + 1) % count, (i + 1) % count + count, i + count) for i in range(count)]
    data = bpy.data.meshes.new(name)
    data.from_pydata(points, [], faces)
    data.update()
    obj = bpy.data.objects.new(name, data)
    asset.objects.link(obj)
    obj.parent = parent
    data.materials.append(material)
    return bevel(obj, 0.003, 3)


def cover_y(z):
    # The front face rises almost vertically, then curves back into its roof.
    return -0.426 + 0.122 * max(0, (z - 1.43) / 0.35) ** 2


def cover(name, width, z0, z1, material, parent):
    steps = 10
    verts = []
    for n in range(steps + 1):
        z = z0 + (z1 - z0) * n / steps
        verts += [(-width / 2, cover_y(z), z), (width / 2, cover_y(z), z)]
    faces = [(2*n, 2*n+1, 2*n+3, 2*n+2) for n in range(steps)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    asset.objects.link(obj)
    obj.parent = parent
    mesh.materials.append(material)
    for p in mesh.polygons:
        p.use_smooth = True
    return obj


def rack(parent, x, y, z, count=8, cap=cap_green, prefix='SpecimenRack'):
    rack_width = count * 0.031 + 0.012
    box(prefix + '_Carrier', (x, y, z), (rack_width, 0.067, 0.040), label, parent, 0.003)
    for n in range(count):
        xx = x + (n - (count - 1) / 2) * 0.031
        cyl(prefix + '_Well', (xx, y, z + 0.022), 0.0128, 0.006, dark, parent)
        cyl(prefix + '_Tube', (xx, y, z + 0.063), 0.0095, 0.085, tube_mat, parent)
        cyl(prefix + '_Paper', (xx, y, z + 0.062), 0.0097, 0.037, label, parent)
        cyl(prefix + '_Cap', (xx, y, z + 0.109), 0.011, 0.012, cap, parent)
        box(prefix + '_Barcode', (xx, y - 0.010, z + 0.06), (0.003, 0.0004, 0.026), ink, parent, 0)


modules = [
    ('01_Intake', 0.38, 'intake'),
    ('02_Chemistry_A', 0.76, 'chemistry'),
    ('03_Chemistry_B', 0.76, 'chemistry'),
    ('04_TransportBridge', 0.56, 'bridge'),
    ('05_Immunoassay', 0.76, 'chemistry'),
    ('06_TransferTower', 0.40, 'transfer'),
    ('07_ReactionModule', 0.76, 'reaction'),
    ('08_WashStation', 0.40, 'wash'),
    ('09_SampleProcessing', 0.76, 'processing'),
    ('10_OutputRack', 0.96, 'output'),
]
total_width = sum(w for _, w, _ in modules)
cursor = -total_width / 2

for index, (name, width, kind) in enumerate(modules):
    center = cursor + width / 2
    cursor += width
    module = group(name, root, (center, 0, 0))
    module['module_role'] = kind
    module['nominal_width_m'] = width
    body = group('LowerCabinet', module)
    upper = group('UpperHousingAndCovers', module)
    internals = group('SimplifiedVisibleMechanism', module)
    # Panel-built lower enclosure, intentionally hollow.
    box('NavyBasePlinth', (0, 0, 0.083), (width - 0.005, 0.818, 0.086), navy, body, 0.004)
    box('CabinetBase', (0, 0, 0.133), (width - 0.015, 0.79, 0.024), steel, body)
    box('RearCabinetSkin', (0, 0.39, 0.565), (width - 0.012, 0.026, 0.882), white, body)
    for x in [-width/2 + 0.012, width/2 - 0.012]:
        box('CabinetSideSkin', (x, 0, 0.565), (0.018, 0.785, 0.882), white, body)
    doors = 2 if width >= 0.65 else 1
    for n in range(doors):
        dw = (width - 0.018) / doors
        x = (n - (doors - 1)/2) * dw
        box('FrontCabinetDoor', (x, -0.403, 0.565), (dw - 0.006, 0.026, 0.877), white, body, 0.003)
        if kind == 'processing':
            box('WasteLevelWindowTrim', (x, -0.418, 0.655), (0.073, 0.008, 0.457), steel, body, 0.014)
            box('WasteLevelWindow', (x, -0.423, 0.655), (0.060, 0.004, 0.44), dark, body, 0.012)
            box('WasteLevelLowMark', (x, -0.426, 0.50), (0.045, 0.001, 0.004), ink, body, 0)
        else:
            box('FlushDoorPull', (x + dw * 0.32, -0.418, 0.976), (0.052, 0.004, 0.008), steel, body, 0.001)
    box('WorktopLip', (0, -0.013, 1.018), (width - 0.006, 0.855, 0.032), trim, body, 0.004)
    for x in [-width * 0.32, width * 0.32]:
        for y in [-0.30, 0.28]:
            cyl('LevellingFoot', (x, y, 0.022), 0.032, 0.044, rubber, body)

    if kind == 'bridge':
        box('BridgeConsole', (0, 0, 1.079), (width - 0.02, 0.76, 0.095), white, upper, 0.004)
        box('ExposedTransportBelt', (0, -0.035, 1.132), (width, 0.305, 0.013), rubber, upper)
        for y in [-0.195, 0.125]:
            box('BridgeGuideRail', (0, y, 1.156), (width, 0.017, 0.028), alum, upper, 0.003)
        box('BridgeIDBadge', (0, -0.419, 0.814), (0.206, 0.008, 0.068), navy, body, 0.003)
        text('BridgeID', 'LAB / LINK', (0, -0.424, 0.815), 0.018, body, label)
        box('BridgeButtonPlate', (-0.17, -0.423, 0.587), (0.046, 0.009, 0.116), steel, body)
        for z, material in [(0.620, green), (0.583, amber), (0.546, dark)]:
            cyl('BridgeStatusButton', (-0.17, -0.434, z), 0.013, 0.012, material, body, 'Y')
        continue

    # Upper chamber and slightly curved white ribs around smoked acrylic.
    box('UpperRearSkin', (0, 0.373, 1.405), (width - 0.042, 0.045, 0.727), white, upper)
    box('DarkMechanismBackdrop', (0, 0.346, 1.391), (width - 0.063, 0.012, 0.668), dark, internals)
    for x in [-width/2 + 0.013, width/2 - 0.013]:
        side_poly = [(-0.404,1.052), (0.397,1.052), (0.397,1.749), (-0.277,1.780), (-0.344,1.712), (-0.393,1.556)]
        extrude_yz('HoodSideCheek', side_poly, x, 0.022, trim, upper)
        profile = [(cover_y(1.07 + n * .7 / 12) - 0.012, 1.07 + n * .7 / 12) for n in range(13)]
        profile += [(y + 0.034, z) for y, z in reversed(profile)]
        extrude_yz('CurvedWhiteHoodUpright', profile, x, 0.031, trim, upper)
        # Visible outside rail emphasizes each modular boundary.
        box('UprightSideRecess', (x, -0.367, 1.347), (0.010, 0.021, 0.504), steel, upper)
    box('TopHeader', (0, -0.26, 1.774), (width - 0.027, 0.141, 0.046), trim, upper, 0.012)
    box('TopRoof', (0, 0.089, 1.767), (width - 0.035, 0.60, 0.029), white, upper, 0.004)
    box('HeaderReadyLamp', (0, -0.291, 1.801), (0.049, 0.031, 0.009), green, upper, 0.003)
    text('HeaderModuleID', f'{index + 1:02} / {kind.upper()}', (0, -0.336, 1.773), 0.013, upper)
    window_bottom = 1.292 if kind in ('chemistry', 'output') else 1.091
    cover('SmokedLowerDoor', width - 0.083, window_bottom, 1.456, smoke_dark, upper)
    cover('SmokedUpperDoor', width - 0.083, 1.456, 1.751, smoke, upper)
    box('DoorHorizontalDivision', (0, cover_y(1.455)-0.003, 1.455), (width - 0.078, 0.009, 0.009), dark, upper)
    box('DoorLowerMetalEdge', (0, cover_y(window_bottom), window_bottom), (width - 0.069, 0.018, 0.021), steel, upper, 0.003)
    box('HoodPull', (0, cover_y(window_bottom)-0.015, window_bottom+0.006), (min(0.155,width*0.43),0.028,0.023), trim, upper, 0.004)
    box('LowerChamberDeck', (0, -0.014, 1.064), (width - 0.07, 0.699, 0.033), dark, internals)
    for z in [1.119, 1.232]:
        box('InteriorTransportRail', (0, -0.188, z), (width - 0.10, 0.017, 0.016), alum, internals)
    box('InteriorBelt', (0,-0.042,1.104), (width-0.083,0.19,0.014), rubber, internals)
    box('InteriorBlueSensor', (width*0.3,-0.180,1.132), (0.037,0.03,0.023), screen_blue, internals)

    if kind == 'chemistry':
        for n in range(4):
            dw = (width - .104) / 4
            x = (n-1.5)*dw
            box('ReagentDrawer', (x, -.348, 1.163), (dw-.008,.168,.194), trim, upper, .004)
            box('ReagentDrawerLowerGrip', (x, -.438, 1.110), (dw-.025,.014,.041), steel, upper, .003)
            text('ReagentDrawerNumber', str(n+1), (x,-.443,1.040), .014, upper)
        cyl('CircularReactionRotor', (0,0.11,1.401), .216,.071, steel, internals, vertices=56)
        cyl('RotorCover', (0,0.11,1.443), .195,.017, dark, internals, vertices=56)
        for n in range(16):
            a = n*math.tau/16
            cyl('RotorCuvette', (.164*math.cos(a),.11+.164*math.sin(a),1.458),.012,.015,label,internals,vertices=12)
        box('PipetteGantry', (0,0.021,1.598), (width-.123,.064,.047), steel, internals)
        for x in [-.133,.145]:
            box('GantrySlide', (x,.008,1.566), (.049,.076,.062), dark, internals)
            cyl('RedPipetteHousing', (x,-.031,1.481), .013,.15, red, internals)
            cyl('PipetteNeedle', (x,-.031,1.377), .003,.07, alum, internals,vertices=12)
    elif kind == 'intake':
        for z in [1.17,1.29,1.41]:
            box('IntakeRackShelf',(0,-.08,z),(width-.1,.42,.018),steel,internals)
        rack(internals,0,-.22,1.201,count=7,cap=cap_purple,prefix='Intake')
        for n in range(10):
            box('IntakeRearSlots',((n-4.5)*.023,.323,1.529),(.011,.005,.115),steel,internals,.001)
    elif kind == 'reaction':
        cyl('LargeReactionBath',(-.089,.053,1.252),.195,.24,trim,internals,vertices=56)
        cyl('BathStainlessRim',(-.089,.053,1.379),.196,.016,steel,internals,vertices=56)
        cyl('BathDarkLid',(-.089,.053,1.391),.177,.009,dark,internals,vertices=56)
        box('InternalDisplayBezel',(-.071,.218,1.542),(.261,.033,.187),dark,internals,.008)
        box('InternalDisplayScreen',(-.071,.199,1.542),(.236,.003,.163),screen,internals,.002)
        text('DisplayTitle','SYSTEM READY',(-.071,.196,1.595),.016,internals,label)
        for n in range(4):
            box('DisplayStatusLine',(-.07,.195,1.559-n*.024),(.153-n*.019,.002,.006),screen_blue,internals,.001)
        for x in [.187,.26]:
            cyl('ReactionProbe',(x,-.025,1.405),.010,.278,steel,internals)
            box('ProbeActuator',(x,.0,1.569),(.050,.068,.057),dark,internals)
    elif kind == 'transfer':
        for z in [1.27,1.40,1.53,1.66]:
            box('TransferShelf',(0,.065,z),(width-.085,.40,.014),steel,internals)
        box('TransferRobotColumn',(.10,-.06,1.395),(.035,.045,.57),red,internals)
        rack(internals,-.013,-.22,1.115,count=7,prefix='Transfer')
    elif kind == 'wash':
        box('WashModuleMetalBox',(0,.086,1.474),(width-.094,.39,.308),steel,internals,.006)
        text('WashModulePrint','WASH',(0,-.113,1.484),.036,internals)
        cyl('WashReservoir',(0,-.023,1.167),.089,.17,label,internals,vertices=28)
    elif kind == 'processing':
        box('ProcessingGantry',(0,.015,1.53),(width-.10,.055,.039),steel,internals)
        for x in [-.11,-.055,0,.055]:
            cyl('ProcessingRedProbe',(x,-.08,1.368),.010,.23,red,internals)
            cyl('ProcessingNeedle',(x,-.08,1.219),.0025,.069,alum,internals,vertices=12)
        box('ProbeCarriage',(0,.005,1.514),(.261,.077,.071),dark,internals)
    elif kind == 'output':
        box('OutputRecessBackplate',(0,-.237,1.159),(width-.103,.010,.206),dark,internals)
        box('OutputTray',(0,-.376,1.084),(width-.115,.167,.025),steel,internals)
        for n in range(4):
            rack(internals,(n-1.5)*.211,-.345,1.116,count=6,prefix=f'OutputRack{n+1}')
            text('OutputRackNumber',f'{n+1:02}',((n-1.5)*.211,-.445,1.045),.012,upper)
        box('OutputGantry',(0,.01,1.518),(width-.11,.04,.035),steel,internals)
        box('OutputVerticalCarriage',(.10,.017,1.422),(.057,.062,.25),red,internals)
        box('OutputCoverLabel',(width*.31,cover_y(1.634)-.002,1.634),(.115,.002,.128),label,upper,.001)
    # Small real hardware at bottom corners of every viewing cover.
    for x in [-width/2+.051,width/2-.051]:
        cyl('CoverFastener',(x,cover_y(window_bottom)-.005,window_bottom+.037),.0035,.003,steel,upper,'Y',12)

# Clean end skins, side ventilation, and external side identification plate.
ends = group('EndPanels',root)
for sign in [-1,1]:
    box('NavyEndCorner',(sign*(total_width/2+.006),.0,.531),(.018,.817,.81),navy,ends,.003)
    box('WhiteEndSidePanel',(sign*(total_width/2+.017),.042,.592),(.014,.704,.887),white,ends,.003)
    for n in range(13):
        box('EndVentSlot',(sign*(total_width/2+.025),.11+(n-6)*.029,.817),(.002,.013,.145),ink,ends,.001)

# Convert lettering and modifiers to simple portable meshes.
bpy.ops.object.select_all(action='DESELECT')
for obj in list(asset.objects):
    if obj.type == 'FONT':
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.convert(target='MESH')
        obj.select_set(False)
    elif obj.type == 'MESH':
        bpy.context.view_layer.objects.active = obj
        for mod in list(obj.modifiers):
            bpy.ops.object.modifier_apply(modifier=mod.name)

for obj in asset.objects:
    obj.select_set(True)
bpy.context.view_layer.objects.active = root
usd_file = OUT/'modular_specimen_analyzer.usd'
bpy.ops.wm.usd_export(
    filepath=str(usd_file), selected_objects_only=True, export_materials=True,
    generate_preview_surface=True, export_textures_mode='KEEP',
    export_animation=False, export_lights=False, export_cameras=False,
    convert_world_material=False, root_prim_path='/LabAnalyzer',
    convert_scene_units='METERS', meters_per_unit=1.0,
)
stage = Usd.Stage.Open(str(usd_file))
stage.SetDefaultPrim(stage.GetPrimAtPath('/LabAnalyzer'))
UsdGeom.SetStageMetersPerUnit(stage,1.0)
UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
# Acrylic is a single surface shell. Author double-sided shading explicitly.
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh) and 'Smoked' in str(prim.GetPath()):
        UsdGeom.Mesh(prim).CreateDoubleSidedAttr(True)
    # Blender's exporter can write Alpha=1 despite the Cycles material value.
    # Preserve both acrylic opacities explicitly in the USD Preview Surface.
    if prim.IsA(UsdShade.Shader) and 'SmokedAcrylic_' in str(prim.GetPath()):
        opacity = 0.40 if 'SmokedAcrylic_Upper' in str(prim.GetPath()) else 0.54
        shader = UsdShade.Shader(prim)
        assert shader.GetInput('opacity'), 'Acrylic shader has no opacity input'
        shader.GetInput('opacity').Set(opacity)
stage.GetRootLayer().Save()
meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
_,assets,missing = UsdUtils.ComputeAllDependencies(str(usd_file))
assert not assets and not missing
stats = {
    'default_prim':'/LabAnalyzer', 'units':'metres', 'up_axis':'Z', 'front':'-Y',
    'modules':10, 'bounds_min':list(bounds.GetMin()), 'bounds_max':list(bounds.GetMax()),
    'size_m':list(bounds.GetSize()), 'mesh_count':len(meshes),
    'triangles_equivalent':sum(sum(n-2 for n in m.GetFaceVertexCountsAttr().Get()) for m in meshes),
    'material_count':sum(p.IsA(UsdShade.Material) for p in stage.Traverse()),
    'external_assets':list(assets), 'unresolved_assets':list(missing),
}
(OUT/'model_info.json').write_text(json.dumps(stats,indent=2)+'\n')
print('USD_VALIDATED',json.dumps(stats),flush=True)

# Studio lives only in the Blender file; it is not in the exported USD.
floor_mat = mat('PreviewFloor',(0.54,.58,.59),0,.84)
floor = box('StudioFloor',(0,0,-.033),(200,200,.06),floor_mat,None,0)
move(floor,studio)
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (.60,.67,.72,1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = .38


def aim(obj,at):
    obj.rotation_euler = (Vector(at)-obj.location).to_track_quat('-Z','Y').to_euler()


def light(name,loc,power,size,size_y,color):
    data = bpy.data.lights.new(name,'AREA')
    data.energy = power
    data.shape = 'RECTANGLE'
    data.size = size
    data.size_y = size_y
    data.color = color
    obj = bpy.data.objects.new(name,data)
    studio.objects.link(obj)
    obj.location = loc
    aim(obj,(0,0,.8))


light('LargeFrontSoftbox',(-2.5,-4.5,6),1350,6,4,(1,.97,.92))
light('RightFill',(4,-1.5,4),800,3,3,(.85,.92,1))
light('LongTopEdge',(0,2.3,5.0),1400,6,2,(.91,.96,1))
data = bpy.data.cameras.new('HeroCamera')
camera = bpy.data.objects.new('HeroCamera',data)
studio.objects.link(camera)
camera.location = (2.6,-11,3.6)
aim(camera,(0,0,.91))
data.type = 'ORTHO'
data.ortho_scale = 7.5
scene.camera = camera
scene.render.engine = 'CYCLES'
scene.cycles.samples = 80
scene.cycles.use_denoising = True
scene.cycles.transparent_max_bounces = 12
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type = 'OPTIX'
    prefs.get_devices()
    if any(d.type=='OPTIX' for d in prefs.devices):
        for device in prefs.devices:
            device.use = device.type=='OPTIX'
        scene.cycles.device = 'GPU'
except Exception as exc:
    print('Using CPU render:',exc)
scene.render.resolution_x = 2300
scene.render.resolution_y = 1080
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.view_settings.view_transform = 'AgX'
scene.render.filepath = str(OUT/'preview.png')
bpy.ops.object.select_all(action='DESELECT')
root.select_set(True)
bpy.context.view_layer.objects.active = root
for screen_ui in bpy.data.screens:
    for area in screen_ui.areas:
        if area.type == 'VIEW_3D':
            area.spaces.active.region_3d.view_perspective = 'CAMERA'
            area.spaces.active.shading.type = 'MATERIAL'
            area.spaces.active.overlay.show_overlays = False
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'modular_specimen_analyzer.blend'))
bpy.ops.render.render(write_still=True)
camera.location = (0,-12,2.20)
aim(camera,(0,0,.89))
scene.render.filepath = str(OUT/'preview_front.png')
bpy.ops.render.render(write_still=True)
print('DONE',str(OUT),flush=True)

"""Create an original reference-inspired laboratory prop in Blender.

Run: blender --background --factory-startup --python build_specimen_processor.py
Units: metres. Front: -Y. Up: +Z. The bottom of the feet is at Z=0.
No downloaded models, texture files, or external Python packages are needed.
"""

from pathlib import Path
import json
import math

import bpy
import bmesh
from mathutils import Vector


OUT = Path(__file__).resolve().parent
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
for collection in list(bpy.data.collections):
    if collection.name != 'Collection':
        bpy.data.collections.remove(collection)
asset = bpy.data.collections.get('Collection')
asset.name = 'SpecimenProcessor'
studio = bpy.data.collections.new('PreviewStudio_NOT_EXPORTED')
bpy.context.scene.collection.children.link(studio)
scene = bpy.context.scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0


def move_to(obj, collection):
    for coll in list(obj.users_collection):
        coll.objects.unlink(obj)
    collection.objects.link(obj)
    return obj


def empty(name, parent=None, location=(0, 0, 0)):
    obj = bpy.data.objects.new(name, None)
    asset.objects.link(obj)
    obj.parent = parent
    obj.location = location
    obj.empty_display_size = 0.045
    return obj


root = empty('SpecimenProcessor')
root['description'] = 'Reference-inspired hollow benchtop laboratory specimen processor; visual prop'
root['width_m'] = 0.64
root['depth_m'] = 0.49
root['front_axis'] = '-Y'
root['physics'] = 'Visual geometry only; no collision or rigid body enabled'
shell = empty('Housing', root)
chamber = empty('StainlessChamber', root)
controls = empty('FrontControls', root)
tray_group = empty('RemovablePerforatedTray', root)
tubes = empty('OptionalSpecimenTubes_HIDE_OR_DELETE', root)
lid = empty('LidHinge_ROTATE_LOCAL_X', root, (0, 0.204, 0.312))
lid.rotation_euler.x = math.radians(-103)
lid['open_angle_degrees'] = 103
lid['closed_rotation_x_degrees'] = 0


def material(name, color, metallic=0.0, roughness=0.4, emission=None):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Metallic'].default_value = metallic
    bsdf.inputs['Roughness'].default_value = roughness
    if emission:
        bsdf.inputs['Emission Color'].default_value = (*color, 1)
        bsdf.inputs['Emission Strength'].default_value = emission
    return mat


enamel = material('WarmWhite_PowderCoatedSteel', (0.81, 0.845, 0.83), 0.12, 0.3)
steel = material('SatinStainlessSteel', (0.48, 0.55, 0.59), 0.87, 0.3)
polished = material('PolishedEdgeSteel', (0.66, 0.73, 0.76), 0.95, 0.21)
dark = material('GraphiteControlFascia', (0.019, 0.029, 0.034), 0.3, 0.36)
rubber = material('RubberGasketsAndFeet', (0.015, 0.021, 0.025), 0.0, 0.69)
knob_mat = material('MouldedKnobs', (0.035, 0.045, 0.05), 0.12, 0.28)
ink = material('LightPanelMarkings', (0.75, 0.82, 0.84), 0.1, 0.5)
dark_ink = material('DarkPrintedMarkings', (0.075, 0.12, 0.13), 0.0, 0.46)
teal = material('LabTealAccent', (0.06, 0.30, 0.32), 0.22, 0.34)
green = material('ReadyIndicator', (0.30, 0.86, 0.38), 0.12, 0.25, 1.5)
amber = material('AmberIndicator', (0.97, 0.26, 0.035), 0.08, 0.3, 0.4)
glass = material('FrostedSampleTubes', (0.63, 0.76, 0.81), 0.15, 0.2)
cap_purple = material('LavenderTubeCaps', (0.35, 0.20, 0.51), 0.0, 0.43)
label_mat = material('TubePaperLabels', (0.91, 0.92, 0.86), 0.0, 0.66)


def finish(obj, name, mat, parent=root):
    obj.name = name
    move_to(obj, asset)
    obj.parent = parent
    obj.data.materials.append(mat)
    return obj


def bevel(obj, width, segments=3):
    if width:
        modifier = obj.modifiers.new('Soft manufactured edges', 'BEVEL')
        modifier.width = width
        modifier.segments = segments
        modifier.limit_method = 'ANGLE'
        normals = obj.modifiers.new('Weighted surface normals', 'WEIGHTED_NORMAL')
        normals.keep_sharp = True
        normals.weight = 30
    return obj


def box(name, loc, size, mat, parent=root, radius=0.0015):
    bpy.ops.mesh.primitive_cube_add(size=1)
    obj = bpy.context.object
    obj.dimensions = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    finish(obj, name, mat, parent)
    obj.location = loc
    return bevel(obj, min(radius, min(size) * 0.42))


def cylinder(name, loc, radius, depth, mat, parent=root, axis='Z', vertices=40):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius, depth=depth)
    obj = finish(bpy.context.object, name, mat, parent)
    obj.location = loc
    if axis == 'Y':
        obj.rotation_euler.x = math.pi / 2
    elif axis == 'X':
        obj.rotation_euler.y = math.pi / 2
    for face in obj.data.polygons:
        face.use_smooth = len(face.vertices) == 4
    return bevel(obj, min(0.001, depth * 0.15), 2)


def text(name, value, loc, height, mat=ink, parent=controls, rotation=(math.pi / 2, 0, 0), align='CENTER'):
    curve = bpy.data.curves.new(name, 'FONT')
    curve.body = value
    curve.size = height
    curve.align_x = align
    curve.align_y = 'CENTER'
    curve.extrude = 0.00004
    curve.resolution_u = 2
    obj = bpy.data.objects.new(name, curve)
    asset.objects.link(obj)
    obj.parent = parent
    obj.location = loc
    obj.rotation_euler = rotation
    curve.materials.append(mat)
    return obj


def screw(name, loc, parent, axis='Y', radius=0.0031):
    cylinder(name, loc, radius, 0.0014, steel, parent, axis, 20)
    x, y, z = loc
    if axis == 'Y':
        box(name + '_slot', (x, y - 0.0008, z), (radius * 1.35, 0.0003, 0.00065), dark, parent, 0.0001)
    else:
        box(name + '_slot', (x, y, z + 0.0008), (radius * 1.35, 0.00065, 0.0003), dark, parent, 0.0001)


# A hollow panel-built enclosure: no cube filling the sample chamber.
box('UndersidePlate', (0, 0, 0.036), (0.61, 0.451, 0.018), dark, shell, 0.004)
box('FrontEnclosure', (0, -0.221, 0.176), (0.64, 0.038, 0.272), enamel, shell, 0.006)
box('BackEnclosure', (0, 0.216, 0.174), (0.64, 0.030, 0.268), enamel, shell, 0.005)
for side in [-1, 1]:
    box('SideEnclosure_' + str(side), (side * 0.301, -0.005, 0.174), (0.038, 0.414, 0.268), enamel, shell, 0.005)
    box('TopRimSide_' + str(side), (side * 0.299, -0.005, 0.307), (0.042, 0.415, 0.016), enamel, shell, 0.003)
box('TopFrontRim', (0, -0.218, 0.307), (0.635, 0.043, 0.016), enamel, shell, 0.003)
box('TopBackRim', (0, 0.199, 0.307), (0.635, 0.043, 0.016), enamel, shell, 0.003)
for x in [-0.259, 0.259]:
    for y in [-0.181, 0.176]:
        cylinder('RubberIsolationFoot', (x, y, 0.017), 0.025, 0.034, rubber, shell)
        cylinder('FootSteelCollar', (x, y, 0.034), 0.021, 0.01, steel, shell)

# Continuous stainless inner well, inset below the white top flange.
box('ChamberFloor', (0, -0.003, 0.17), (0.559, 0.37, 0.006), steel, chamber)
box('WellFrontWall', (0, -0.193, 0.237), (0.56, 0.007, 0.137), steel, chamber)
box('WellBackWall', (0, 0.18, 0.237), (0.56, 0.007, 0.137), steel, chamber)
for x in [-0.28, 0.28]:
    box('WellSideWall', (x, -0.006, 0.237), (0.007, 0.377, 0.137), steel, chamber)
    box('GasketSide', (x, -0.006, 0.312), (0.006, 0.38, 0.004), rubber, chamber, 0.001)
    box('TraySupportRail', (x * 0.955, -0.006, 0.224), (0.014, 0.34, 0.009), polished, chamber)
for y in [-0.195, 0.183]:
    box('GasketCross', (0, y, 0.312), (0.565, 0.006, 0.004), rubber, chamber, 0.001)
for z in [0.28, 0.29]:
    box('PressedBackWallRib', (0, 0.1755, z), (0.536, 0.002, 0.0012), polished, chamber, 0.0004)

# The tray holes are real openings in a connected mesh, not black decals.
cols, rows, steps = 18, 10, 16
tray_w, tray_d = 0.512, 0.314
pitch_x, pitch_y = tray_w / cols, tray_d / rows
hole_radius = 0.0092
verts, faces = [], []
for row in range(rows):
    for col in range(cols):
        cx = -tray_w / 2 + pitch_x * (col + 0.5)
        cy = -tray_d / 2 + pitch_y * (row + 0.5) - 0.006
        base = len(verts)
        for ring in [0, 1]:
            for n in range(steps):
                angle = 2 * math.pi * n / steps
                # Rectangular cell boundaries share vertices with adjacent cells.
                unit_x, unit_y = math.cos(angle), math.sin(angle)
                if ring == 0:
                    px, py = unit_x * hole_radius, unit_y * hole_radius
                else:
                    length = 1 / max(abs(unit_x), abs(unit_y))
                    px = unit_x * length * pitch_x / 2
                    py = unit_y * length * pitch_y / 2
                verts.append((cx + px, cy + py, 0.241))
        for n in range(steps):
            nxt = (n + 1) % steps
            faces.append((base + n, base + steps + n, base + steps + nxt, base + nxt))
mesh = bpy.data.meshes.new('180TruePerforationsMesh')
mesh.from_pydata(verts, [], faces)
mesh.update()
bm = bmesh.new()
bm.from_mesh(mesh)
bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.000001)
bm.to_mesh(mesh)
bm.free()
tray = bpy.data.objects.new('PerforatedTray_180_OpenHoles', mesh)
asset.objects.link(tray)
tray.parent = tray_group
mesh.materials.append(steel)
solid = tray.modifiers.new('SheetMetal_2mm', 'SOLIDIFY')
solid.thickness = 0.002
bevel(tray, 0.00025, 2)
for x in [-0.260, 0.260]:
    box('TrayLiftEdge', (x, -0.006, 0.250), (0.010, 0.334, 0.024), steel, tray_group, 0.0015)
for y in [-0.168, 0.156]:
    box('TrayShortEdge', (0, y, 0.244), (0.526, 0.010, 0.010), steel, tray_group, 0.001)

# Six removable sample tubes, confined to the rear-left corner of the rack.
for i, (col, row) in enumerate([(2, 6), (3, 6), (4, 6), (2, 7), (3, 7), (4, 7)]):
    x = -tray_w / 2 + pitch_x * (col + 0.5)
    y = -tray_d / 2 + pitch_y * (row + 0.5) - 0.006
    tube = empty(f'SampleTube_{i + 1:02}', tubes, (x, y, 0))
    cylinder('TubeBody', (0, 0, 0.257), 0.0073, 0.080, glass, tube, vertices=24)
    cylinder('TubeLabel', (0, 0, 0.265), 0.00745, 0.023, label_mat, tube, vertices=24)
    cylinder('LavenderCap', (0, 0, 0.303), 0.0088, 0.016, cap_purple, tube, vertices=24)
    cylinder('CapInset', (0, 0, 0.311), 0.0059, 0.001, cap_purple, tube, vertices=24)

# Open lid; all its detail is local to a real hinge empty for easy posing.
box('LidOuterShell', (0, -0.198, 0.012), (0.607, 0.406, 0.025), enamel, lid, 0.005)
box('LidInnerRecess', (0, -0.198, -0.0015), (0.575, 0.374, 0.004), rubber, lid, 0.002)
box('LidInnerSteel', (0, -0.198, -0.004), (0.560, 0.360, 0.005), steel, lid, 0.002)
for x in [-0.29, 0.29]:
    box('LidMetalSideLip', (x, -0.198, -0.004), (0.009, 0.386, 0.009), polished, lid, 0.0018)
for y in [-0.387, -0.009]:
    box('LidMetalCrossLip', (0, y, -0.004), (0.585, 0.009, 0.009), polished, lid, 0.0018)
for x in [-0.080, 0.080]:
    cylinder('LidFastener', (x, -0.343, -0.0078), 0.0043, 0.0016, polished, lid, vertices=24)
    box('LidFastenerSlot', (x, -0.343, -0.0087), (0.005, 0.0008, 0.0003), dark, lid, 0.0001)
box('LidHandleBase', (0, -0.384, 0.031), (0.16, 0.014, 0.012), steel, lid, 0.003)
box('LidHandleGrip', (0, -0.391, 0.041), (0.145, 0.019, 0.017), dark, lid, 0.004)
for x in [-0.218, 0.218]:
    cylinder('HingeBarrel', (x, 0.204, 0.312), 0.011, 0.071, steel, shell, axis='X')
    box('HingeBracket', (x, 0.195, 0.302), (0.048, 0.034, 0.015), polished, shell)

# Front control fascia and two physical rotary knobs.
box('ControlPanelBezel', (0, -0.242, 0.130), (0.539, 0.007, 0.137), steel, controls, 0.005)
box('BlackControlPanel', (0, -0.247, 0.130), (0.528, 0.006, 0.126), dark, controls, 0.004)
for x in [-0.174, 0.174]:
    cylinder('DialSteelSurround', (x, -0.252, 0.129), 0.039, 0.002, steel, controls, axis='Y')
    cylinder('DialScaleBackground', (x, -0.2535, 0.129), 0.037, 0.0015, dark, controls, axis='Y')
    for n in range(13):
        a = math.radians(-125 + n * (250 / 12))
        xx = x + 0.033 * math.sin(a)
        zz = 0.129 + 0.033 * math.cos(a)
        tick = box('DialTick', (xx, -0.2546, zz), (0.0008, 0.0004, 0.0042 if n % 3 == 0 else 0.0024), ink, controls, 0.0001)
        tick.rotation_euler.y = a
    cylinder('RotaryKnob', (x, -0.264, 0.129), 0.023, 0.021, knob_mat, controls, axis='Y', vertices=48)
    grip = box('KnobFingerGrip', (x, -0.279, 0.129), (0.010, 0.013, 0.040), knob_mat, controls, 0.003)
    grip.rotation_euler.y = math.radians(-22 if x < 0 else 30)
    pointer = box('KnobWhitePointer', (x, -0.287, 0.146), (0.003, 0.001, 0.007), ink, controls, 0.0005)
    pointer.rotation_euler.y = grip.rotation_euler.y
    text('DialHeading', 'TEMPERATURE' if x < 0 else 'TIMER', (x, -0.251, 0.182), 0.0078)
    text('DialUnit', '20 - 200 C' if x < 0 else '0 - 60 MIN', (x, -0.251, 0.081), 0.0065)
for x in [-0.252, 0.252]:
    for z in [0.082, 0.178]:
        screw('FasciaScrew', (x, -0.252, z), controls)
text('PanelEquipmentTitle', 'SPECIMEN PROCESSOR', (0, -0.251, 0.170), 0.009)
text('PanelSubtitle', 'LABORATORY EQUIPMENT', (0, -0.251, 0.157), 0.0058)
for x, mat, label in [(-0.031, green, 'READY'), (0.031, amber, 'HEAT')]:
    cylinder('IndicatorSteelBezel', (x, -0.253, 0.115), 0.010, 0.004, polished, controls, axis='Y')
    cylinder('IndicatorLens', (x, -0.257, 0.115), 0.0074, 0.005, mat, controls, axis='Y')
    text('IndicatorLabel', label, (x, -0.251, 0.091), 0.006)
text('PrintedModelName', 'LAB / 03', (-0.260, -0.2405, 0.265), 0.017, dark_ink, shell, align='LEFT')
text('PrintedModelSubtitle', 'SPECIMEN PREPARATION', (-0.260, -0.2405, 0.243), 0.0065, dark_ink, shell, align='LEFT')
box('TealModelAccent', (0.237, -0.2407, 0.260), (0.044, 0.0009, 0.0035), teal, shell, 0.0003)

# Right-side inset ventilation panel and back service details.
box('SideVentRecess', (0.3202, 0.063, 0.117), (0.0018, 0.211, 0.076), dark, shell, 0.0006)
for z in [0.087 + n * 0.009 for n in range(8)]:
    box('SideVentLouvre', (0.322, 0.063, z), (0.003, 0.194, 0.005), enamel, shell, 0.001)
box('BackServicePanel', (0, 0.232, 0.12), (0.23, 0.003, 0.12), steel, shell, 0.003)
box('PowerSocket', (0.060, 0.236, 0.103), (0.031, 0.01, 0.022), dark, shell, 0.001)
box('BackPowerSwitch', (-0.067, 0.236, 0.103), (0.019, 0.009, 0.026), rubber, shell, 0.001)

# Convert lettering to mesh and apply modelling modifiers for stable USD export.
bpy.ops.object.select_all(action='DESELECT')
for obj in list(asset.objects):
    if obj.type == 'FONT':
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.convert(target='MESH')
        obj.select_set(False)
    elif obj.type == 'MESH':
        bpy.context.view_layer.objects.active = obj
        for modifier in list(obj.modifiers):
            bpy.ops.object.modifier_apply(modifier=modifier.name)

# Export only the asset. USD Preview Surface materials are portable and texture-free.
for obj in asset.objects:
    obj.select_set(True)
bpy.context.view_layer.objects.active = root
usd_path = OUT / 'specimen_processor.usd'
bpy.ops.wm.usd_export(
    filepath=str(usd_path),
    selected_objects_only=True,
    export_materials=True,
    generate_preview_surface=True,
    export_textures_mode='KEEP',
    export_animation=False,
    export_lights=False,
    export_cameras=False,
    convert_world_material=False,
    root_prim_path='/LabEquipment',
    convert_scene_units='METERS',
    meters_per_unit=1.0,
)

# Verify the exported USD and capture facts for downstream Isaac Sim use.
from pxr import Usd, UsdGeom, UsdShade
stage = Usd.Stage.Open(str(usd_path))
stage.SetDefaultPrim(stage.GetPrimAtPath('/LabEquipment'))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.GetRootLayer().Save()
usd_meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
usd_materials = [p for p in stage.Traverse() if p.IsA(UsdShade.Material)]
assert usd_meshes, 'USD contains no meshes'
assert stage.GetDefaultPrim(), 'USD requires a default prim'
assert not any(p.IsA(UsdGeom.Camera) for p in stage.Traverse())
assert not any('Light' in p.GetTypeName() for p in stage.Traverse())
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
stats = {
    'usd_default_prim': str(stage.GetDefaultPrim().GetPath()),
    'up_axis': str(UsdGeom.GetStageUpAxis(stage)),
    'meters_per_unit': UsdGeom.GetStageMetersPerUnit(stage),
    'mesh_count': len(usd_meshes),
    'material_count': len(usd_materials),
    'vertices': sum(len(UsdGeom.Mesh(p).GetPointsAttr().Get()) for p in usd_meshes),
    'triangles_equivalent': sum(sum(n - 2 for n in UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) for p in usd_meshes),
    'bounds_min': list(bbox.GetMin()),
    'bounds_max': list(bbox.GetMax()),
    'size_m': list(bbox.GetSize()),
}
(OUT / 'model_info.json').write_text(json.dumps(stats, indent=2) + '\n')
print('ASSET_VERIFIED', json.dumps(stats), flush=True)

# Neutral studio, kept separate in the blend and excluded from the USD asset.
floor_mat = material('StudioWarmGrey', (0.145, 0.18, 0.20), 0, 0.86)
floor = box('StudioFloor_NOT_EXPORTED', (0, 0, -0.015), (200, 200, 0.025), floor_mat, None, 0)
move_to(floor, studio)
scene.world.color = (0.20, 0.20, 0.20)
scene.world.use_nodes = True
scene.world.node_tree.nodes['Background'].inputs['Color'].default_value = (0.32, 0.39, 0.46, 1)
scene.world.node_tree.nodes['Background'].inputs['Strength'].default_value = 0.35


def point_at(obj, target):
    obj.rotation_euler = (Vector(target) - obj.location).to_track_quat('-Z', 'Y').to_euler()


def area_light(name, loc, energy, size, color, target=(0, 0, 0.25), size_y=None):
    data = bpy.data.lights.new(name, 'AREA')
    data.energy = energy
    data.color = color
    data.shape = 'RECTANGLE'
    data.size = size
    data.size_y = size_y or size
    obj = bpy.data.objects.new(name, data)
    studio.objects.link(obj)
    obj.location = loc
    point_at(obj, target)


area_light('LargeKeySoftbox', (-0.85, -0.9, 1.6), 160, 1.2, (1.0, 0.93, 0.83))
area_light('RightFillSoftbox', (1.3, -0.2, 0.95), 105, 0.9, (0.80, 0.9, 1.0))
area_light('BackEdgeStrip', (0.15, 0.85, 1.4), 180, 0.8, (0.90, 0.97, 1.0), size_y=0.35)
area_light('FrontBounce', (0.1, -1.3, 0.46), 22, 0.75, (0.9, 1.0, 1.0))
camera_data = bpy.data.cameras.new('PreviewCamera')
camera = bpy.data.objects.new('PreviewCamera', camera_data)
studio.objects.link(camera)
camera.location = (1.06, -1.46, 1.08)
point_at(camera, (0, 0.02, 0.345))
camera_data.type = 'ORTHO'
camera_data.ortho_scale = 1.22
camera_data.lens = 60
scene.camera = camera
scene.render.engine = 'CYCLES'
scene.cycles.samples = 64
scene.cycles.use_denoising = True
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    prefs.compute_device_type = 'OPTIX'
    prefs.get_devices()
    devices = [d for d in prefs.devices if d.type == 'OPTIX']
    if devices:
        for device in prefs.devices:
            device.use = device.type == 'OPTIX'
        scene.cycles.device = 'GPU'
except Exception as exc:
    print('GPU render unavailable; using CPU:', exc)
scene.render.resolution_x = 1500
scene.render.resolution_y = 1400
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.render.image_settings.color_mode = 'RGBA'
scene.render.film_transparent = False
scene.view_settings.view_transform = 'AgX'
scene.render.filepath = str(OUT / 'preview.png')

# Save with the camera view already framed and only the machine selected.
bpy.ops.object.select_all(action='DESELECT')
root.select_set(True)
bpy.context.view_layer.objects.active = root
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces.active.region_3d.view_perspective = 'CAMERA'
            area.spaces.active.shading.type = 'MATERIAL'
            area.spaces.active.overlay.show_overlays = False
bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'specimen_processor.blend'))
bpy.ops.render.render(write_still=True)
print('DELIVERABLES', str(usd_path), str(OUT / 'specimen_processor.blend'), str(OUT / 'preview.png'), flush=True)

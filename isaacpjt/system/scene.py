"""병원 씬 + 로봇 N대 구성. SimulationApp 을 만든 뒤에 import 할 것.

사람/NavMesh/라이다 경량화는 pjt_alpha/run_hospital_sim.py 와 같은 절차다
(그 파일은 단일 로봇 회귀 기준으로 그대로 둔다). 모든 변경은 세션 레이어에만 하고
USD 파일은 저장하지 않는다.

로봇 i (1부터):
    prim   /World/robot_nova (i=1, 씬에 있던 것) 또는 /World/robot_nova_{i} (복제)
    graph  /World/nova_carter_ros (i=1) 또는 /World/robot{i}_ros (복제)
    ROS    네임스페이스 robot{i} — cmd_vel, chassis/odom, tf, front_3d_lidar/lidar_points ...
           frame id 는 그대로(base_link, odom, front_3d_lidar). 로봇마다 tf 토픽이 따로다.
"""

import time
from pathlib import Path

import carb
import omni.graph.core as og
from pxr import Gf, Sdf, Usd

ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets"
SCENE_USD = str(ASSETS_DIR / "hospital_integration_human.usd")
PEOPLE_COMMAND_FILE = str(ASSETS_DIR / "people" / "hospital_integration_human_command.txt")
PEOPLE_NAMES = {"Character", "Character_01", "Character_02"}
PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.kit.scripting",
]

SOURCE_ROBOT = "/World/robot_nova"
SOURCE_GRAPH = "/World/nova_carter_ros"
LIDAR_REL = "nova_carter/chassis_link/sensors/XT_32/front_3d_lidar"
BASE_LINK_REL = "nova_carter/chassis_link/base_link"
WRIST_COLOR_CAMERA = "Camera_OmniVision_OV9782_Color"
WRIST_RESOLUTION = (640, 480)
WRIST_FRAME_SKIP = 4


def robot_prim(index):
    return SOURCE_ROBOT if index == 1 else f"{SOURCE_ROBOT}_{index}"


def robot_graph(index):
    return SOURCE_GRAPH if index == 1 else f"/World/robot{index}_ros"


def namespace(index):
    return f"robot{index}"


def enable_people_extensions(simulation_app):
    from isaacsim.core.utils.extensions import enable_extension
    for name in PEOPLE_EXTENSIONS:
        enable_extension(name)
        simulation_app.update()


def configure_people(walk_scale):
    """omni.anim.people 설정과 보행 속도 상한. 씬을 열기 전에 호출한다."""
    settings = carb.settings.get_settings()
    settings.set("/exts/omni.anim.people/command_settings/command_file_path", PEOPLE_COMMAND_FILE)
    settings.set("/exts/omni.anim.people/command_settings/number_of_loop", "inf")
    settings.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", True)
    settings.set("/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled", False)

    from omni.anim.people.scripts.commands.base_command import Command
    original_walk = Command.walk

    def slow_walk(command, delta_time):
        result = original_walk(command, delta_time)
        if command.desired_walk_speed > 0.0:
            command.character.set_variable("Walk", min(command.actual_walk_speed, walk_scale))
        return result

    Command.walk = slow_walk


def validated_behavior_prims():
    """씬의 캐릭터 스크립트가 omni.anim.people 의 character_behavior.py 인지 확인한다."""
    import omni.anim.people.scripts.character_behavior as people_behavior

    expected = Path(people_behavior.__file__).resolve()
    tail = Path(*expected.parts[-5:])
    stage = Usd.Stage.Open(SCENE_USD)
    paths = []
    for prim in stage.Traverse():
        scripts = prim.GetAttribute("omni:scripting:scripts")
        if not scripts or not scripts.Get():
            continue
        for script in scripts.Get():
            candidate = Path(script.path)
            if (not str(prim.GetPath()).startswith("/World/Characters/")
                    or len(candidate.parts) < 5 or Path(*candidate.parts[-5:]) != tail):
                raise RuntimeError(f"Unexpected USD behavior: {prim.GetPath()} {script}")
        paths.append(prim.GetPath())
    if len(paths) != len(PEOPLE_NAMES):
        raise RuntimeError(f"Expected {len(PEOPLE_NAMES)} pedestrian behaviors, found {len(paths)}")
    return paths, expected


def setup_people(stage, behavior_paths, script_path, enabled):
    """세션 레이어: 캐릭터 스크립트 경로를 이 PC 사본으로, 시작 위치를 루프 끝점으로."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "pjt_alpha"))
    from hospital_people_commands import loop_origins

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path in behavior_paths:
            stage.GetPrimAtPath(path).GetAttribute("omni:scripting:scripts").Set(
                Sdf.AssetPathArray([str(script_path)]))
        for name, position in loop_origins(PEOPLE_COMMAND_FILE, PEOPLE_NAMES).items():
            stage.GetPrimAtPath(f"/World/Characters/{name}").GetAttribute(
                "xformOp:translate").Set(Gf.Vec3d(*position))
        if not enabled:
            stage.GetPrimAtPath("/World/Characters").SetActive(False)


def bake_navmesh(simulation_app, timeout_s=30.0):
    import omni.anim.navigation.core as navigation

    interface = navigation.acquire_interface()
    interface.start_navmesh_baking()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if not interface.is_navmesh_baking():
            break
    else:
        raise RuntimeError(f"NavMesh bake timed out after {timeout_s:.0f}s")
    if interface.get_navmesh() is None:
        raise RuntimeError("NavMesh bake failed")


def set_viewport_lighting(rig="Default"):
    """뷰포트 라이팅을 Stage Lights 대신 rig 로 바꾼다(뷰포트 메뉴의 'Default' 와 같은 액션).

    씬을 연 뒤에 부른다. 손목 카메라 렌더에도 적용된다. 헤드리스는 뷰포트가 없어 경고만 남긴다."""
    import omni.kit.actions.core
    action = omni.kit.actions.core.get_action_registry().get_action(
        "omni.kit.viewport.menubar.lighting", "set_lighting_mode_rig")
    if action is None:
        carb.log_warn("viewport lighting action not found; lighting left as Stage Lights")
        return False
    action.execute(rig)
    return True


def lighten_lidar(stage, lidar_path, report_rate_hz=18000):
    """고도마다 방위 0 에 가장 가까운 발광기 하나(32채널)만 남기고 발사 빈도를 낮춘다.

    run_hospital_sim.lighten_front_lidar 와 같은 처리(2026-09-25 실측 근거는 그쪽 docstring)."""
    lidar = stage.GetPrimAtPath(lidar_path)
    prefix = "omni:sensor:Core:emitterState:s001:"
    azimuth = list(lidar.GetAttribute(prefix + "azimuthDeg").Get())
    elevation = list(lidar.GetAttribute(prefix + "elevationDeg").Get())
    keep = {}
    for index, (az, el) in enumerate(zip(azimuth, elevation)):
        key = round(el, 2)
        if key not in keep or abs(az) < abs(azimuth[keep[key]]):
            keep[key] = index
    indices = sorted(keep.values(), key=lambda i: elevation[i])
    for name in ("azimuthDeg", "elevationDeg", "fireTimeNs"):
        attribute = lidar.GetAttribute(prefix + name)
        values = list(attribute.Get())
        attribute.Set(type(attribute.Get())([values[i] for i in indices]))
    channel = lidar.GetAttribute(prefix + "channelId")
    channel.Set(type(channel.Get())(list(range(1, len(indices) + 1))))
    lidar.GetAttribute("omni:sensor:Core:numberOfEmitters").Set(len(indices))
    lidar.GetAttribute("omni:sensor:Core:numberOfChannels").Set(len(indices))
    lidar.GetAttribute("omni:sensor:Core:reportRateBaseHz").Set(report_rate_hz)
    return len(indices)


def _remap_list_op(list_op, old, new):
    """PathListOp 의 모든 항목에서 경로 접두어 old 를 new 로 바꾼다."""
    def remap(items):
        return [Sdf.Path(str(p).replace(old, new, 1)) if str(p).startswith(old) else p
                for p in items]
    if list_op.isExplicit:
        list_op.explicitItems = remap(list_op.explicitItems)
    list_op.prependedItems = remap(list_op.prependedItems)
    list_op.appendedItems = remap(list_op.appendedItems)
    list_op.deletedItems = remap(list_op.deletedItems)


def _remap_layer_paths(layer, root, old, new):
    """root 아래 모든 relationship target / attribute connection 의 접두어를 바꾼다."""
    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.RelationshipSpec):
            _remap_list_op(spec.targetPathList, old, new)
        elif isinstance(spec, Sdf.AttributeSpec):
            _remap_list_op(spec.connectionPathList, old, new)
    layer.Traverse(root, visit)


def _set_namespace(stage, graph_path, ns):
    """그래프 안 ROS 노드의 nodeNamespace 를 ns 로. 이미 값이 있으면(/front_stereo_camera) 앞에 붙인다."""
    for prim in Usd.PrimRange(stage.GetPrimAtPath(graph_path)):
        attribute = prim.GetAttribute("inputs:nodeNamespace")
        if not attribute:
            continue
        current = (attribute.Get() or "").strip("/")
        attribute.Set(f"/{ns}/{current}" if current else f"/{ns}")


def _disconnect(stage, attribute_path):
    stage.GetAttributeAtPath(attribute_path).SetConnections([])


def add_robots(stage, poses, front_camera=True):
    """poses[i-1] = (x, y, yaw_deg) 인 로봇 i 를 만들고 ROS 그래프를 네임스페이스로 복제한다.

    로봇 1 은 씬에 있던 것을 쓰고 네임스페이스만 robot1 로 바꾼다. poses[0] 이 None 이면 씬 자세 그대로,
    값이 있으면 그 자세로 옮긴다 (세션 레이어)."""
    root = stage.GetRootLayer()
    session = stage.GetSessionLayer()
    Sdf.CreatePrimInLayer(session, "/World")
    for index, pose in enumerate(poses, start=1):
        x, y, yaw_deg = pose if pose is not None else (0.0, 0.0, 0.0)
        prim, graph = robot_prim(index), robot_graph(index)
        if index > 1:
            if not Sdf.CopySpec(root, SOURCE_ROBOT, session, prim):
                raise RuntimeError(f"robot copy failed: {prim}")
            if not Sdf.CopySpec(root, SOURCE_GRAPH, session, graph):
                raise RuntimeError(f"graph copy failed: {graph}")
            _remap_layer_paths(session, graph, SOURCE_GRAPH + "/", graph + "/")
            _remap_layer_paths(session, graph, SOURCE_ROBOT + "/", prim + "/")
            _remap_layer_paths(session, prim, SOURCE_ROBOT + "/", prim + "/")
        with Usd.EditContext(stage, session):
            if index > 1 or poses[0] is not None:
                xform = stage.GetPrimAtPath(prim)
                xform.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, 0.0))
                xform.GetAttribute("xformOp:orient").Set(
                    Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg).GetQuat()))
            _set_namespace(stage, graph, namespace(index))
            if not front_camera:
                # 병원 주행은 전방 스테레오 카메라를 쓰지 않는다. render product 를 안 만들면 렌더 부하가 준다
                _disconnect(stage, f"{graph}/ros_lidars/rp_cam_front.inputs:execIn")
                _disconnect(stage, f"{graph}/ros_lidars/pub_cam_front.inputs:execIn")
            lighten_lidar(stage, f"{prim}/{LIDAR_REL}")


def find_wrist_camera(stage, index):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_prim(index))):
        if prim.GetName() == WRIST_COLOR_CAMERA and prim.GetTypeName() == "Camera":
            return str(prim.GetPath())
    raise RuntimeError(f"{WRIST_COLOR_CAMERA} not found under {robot_prim(index)}")


def build_wrist_camera_graph(stage, index, frame_skip=WRIST_FRAME_SKIP):
    """손목 D455 컬러/깊이/camera_info 발행 (pick_and_place_detection.build_detection_graph 의 카메라 부분).

    토픽: /robot{i}/wrist_camera/color/image_raw, depth/image_raw, color/camera_info"""
    from isaacsim.core.utils.prims import set_targets

    graph = f"/World/robot{index}_wrist"
    ns = f"/{namespace(index)}"
    og.Controller.edit(
        {"graph_path": graph, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("rp", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_color", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("pub_info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "rp.inputs:execIn"),
                ("rp.outputs:execOut", "pub_color.inputs:execIn"),
                ("rp.outputs:execOut", "pub_depth.inputs:execIn"),
                ("rp.outputs:execOut", "pub_info.inputs:execIn"),
                ("rp.outputs:renderProductPath", "pub_color.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_depth.inputs:renderProductPath"),
                ("rp.outputs:renderProductPath", "pub_info.inputs:renderProductPath"),
                ("context.outputs:context", "pub_color.inputs:context"),
                ("context.outputs:context", "pub_depth.inputs:context"),
                ("context.outputs:context", "pub_info.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("rp.inputs:width", WRIST_RESOLUTION[0]),
                ("rp.inputs:height", WRIST_RESOLUTION[1]),
                ("pub_color.inputs:type", "rgb"),
                ("pub_color.inputs:nodeNamespace", ns),
                ("pub_color.inputs:topicName", "wrist_camera/color/image_raw"),
                ("pub_color.inputs:frameId", "d455_color_optical_frame"),
                ("pub_color.inputs:frameSkipCount", frame_skip),
                ("pub_depth.inputs:type", "depth"),
                ("pub_depth.inputs:nodeNamespace", ns),
                ("pub_depth.inputs:topicName", "wrist_camera/depth/image_raw"),
                ("pub_depth.inputs:frameId", "d455_color_optical_frame"),
                ("pub_depth.inputs:frameSkipCount", frame_skip),
                ("pub_info.inputs:nodeNamespace", ns),
                ("pub_info.inputs:topicName", "wrist_camera/color/camera_info"),
                ("pub_info.inputs:frameId", "d455_color_optical_frame"),
                ("pub_info.inputs:frameSkipCount", frame_skip),
            ],
        },
    )
    set_targets(prim=stage.GetPrimAtPath(f"{graph}/rp"), attribute="inputs:cameraPrim",
                target_prim_paths=[find_wrist_camera(stage, index)])


def base_link_pose(index):
    """로봇 i 의 base_link 월드 자세 (x, y, yaw_deg)."""
    import math
    from isaacsim.core.prims import XFormPrim

    positions, quaternions = XFormPrim(f"{robot_prim(index)}/{BASE_LINK_REL}").get_world_poses()
    w, qx, qy, qz = (float(v) for v in quaternions[0])
    yaw = math.degrees(math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))
    return float(positions[0][0]), float(positions[0][1]), yaw

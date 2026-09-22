"""integration_human.usd 를 띄우고 ROS2 브리지와 사람 애니메이션만 돌린다.

    isaac_python run_human_scene.py

Nav2 주행 실험용 최소 실행기다. 팔/그리퍼 '동작'은 없고 드라이브만 잡아
중력에 처지지 않게만 한다. 픽앤플레이스 시퀀스·IK 솔버는 일절 없다.

씬 구성 (integration_human.usd)
    /World/robot_nova/nova_carter   노바카터 + M0609 (하나의 아티큘레이션)
    /World/Character...             omni.anim.people 캐릭터
    ActionGraph                     USD 에 구워져 있음. /clock, TF, /chassis/odom,
                                    라이다 발행 + cmd_vel 구독 (네임스페이스 없음)

실행 순서에 제약이 많다. 아래 주석의 "반드시" 를 지킬 것.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import time
from pathlib import Path

import isaacsim

# pick_and_place 는 import 시점에 자기 SimulationApp 을 만들려 한다. 위에서 만든 것을 돌려준다.
isaacsim.SimulationApp = lambda *args, **kwargs: simulation_app

M0609_DIR = Path(__file__).resolve().parent.parent / "M0609"
sys.path[:0] = [str(M0609_DIR / "move"), str(M0609_DIR / "teleop")]

# standalone 은 GUI 와 달리 확장을 최소만 로드한다. 반드시 스테이지를 열기 전에 켠다.
from isaacsim.core.utils.extensions import enable_extension

# omni.anim.people 의 캐릭터 behavior 스크립트가 omni.anim.graph.core 등을 import 한다.
# 안 켜면 "ModuleNotFoundError: No module named 'omni.anim.graph.core'" 로 사람이 안 움직인다.
# 목록은 NVIDIA 공식 예제(standalone_examples/testing/isaacsim.ros2.bridge/test_people_sim.py) 기준.
PEOPLE_EXTENSIONS = [
    "omni.anim.people",
    "omni.anim.navigation.bundle",
    "omni.anim.timeline",
    "omni.anim.graph.bundle",
    "omni.anim.graph.core",
    "omni.anim.graph.ui",
    "omni.anim.retarget.bundle",
    "omni.anim.retarget.core",
    "omni.anim.retarget.ui",
    "omni.kit.scripting",
]
for _ext in PEOPLE_EXTENSIONS:
    enable_extension(_ext)
    simulation_app.update()

# 씬의 ActionGraph 가 /clock, TF, /chassis/odom, 라이다를 내보내려면 브리지가 켜져 있어야 한다.
# 켜지 않으면 Nav2 가 센서 데이터를 전혀 못 받는다.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# people/config.yaml 은 Actor SDG 확장(UI)이 읽는 파일이라 standalone 에서는 아무도 안 읽는다.
# 캐릭터 behavior 스크립트는 아래 carb 설정에서 명령 파일 경로를 가져오고(기본값 ""),
# 초기화 시점에 한 번만 읽으므로 반드시 open_stage() 앞에서 설정해야 한다.
import carb

PEOPLE_COMMAND_FILE = "/home/rokey/cobot3_ws/isaacpjt/assets/people/command.txt"
_settings = carb.settings.get_settings()
_settings.set("/exts/omni.anim.people/command_settings/command_file_path", PEOPLE_COMMAND_FILE)
_settings.set("/exts/omni.anim.people/command_settings/number_of_loop", "inf")  # 기본 "0" 은 1회 재생 후 정지
_settings.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", True)
_settings.set("/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled", True)
simulation_app.update()

import omni.usd
from pxr import Sdf, Usd

from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import is_stage_loading, open_stage

# 팔 드라이브 설정만 빌려 쓴다. 픽앤플레이스 시퀀스는 쓰지 않는다.
import pick_and_place as _pnp


USD_PATH = Path("/home/rokey/cobot3_ws/isaacpjt/assets/integration_human.usd")

MOVE_ROOT_PATH = "/World/robot_nova"
ROBOT_PRIM_PATH = MOVE_ROOT_PATH + "/nova_carter"
# 카터와 팔이 하나의 아티큘레이션이다. 드라이브 설정은 팔이 든 nova_carter 서브트리 기준,
# Articulation 등록은 실제 루트인 chassis_link 기준 (바퀴로 주행하려면 루트가 리지드 바디여야 한다).
ART_ROOT_PATH = ROBOT_PRIM_PATH + "/chassis_link"

# 후방 2D 라이다 (스톡 Nova Carter 의 SLAMTEC RPLIDAR S2E. robot_nova.usd 에서 위치만 override).
# 프로파일 실측: numberOfEmitters 1, elevationDeg [0] -> 완전한 2D, farRange 30m, 10Hz.
# 2D 이므로 point_cloud 가 아니라 laser_scan 으로 바로 발행한다. pointcloud_to_laserscan 불필요.
REAR_RPLIDAR_PATH = ROBOT_PRIM_PATH + "/chassis_link/sensors/rear_RPLidar"
REAR_TOPIC = "scan_rear"
REAR_FRAME = "rear_rplidar"      # TF 프레임 이름. 아래에서 isaac:nameOverride 로 고정한다.

LIDAR_GRAPH_PATH = "/World/nova_carter_ros/ros_lidars"
TF_SENSORS_NODE = "/World/nova_carter_ros/transform_tree_odometry/tf_sensors"


def section(title):
    print(f"\n=== {title} " + "=" * max(0, 50 - len(title)))


def bake_navmesh(timeout_s=30.0):
    """NavMesh 를 베이크하고 완료를 기다린다.

    베이크 결과는 USD 에 저장되지 않는다(스키마에 담을 프림 타입 자체가 없다).
    GUI 는 autoRebakeOnChanges 로 알아서 다시 굽지만 standalone 은 직접 호출해야 한다.
    안 구우면 캐릭터의 GoTo 가 전부 'invalid command' 로 거부돼 사람이 제자리에 선다.
    """
    try:
        import omni.anim.navigation.core as nav
    except ImportError as e:
        print(f"   navmesh      건너뜀 (omni.anim.navigation.core 없음: {e})")
        return False

    inav = nav.acquire_interface()
    inav.start_navmesh_baking()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        simulation_app.update()
        if not inav.is_navmesh_baking():
            break
    else:
        print(f"   navmesh      베이크 시간 초과 ({timeout_s:.0f}s)")
        return False

    if inav.get_navmesh() is None:
        print("   navmesh      실패 (get_navmesh() 가 None). NavMeshVolume 범위와 "
              "agentMinIslandRadius 를 확인할 것")
        return False
    print("   navmesh      베이크 완료")
    return True


def find_rtx_lidar(stage, root_path):
    """root_path 하위에서 RTX 라이다 프림을 찾는다.

    스톡 nova_carter.usd 가 온라인 에셋이라 오프라인에서 rear_RPLidar 하위 프림 이름을
    확인할 수 없다. robot_nova.usd 는 위치 override(over)만 갖고 있다.
    그래서 이름을 하드코딩하지 않고 타입으로 찾는다.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):                       # 1순위: OmniLidar 타입
        if prim.GetTypeName() == "OmniLidar":
            return prim
    for prim in Usd.PrimRange(root):                       # 2순위: 센서 스키마 속성 보유
        if any(a.GetName().startswith("omni:sensor:") for a in prim.GetAttributes()):
            return prim
    return None


def ensure_rear_lidar_graph(stage):
    """후방 2D 라이다의 ROS2 발행 노드와 TF 를 액션그래프에 추가한다 (멱등).

    integration_human.usd 를 직접 고치지 않는다. 사용자가 GUI 에서 편집 중인 파일이라
    충돌 위험이 있고 되돌리기도 번거롭다. 동작이 확인되면 GUI 에서 저장하면 된다.

    기존 전방 라이다와 같은 패턴으로 노드 2개만 추가하고
    tick / one_frame / context / qos 는 기존 것을 재사용한다.

        one_frame.outputs:step -> rp_rear_2d(IsaacCreateRenderProduct)
                                     -> pub_rear_2d(ROS2RtxLidarHelper, type=laser_scan)
    """
    import omni.graph.core as og

    lidar = find_rtx_lidar(stage, REAR_RPLIDAR_PATH)
    if lidar is None:
        print(f"   rear lidar   못 찾음: {REAR_RPLIDAR_PATH} 하위에 RTX 라이다가 없다.")
        print("                robot_nova.usd 의 rear_RPLidar 가 페이로드로 로드됐는지 확인할 것.")
        return False
    lidar_path = str(lidar.GetPath())
    print(f"   rear lidar   {lidar_path}")

    # TF 프레임 이름을 고정한다. ROS2PublishTransformTree 는 프림 이름(또는 nameOverride)을
    # 프레임 이름으로 쓴다. 발행 노드의 frameId 와 반드시 같아야 costmap 이 스캔을 변환할 수 있다.
    ov = lidar.GetAttribute("isaac:nameOverride")
    if not ov:
        ov = lidar.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
    if ov.Get() != REAR_FRAME:
        ov.Set(REAR_FRAME)
        print(f"   rear frame   isaac:nameOverride = {REAR_FRAME}")

    # --- 1) 발행 노드 2개
    if stage.GetPrimAtPath(LIDAR_GRAPH_PATH + "/pub_rear_2d").IsValid():
        print("   rear graph   이미 있음 (건너뜀)")
    else:
        g = LIDAR_GRAPH_PATH
        og.Controller.edit(g, {
            og.Controller.Keys.CREATE_NODES: [
                ("rp_rear_2d", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("pub_rear_2d", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("rp_rear_2d.inputs:width", 1),
                ("rp_rear_2d.inputs:height", 1),
                ("pub_rear_2d.inputs:topicName", REAR_TOPIC),
                ("pub_rear_2d.inputs:frameId", REAR_FRAME),
                ("pub_rear_2d.inputs:type", "laser_scan"),   # 2D. 전방(3D)은 point_cloud
                ("pub_rear_2d.inputs:fullScan", True),       # 360도 한 바퀴를 한 메시지로
                ("pub_rear_2d.inputs:enabled", True),
                ("pub_rear_2d.inputs:resetSimulationTimeOnStop", False),
            ],
            og.Controller.Keys.CONNECT: [
                (g + "/one_frame.outputs:step", "rp_rear_2d.inputs:execIn"),
                ("rp_rear_2d.outputs:execOut", "pub_rear_2d.inputs:execIn"),
                ("rp_rear_2d.outputs:renderProductPath",
                 "pub_rear_2d.inputs:renderProductPath"),
                (g + "/context.outputs:context", "pub_rear_2d.inputs:context"),
                (g + "/qos.outputs:qosProfile", "pub_rear_2d.inputs:qosProfile"),
            ],
        })
        # cameraPrim 은 릴레이션이라 SET_VALUES 로는 안 붙는다. USD API 로 직접 건다.
        stage.GetPrimAtPath(g + "/rp_rear_2d").GetRelationship(
            "inputs:cameraPrim").SetTargets([Sdf.Path(lidar_path)])
        print(f"   rear graph   생성 -> /{REAR_TOPIC} (laser_scan, frame={REAR_FRAME})")

    # --- 2) TF. 이게 빠지면 토픽은 나오는데 costmap 이 전부 버린다 (조용한 실패)
    tf = stage.GetPrimAtPath(TF_SENSORS_NODE)
    if not tf.IsValid():
        print(f"   tf_sensors   노드 없음: {TF_SENSORS_NODE}")
        return False
    rel = tf.GetRelationship("inputs:targetPrims")
    targets = list(rel.GetTargets())
    if Sdf.Path(lidar_path) in targets:
        print("   tf_sensors   rear 라이다 이미 포함")
    else:
        rel.SetTargets(targets + [Sdf.Path(lidar_path)])
        print(f"   tf_sensors   rear 라이다 추가 (대상 {len(targets)} -> {len(targets) + 1})")
        print(f"                확인: ros2 run tf2_ros tf2_echo base_link {REAR_FRAME}")
    return True


def report_action_graph(stage):
    """ROS2 발행/구독 노드가 실제로 씬에 있는지 이름만 훑어 보고한다.

    Nav2 가 안 붙을 때 "브리지가 꺼진 건지 그래프가 없는 건지"를 먼저 가르기 위한 것.
    """
    names = []
    for prim in stage.Traverse():
        t = str(prim.GetTypeName())
        if "Ros2" in t or "ROS2" in t:
            names.append(f"{prim.GetName()} ({t})")
    if names:
        print(f"   ROS2 노드    {len(names)}개")
        for n in names[:12]:
            print(f"                {n}")
    else:
        print("   ROS2 노드    0개 — ActionGraph 가 없거나 타입 이름이 다르다. "
              "Nav2 가 센서를 못 받으면 여기부터 확인할 것")


def main():
    if not USD_PATH.is_file():
        raise FileNotFoundError(f"USD file was not found: {USD_PATH}")

    section("STAGE")
    print(f"   scene        {USD_PATH.name}")
    if not open_stage(str(USD_PATH)):
        raise RuntimeError(f"Could not open USD: {USD_PATH}")
    while is_stage_loading():
        simulation_app.update()

    bake_navmesh()

    stage = omni.usd.get_context().get_stage()
    report_action_graph(stage)

    section("REAR LIDAR")
    ensure_rear_lidar_graph(stage)

    world = World(stage_units_in_meters=1.0)

    section("ROBOT")
    # 팔이 중력에 처지면 차체 무게중심이 바뀌어 주행이 흔들린다. 드라이브만 잡아 자세를 유지시킨다.
    _pnp.ROBOT_PRIM_PATH = ROBOT_PRIM_PATH
    _pnp.setup_arm_drives()
    _pnp.setup_gripper_drive()

    # ActionGraph 의 ArticulationController 가 cmd_vel 로 바퀴를 돌리려면
    # 아티큘레이션이 초기화돼 있어야 한다. 루트는 리지드 바디인 chassis_link.
    robot = world.scene.add(Robot(prim_path=ART_ROOT_PATH, name="nova_carter"))

    world.reset()
    print(f"   articulation {ART_ROOT_PATH}  dof={robot.num_dof}")

    section("RUN")
    print("   Nav2 를 띄우고 주행 명령을 보내면 된다. 종료는 창 닫기 또는 Ctrl+C.")
    try:
        while simulation_app.is_running():
            world.step(render=True)
    except KeyboardInterrupt:
        print("\n   중단됨")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
